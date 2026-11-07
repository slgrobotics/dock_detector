"""Offline planar LiDAR model for fixed-length bay-panel comparisons."""
import math
import numpy as np


def panel_polygons(flare, length=.5, thickness=.05, shape='trapezoid'):
    if shape not in ('trapezoid', 'v'):
        raise ValueError('shape must be trapezoid or v')
    """Rear-face origin (0,0), opening +X; same construction as the SDF."""
    specs = [(( -thickness/2, 0.), math.pi/2)] if shape == 'trapezoid' else []
    rear_half_width = length/2 if shape == 'trapezoid' else 0.
    for sign in (-1, 1):
        specs.append(((length/2*math.cos(flare)-thickness/2*math.sin(flare),
                       sign*(rear_half_width+length/2*math.sin(flare)+thickness/2*math.cos(flare))),
                      sign*flare))
    polygons = []
    for center, angle in specs:
        d = np.array([math.cos(angle), math.sin(angle)])*length/2
        n = np.array([-math.sin(angle), math.cos(angle)])*thickness/2
        center = np.array(center)
        polygons.append(np.array([center-d-n, center+d-n, center+d+n, center-d+n]))
    return polygons


def polygons_intersect(a, b):
    """Convex SAT, treating contact as collision."""
    for polygon in (a, b):
        for edge in np.roll(polygon, -1, axis=0)-polygon:
            axis = np.array([-edge[1], edge[0]])
            pa, pb = a@axis, b@axis
            if pa.max() < pb.min()-1e-10 or pb.max() < pa.min()-1e-10:
                return False
    return True


def footprint_free(x, y, yaw, flare, footprint, shape='trapezoid', length=.5):
    c, s = math.cos(yaw), math.sin(yaw)
    body = np.asarray(footprint)@np.array([[c, s], [-s, c]])+[x, y]
    return not any(polygons_intersect(body, p) for p in panel_polygons(flare, shape=shape, length=length))


def simulate_scan(x, y, yaw, flare, rng, *, samples=360, noise=.01,
                  min_range=.3, max_range=24., blind_degrees=5., laser_x=.05, shape='trapezoid', length=.5):
    """First-hit wall-box rays; unseen/too-close hits never reveal surfaces behind.

    Returns noisy world points and clean support on the three inner faces.
    All coordinates are relative to the inner rear-face centre.
    """
    angles = np.linspace(-math.pi+math.radians(blind_degrees/2),
                         math.pi-math.radians(blind_degrees/2), samples)+yaw
    rays = np.c_[np.cos(angles), np.sin(angles)]
    origin = np.array([x+laser_x*math.cos(yaw), y+laser_x*math.sin(yaw)])
    polygons = panel_polygons(flare, shape=shape, length=length)
    segments = np.array([[p[i], p[(i+1)%4]] for p in polygons for i in range(4)])
    edge = segments[:, 1]-segments[:, 0]
    offset = segments[:, 0]-origin
    cross = lambda a,b: a[...,0]*b[...,1]-a[...,1]*b[...,0]
    denom = cross(rays[:,None,:], edge[None,:,:])
    safe = np.where(abs(denom)>1e-12, denom, np.nan)
    distance = cross(offset, edge)[None,:]/safe
    u = cross(offset[None,:,:], rays[:,None,:])/safe
    distance = np.where((distance>0)&(u>=0)&(u<=1), distance, np.inf)
    nearest = np.min(distance, axis=1)
    visible = np.isfinite(nearest)&(nearest>=min_range)&(nearest<=max_range)
    clean = origin+nearest[visible,None]*rays[visible]
    # Draw one error per ray, including rays without hits, to pair RNG streams
    # consistently across flare angles at the same pose/repetition.
    errors = rng.normal(0., noise, samples)
    measured = nearest[visible]+errors[visible]
    usable = (measured>=min_range)&(measured<=max_range)
    points = origin+measured[usable,None]*rays[visible][usable]
    clean = clean[usable]
    from .refinement import template, projections
    inner = template(length, length*math.cos(flare), flare) if shape == 'trapezoid' else np.array([
        [[0.,0.],[length*math.cos(flare), sign*length*math.sin(flare)]] for sign in (-1,1)])
    closest, along = projections(clean, inner)
    distances = np.linalg.norm(clean[:,None,:]-closest, axis=2)
    labels = distances.argmin(axis=1)
    support = []
    for panel in range(len(inner)):
        mask=(labels==panel)&(distances[:,panel]<1e-7)
        count=int(mask.sum())
        span=float(np.ptp(along[mask,panel])*length) if count>1 else 0.
        support.append((count, span))
    return points, support


def polygon_clearance(a, b):
    """Minimum Euclidean separation of convex polygons; zero on contact/overlap."""
    if polygons_intersect(a, b):
        return 0.
    distances=[]
    for points,polygon in ((a,b),(b,a)):
        edge=np.roll(polygon,-1,axis=0)-polygon
        delta=points[:,None,:]-polygon[None,:,:]
        t=np.clip(np.sum(delta*edge,axis=2)/np.sum(edge*edge,axis=1),0.,1.)
        nearest=polygon[None,:,:]+t[:,:,None]*edge
        distances.append(float(np.linalg.norm(points[:,None,:]-nearest,axis=2).min()))
    return min(distances)


def footprint_clearance(x,y,yaw,flare,footprint,shape='v',length=.7):
    c,s=math.cos(yaw),math.sin(yaw)
    body=np.asarray(footprint)@np.array([[c,s],[-s,c]])+[x,y]
    return min(polygon_clearance(body,p) for p in panel_polygons(flare,length=length,shape=shape))
