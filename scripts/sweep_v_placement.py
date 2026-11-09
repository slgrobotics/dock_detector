#!/usr/bin/env python3
"""Fixed 55-degree / 70 cm V; vary apex standoff behind the final point only."""
import concurrent.futures,csv,hashlib,json,math,time
from pathlib import Path
import argparse,numpy as np,yaml
from dock_detector.core.geometry import detect_v,angle_error
from dock_detector.core.simulation import simulate_scan,footprint_free,footprint_clearance
PACKAGE=Path(__file__).resolve().parents[1]
from dock_detector.shape_parameters import geometry_parameters
CFG=yaml.safe_load((PACKAGE/'config/seggy_t.yaml').read_text())['dock_detector']['ros__parameters']
_flat = {}
for key, value in CFG.items():
    if isinstance(value, dict):
        _flat.update({key+"."+name: item for name, item in value.items()})
    else:
        _flat[key] = value
CFG = geometry_parameters(_flat)
NAV=PACKAGE.parent/'articubot_one/robots/seggy/config/nav2_params.yaml'
BODY=json.loads(yaml.safe_load(NAV.read_text())['local_costmap']['local_costmap']['ros__parameters']['footprint'])
DISTANCES=tuple(np.round(np.arange(.425,.826,.025),3))
FLARE=math.radians(55);LENGTH=.7
KEYS=('distance_threshold','min_points','min_length','max_length','angle_tolerance','position_tolerance','yaw_tolerance')

def evaluate(task):
    d,dx,y,heading,index,repeat,kind=task
    yaw=math.radians(heading);x=d+dx
    clear=footprint_free(x,y,yaw,FLARE,BODY,shape='v',length=LENGTH)
    common=all(footprint_free(v+dx,y,yaw,FLARE,BODY,shape='v',length=LENGTH) for v in DISTANCES)
    row=dict(standoff_m=d,x_from_final=dx,y_from_final=y,heading_deg=heading,index=index,
             repeat=repeat,kind=kind,clear=clear,common_clear=common,matched=False,good=False)
    if not clear:return {**row,'reason':'footprint_collision'}
    rng=np.random.default_rng(20261106+index*101+repeat*100003+(0 if kind=='grid' else 9000000))
    points,support=simulate_scan(x,y,yaw,FLARE,rng,shape='v',length=LENGTH)
    points=points[np.linalg.norm(points,axis=1)<=CFG['search_radius']]
    result=detect_v(points,(0.,0.),0.,expected_angle=2*FLARE,max_apex_gap=CFG['max_apex_gap'],**{k:CFG[k] for k in KEYS})
    row.update(matched=result is not None,reason='matched' if result is not None else 'no_v_candidate',
               visible_faces=all(n>=5 and span>=.12 for n,span in support))
    if result is not None:
        target=result.apex+d*np.array([math.cos(result.yaw),math.sin(result.yaw)])
        row['position_error_m']=float(np.linalg.norm(target-[d,0.]));row['yaw_error_deg']=abs(math.degrees(angle_error(result.yaw,0.)))
        row['good']=row['position_error_m']<=.03 and row['yaw_error_deg']<=3.
    return row


def run(out,workers):
    out.mkdir(parents=True,exist_ok=False)
    meta={'flare_per_arm_deg':55,'panel_length_m':.7,'panel_thickness_m':.05,'standoffs_m':DISTANCES,
          'grid_x_from_final_m':[0,.8],'grid_y_from_final_m':[-.2,.2],'grid_step_m':.1,
          'headings_deg':[-20,-10,0,10,20],'noise_repeats':3,'lidar':{'samples':360,'view_deg':355,'min_range_m':.3,'max_range_m':24,'noise_sigma_m':.01,'laser_forward_offset_m':.05},
          'final_pose_tolerance_window':{'x_m':[-.05,0,.05],'y_m':[-.1,-.05,0,.05,.1],'yaw_deg':[-10,0,10]},
          'required_worst_clearance_m':.05,'final_world':[-1,3,0],'footprint':BODY,'detector_config':CFG}
    sources=[PACKAGE/'dock_detector'/n for n in ('simulation.py','geometry.py')]+[PACKAGE/'config/seggy_t.yaml',NAV]
    meta['source_sha256']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    (out/'metadata.json').write_text(json.dumps(meta,indent=2));(out/'sweep_v_placement.py').write_text(Path(__file__).read_text())
    for n in ('simulation.py','geometry.py'):(out/n).write_text((PACKAGE/'dock_detector'/n).read_text())
    placements=[]
    for d in DISTANCES:
        margins=[footprint_clearance(d+x,y,math.radians(h),FLARE,BODY) for x in (-.05,0,.05) for y in (-.1,-.05,0,.05,.1) for h in (-10,0,10)]
        placements.append({'standoff_m':d,'nominal_clearance_m':footprint_clearance(d,0,0,FLARE,BODY),
                           'worst_tolerance_clearance_m':min(margins),'tolerance_collisions':sum(m<=1e-10 for m in margins),
                           'apex_world':[-1-d,3.]})
    (out/'placement_geometry.json').write_text(json.dumps(placements,indent=2))
    tasks=[]
    for d in DISTANCES:
        for i,dx in enumerate(np.arange(0,.801,.1)):
            for j,y in enumerate(np.arange(-.2,.201,.1)):
                for k,h in enumerate((-20,-10,0,10,20)):
                    for repeat in range(3):tasks.append((float(d),float(dx),float(y),h,(i*5+j)*5+k,repeat,'grid'))
        for path,y in enumerate((-.1,0,.1)):
            for i,dx in enumerate(np.arange(.8,-.0001,-.035/6)):tasks.append((float(d),float(dx),y,0,i+path*1000,path,'path'))
    rows=[];start=time.monotonic()
    with (out/'samples.jsonl').open('w',buffering=1) as stream,concurrent.futures.ProcessPoolExecutor(max_workers=workers) as pool:
        for row in pool.map(evaluate,tasks,chunksize=12):
            rows.append(row);stream.write(json.dumps(row)+'\n')
            if len(rows)%1000==0:print(json.dumps({'completed':len(rows),'total':len(tasks),'elapsed_s':round(time.monotonic()-start,1)}),flush=True)
    fields=sorted(set().union(*(r.keys() for r in rows)))
    with (out/'samples.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(rows)
    print('FINISHED '+str(out),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);p.add_argument('--workers',type=int,default=4);a=p.parse_args();run(a.output,a.workers)
