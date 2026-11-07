#!/usr/bin/env python3
"""Offline angle comparison; never imports ROS or commands the simulator."""
import argparse,concurrent.futures,csv,hashlib,json,math,os,time
from pathlib import Path
import numpy as np
import yaml
from dock_detector.geometry import detect_trapezoid,detect_v,angle_error
from dock_detector.simulation import simulate_scan,footprint_free

ANGLES=(20,25,30,35,40,45)
HEADINGS=(-20,-10,0,10,20)
REPEATS=3
PACKAGE=Path(__file__).resolve().parents[1]
CONFIG=yaml.safe_load((PACKAGE/'config/seggy.yaml').read_text())['dock_detector']['ros__parameters']
NAV=PACKAGE.parent/'articubot_one/robots/seggy/config/nav2_params.yaml'
FOOTPRINT=json.loads(yaml.safe_load(NAV.read_text())['local_costmap']['local_costmap']['ros__parameters']['footprint'])
KEYS=('distance_threshold','min_points','min_length','max_length','angle_tolerance',
      'position_tolerance','yaw_tolerance','dimension_tolerance','max_corner_gap')


def evaluate(task):
    degree,x,y,heading,index,kind,repeat,shape,compare_v,angles,v_length=task
    flare=math.radians(degree);yaw=math.radians(heading)
    length=v_length if shape=='v' else .5
    clear=footprint_free(x,y,yaw,flare,FOOTPRINT,shape=shape,length=length)
    common=(all(footprint_free(x,y,yaw,math.radians(d),FOOTPRINT,shape='v',length=v_length) for d in angles)
            and footprint_free(x,y,yaw,math.radians(32.5),FOOTPRINT)) if compare_v else all(
            footprint_free(x,y,yaw,math.radians(d),FOOTPRINT) for d in angles)
    row=dict(kind=kind,shape=shape,panel_length_m=length,angle_deg=degree,x_rear=x,y_rear=y,x_world=x-1.425,y_world=y+3,
             heading_deg=heading,index=index,repeat=repeat,clear=clear,common_clear=common)
    if not clear:return {**row,'matched':False,'good':False,'reason':'footprint_collision'}
    # Identical ray-noise streams for each angle at a given pose/repetition.
    rng=np.random.default_rng(20261106+index*101+repeat*100003+(0 if kind=='grid' else 9000000))
    points,support=simulate_scan(x,y,yaw,flare,rng,shape=shape,length=length)
    points=points[np.linalg.norm(points,axis=1)<=CONFIG['search_radius']]
    stats={}
    if shape == 'v':
        vkeys=tuple(k for k in KEYS if k not in ('dimension_tolerance','max_corner_gap'))
        result=detect_v(points,(0.,0.),0.,expected_angle=2*flare,
                        max_apex_gap=CONFIG['max_apex_gap'],**{k:CONFIG[k] for k in vkeys})
        stats['reason']='matched' if result is not None else 'no_v_candidate'
    else:
        result=detect_trapezoid(points,(0.,0.),0.,width=.5,depth=.5*math.cos(flare),
            flare_angle=flare,max_line_gap=CONFIG['trapezoid_max_line_gap'],diagnostics=stats,
            joint_refinement=CONFIG['joint_refinement'],**{k:CONFIG[k] for k in KEYS})
    row.update(matched=result is not None,good=False,reason=stats['reason'],roi_points=len(points),
               confidence=stats.get('confidence'),
               visible_faces=all(n>=CONFIG['min_points'] and span>=CONFIG['min_length'] for n,span in support))
    if shape == 'trapezoid':
        row['visible_three_faces'] = row['visible_faces']
    for name,(n,span) in zip(('rear','lower','upper') if shape=='trapezoid' else ('lower','upper'),support):
        row[name+'_returns']=n;row[name+'_span_m']=span
    if result is not None:
        # Compare the published parking point, not just fitted rear-face centre.
        parking=result.apex+.425*np.array([math.cos(result.yaw),math.sin(result.yaw)])
        row['position_error_m']=float(np.linalg.norm(parking-[.425,0]))
        row['yaw_error_deg']=abs(math.degrees(angle_error(result.yaw,0.)))
        row['good']=row['position_error_m']<=.03 and row['yaw_error_deg']<=3.
    return row


def run(out,workers,shape,angles=ANGLES,v_length=.5):
    out.mkdir(parents=True,exist_ok=False)
    (out/'sweep_flare.py').write_text(Path(__file__).read_text())
    sources=[PACKAGE/'dock_detector'/name for name in ('geometry.py','refinement.py','simulation.py')]
    sources+=[PACKAGE/'config/seggy.yaml',NAV,PACKAGE.parent/'articubot_one/description/lidar.xacro',PACKAGE.parent/'articubot_one/robots/seggy/description/robot.urdf.xacro']
    meta={'shape':shape,'comparison_baseline':{'shape':'trapezoid','flare_deg':32.5} if shape=='v' else None,
          'angles_deg':angles,'headings_deg':HEADINGS,'repeats':REPEATS,'grid_step_m':.1,
          'grid_world_x':[-1.4,.4],'grid_world_y':[2.4,3.6],'rear_world':[-1.425,3.],
          'panel_lengths_m':[v_length,v_length] if shape=='v' else [.5,.5,.5],'panel_thickness_m':.05,'ray_samples':360,
          'angular_span_deg':355,'min_range_m':.3,'max_range_m':24,'range_noise_sigma_m':.01,
          'laser_x_from_base_link_m':.05,'sensor_rate_hz':6,'detector_config':CONFIG,'footprint':FOOTPRINT,
          'source_sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
          'limitations':['Ideal planar first-hit geometry; no robot self-occlusion beyond configured rear gap.',
             'No unrelated world clutter, reflectance, beam divergence, scan-motion distortion, TF or processing latency.',
             'Perfect coarse dock prior; fixed existing detector thresholds. Three noise realizations per grid pose.',
             'Path gaps reflect ideal 6 Hz synthetic scans, not real-time controller/network timing.']}
    (out/'metadata.json').write_text(json.dumps(meta,indent=2))
    positions=[(world_x/10+1.425,world_y/10) for world_x in range(-14,5) for world_y in range(-6,7)]
    tasks=[]
    scenarios=[(shape,d) for d in angles]+([('trapezoid',32.5)] if shape=='v' else [])
    for current_shape,degree in scenarios:
        for i,(x,y) in enumerate(positions):
            for j,heading in enumerate(HEADINGS):
                for repeat in range(REPEATS):tasks.append((degree,x,y,heading,i*len(HEADINGS)+j,'grid',repeat,current_shape,shape=='v',tuple(angles),v_length))
    # Same three parallel reverse paths from staging to final reference. At
    # 0.035 m/s and 6 Hz the scan spacing is 5.83 mm, independently of the grid.
    for current_shape,degree in scenarios:
        for path,y in enumerate((-.1,0.,.1)):
            for i,x in enumerate(np.arange(1.225,.425-.0001,-.035/6)):
                tasks.append((degree,float(x),y,0,i+path*1000,'path',path,current_shape,shape=='v',tuple(angles),v_length))
    start=time.monotonic();rows=[]
    with (out/'samples.jsonl').open('w',buffering=1) as f,concurrent.futures.ProcessPoolExecutor(max_workers=workers) as pool:
        for row in pool.map(evaluate,tasks,chunksize=12):
            rows.append(row);f.write(json.dumps(row)+'\n')
            if len(rows)%500==0:
                print(json.dumps({'completed':len(rows),'total':len(tasks),'elapsed_s':round(time.monotonic()-start,1)}),flush=True)
                (out/'progress.json').write_text(json.dumps({'completed':len(rows),'total':len(tasks)}))
    fields=sorted(set().union(*(r.keys() for r in rows)))
    with (out/'samples.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(rows)
    (out/'progress.json').write_text(json.dumps({'completed':len(rows),'total':len(tasks),'finished':True}))
    print('FINISHED '+str(out),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);p.add_argument('--workers',type=int,default=4);p.add_argument('--shape',choices=('trapezoid','v'),default='trapezoid')
    p.add_argument('--angles',type=float,nargs='+',default=list(ANGLES))
    p.add_argument('--v-panel-length',type=float,default=.5)
    a=p.parse_args()
    if not all(math.isfinite(d) and 0<d<90 for d in a.angles):p.error('angles must lie between 0 and 90 degrees')
    if not math.isfinite(a.v_panel_length) or a.v_panel_length<=0:p.error('v-panel-length must be positive')
    run(a.output,a.workers,a.shape,a.angles,a.v_panel_length)
