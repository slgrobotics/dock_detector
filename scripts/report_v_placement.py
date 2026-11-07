#!/usr/bin/env python3
"""Fixed-angle V placement report and dimensioned placement diagram."""
import collections,csv,json,math,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from dock_detector.simulation import panel_polygons
ROOT=Path(sys.argv[1]);meta=json.loads((ROOT/'metadata.json').read_text());geometry=json.loads((ROOT/'placement_geometry.json').read_text());rows=[json.loads(l) for l in (ROOT/'samples.jsonl').read_text().splitlines()]
def longest(flags):
    best=run=0
    for f in flags:run=0 if f else run+1;best=max(best,run)
    return best
stats=[]
for g in geometry:
    d=g['standoff_m'];grid=[r for r in rows if r['standoff_m']==d and r['kind']=='grid'];valid=[r for r in grid if r['clear']];common=[r for r in grid if r['common_clear']];accepted=[r for r in valid if r['matched']]
    s={**g,'samples':len(grid),'clear_fraction':float(np.mean([r['clear'] for r in grid])),
       'useful_fraction_all':float(np.mean([r['good'] for r in grid])),
       'useful_fraction_on_clear':float(np.mean([r['good'] for r in valid])),
       'useful_fraction_common':float(np.mean([r['good'] for r in common])),
       'common_samples':len(common),'position_p95_cm':float(np.percentile([r['position_error_m']*100 for r in accepted],95)),
       'yaw_p95_deg':float(np.percentile([r['yaw_error_deg'] for r in accepted],95))}
    s['paths']=[]
    for p in range(3):
        path=sorted([r for r in rows if r['kind']=='path' and r['standoff_m']==d and r['repeat']==p],key=lambda r:r['index'])
        s['paths'].append({'y_offset_m':[-.1,0,.1][p],'collision_samples':sum(not r['clear'] for r in path),'missed_samples_on_clear':sum(r['clear'] and not r['matched'] for r in path),'longest_missed_run_on_clear_s':longest([r['matched'] if r['clear'] else True for r in path])/6})
    stats.append(s)
# Explicit design objective: compact standoff satisfying sampled clearance and
# strong common-population perception, with complete near-final reverse paths.
candidates=[s for s in stats if s['worst_tolerance_clearance_m']>=.05 and s['useful_fraction_common']>=.90 and all(p['collision_samples']==0 and p['longest_missed_run_on_clear_s']<=.5 for p in s['paths'])]
chosen=min(candidates,key=lambda s:s['standoff_m']) if candidates else max(stats,key=lambda s:s['worst_tolerance_clearance_m'])
d=chosen['standoff_m'];flare=math.radians(55);length=.7
apex=np.array([-d,0.]);tips=[apex+[length*math.cos(flare),sign*length*math.sin(flare)] for sign in (-1,1)]
placement={'apex_relative_final_m':apex.tolist(),'inner_front_tips_relative_final_m':[p.tolist() for p in tips],
 'wall_centres_relative_final_m':[], 'wall_yaws_rad':[-flare,flare], 'panel_length_m':length,
 'entrance_width_m':2*length*math.sin(flare),'forward_depth_m':length*math.cos(flare),
 'apex_world':[-1-d,3.], 'v_offset':d}
for poly in panel_polygons(flare,shape='v',length=length):placement['wall_centres_relative_final_m'].append((poly.mean(axis=0)+apex).tolist())
placement['wall_centres_world']=[[x-1,y+3] for x,y in placement['wall_centres_relative_final_m']]
summary={'selected':chosen,'placements':stats,'recommended_geometry':placement,
 'selection_rule':'Smallest standoff with >=5 cm clearance over the sampled final tolerance window, >=90% useful detection on common-clear corridor placements, and fully clear tested paths with <=0.5 s missed runs.'}
(ROOT/'summary.json').write_text(json.dumps(summary,indent=2))
with (ROOT/'placement_ranking.csv').open('w') as f:
    fields=[k for k in stats[0] if k not in ('paths','apex_world')];w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(stats)

def hull(points):
    p=sorted(set(map(tuple,points)))
    def cross(o,a,b):return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    low=[]
    for point in p:
        while len(low)>=2 and cross(low[-2],low[-1],point)<=0:low.pop()
        low.append(point)
    high=[]
    for point in reversed(p):
        while len(high)>=2 and cross(high[-2],high[-1],point)<=0:high.pop()
        high.append(point)
    return np.array(low[:-1]+high[:-1])
body=np.array(meta['footprint']);envelope=[]
for x in (-.05,0,.05):
 for y in (-.1,-.05,0,.05,.1):
  for a in map(math.radians,(-10,0,10)):
   c,s=math.cos(a),math.sin(a);envelope.extend(body@np.array([[c,s],[-s,c]])+[x,y])
envelope=hull(envelope)
fig,axes=plt.subplots(1,2,figsize=(14,6.5),constrained_layout=True);ax=axes[0]
for poly in panel_polygons(flare,shape='v',length=length):
    old=poly+[-.425,0];ax.fill(old[:,0],old[:,1],color='.75',alpha=.6)
    new=poly+apex;ax.fill(new[:,0],new[:,1],color='black')
ax.fill(envelope[:,0],envelope[:,1],color='royalblue',alpha=.12,label='Sampled final-pose tolerance envelope')
ax.fill(body[:,0],body[:,1],color='royalblue',alpha=.45,label='Nominal configured Nav2 footprint')
ax.scatter([0],[0],marker='+',s=150,linewidths=2,c='blue',label='Final base_link point')
ax.scatter([.8],[0],marker='+',s=150,linewidths=2,c='cyan',label='Staging reference')
ax.scatter([-d],[0],marker='x',s=50,c='black');ax.annotate(f'Proposed apex: (−{d:.3f}, 0) m',(-d,0),xytext=(-.95,-.65),arrowprops={'arrowstyle':'->'},fontsize=9)
ax.annotate('',xy=(0,-.73),xytext=(-d,-.73),arrowprops={'arrowstyle':'<->'});ax.text(-d/2,-.71,f'{d*100:g} cm',ha='center',va='bottom')
ax.annotate('Reverse approach',xy=(.32,0),xytext=(.7,.15),arrowprops={'arrowstyle':'->'},ha='center',fontsize=9)
ax.set_title('Fixed 70 cm panels, 55° per arm\nBlack: proposed placement; gray: previous 42.5 cm standoff')
ax.set_aspect('equal');ax.set_xlabel('X relative to final point (m)');ax.set_ylabel('Y relative to final point (m)');ax.set_xlim(-1.,1.);ax.set_ylim(-.8,.8);ax.grid(alpha=.2);ax.legend(loc='upper right',fontsize=8)
ax=axes[1];dist=np.array([s['standoff_m'] for s in stats]);ax.plot(dist,[s['nominal_clearance_m']*100 for s in stats],'o-',label='Nominal footprint clearance')
ax.plot(dist,[s['worst_tolerance_clearance_m']*100 for s in stats],'o-',label='Worst sampled tolerance clearance');ax.axhline(5,c='gray',ls=':',label='Chosen 5 cm margin');ax.axvline(d,c='black',ls='--',label='Proposed standoff')
ax2=ax.twinx();ax2.plot(dist,[s['useful_fraction_common']*100 for s in stats],'s-',c='green',label='Useful detection, common-clear poses');ax2.set_ylabel('Useful detection (%)',color='green');ax2.set_ylim(0,101)
ax.set_xlabel('Final-point distance ahead of V apex (m)');ax.set_ylabel('Minimum footprint-to-panel clearance (cm)');ax.grid(alpha=.2);ax.set_title('Move walls back enough for tolerance,\nthen retain the smallest suitable standoff');ax.legend(loc='upper left',fontsize=8);ax2.legend(loc='lower right',fontsize=8)
fig.savefig(ROOT/'placement_recommendation.png',dpi=170);plt.close(fig)
text=f'''# Placement recommendation: fixed 55° / 70 cm V

## Proposed placement

Keep the V axis aligned with the parked robot's +X heading and centered laterally on the final point. Put its inner apex **{d:.3f} m behind the final base_link point**. This moves the apex {(d-.425)*100:.1f} cm farther back than the previous 0.425 m offset, without changing panel length or angle. The proposed detector parking offset is v_offset={d:.3f}. The world/parking offset has NOT been applied: this task evaluates and suggests placement only.

For final world (-1,3,yaw=0), proposed apex is ({-1-d:.3f},3). World panel centres are {placement['wall_centres_world']} with yaws [-55°, +55°], 0.70 m length, 0.05 m thickness. The front inner tips are {placement['inner_front_tips_relative_final_m']} relative to the final point. Entrance width is {placement['entrance_width_m']:.3f} m and forward panel projection is {placement['forward_depth_m']:.3f} m.

## Clearance and perception

Nominal Nav2-footprint clearance is {chosen['nominal_clearance_m']*100:.1f} cm. Minimum clearance across the **sampled** final-pose window is {chosen['worst_tolerance_clearance_m']*100:.1f} cm: fore/aft errors [-5,0,+5] cm, lateral errors [-10,-5,0,+5,+10] cm, yaw errors [-10,0,+10] degrees. This is a discrete sampled envelope, not a continuous clearance proof or a replacement for collision checking.

The design rule chooses the smallest tested standoff with at least 5 cm sampled clearance, at least 90% useful detections on the common-clear corridor population, and no collision on the three seeded reverse paths with missed runs <=0.5 seconds. The 5 cm margin is an explicit design choice, not a hardware specification supplied by the user. All candidates keep the same 55 degree flare, 70 cm panels and 5 cm thickness.

| Apex standoff | Nominal clearance | Worst sampled clearance | Useful coverage, common-clear | Useful coverage, all corridor cases | Position p95 |
|---|---|---|---|---|---|
'''
for s in stats:text+=f"| {s['standoff_m']:.3f} m | {s['nominal_clearance_m']*100:.1f} cm | {s['worst_tolerance_clearance_m']*100:.1f} cm | {s['useful_fraction_common']*100:.1f}% | {s['useful_fraction_all']*100:.1f}% | {s['position_p95_cm']:.2f} cm |\n"
text+='''
## Method and limitations

Only apex-to-final-point distance was varied, from 0.425 to 0.825 m in 2.5 cm steps. Lateral centering follows the symmetric robot, panels and test corridor; arbitrary lateral shifts were not optimized. The 10 cm corridor grid covers 0…0.8 m ahead of the final point and lateral offsets -0.2…+0.2 m, with headings -20,-10,0,+10,+20 degrees and three paired noise repetitions. It is narrower than the earlier full practice-area sweep. Shared-population perception uses exactly placements collision-free at every standoff; the all-case score also counts footprint collisions as unusable placements. Accepted-pose error includes the actual standoff's heading lever arm.

LiDAR uses 360 rays over 355 degrees, 1 cm Gaussian range noise, 0.3 m minimum range and 5 cm forward sensor offset. The actual V matcher and configured footprint are used. Three seeded reverse paths cover the final 0.8 m, at Y offsets -0.1,0,+0.1 m, 6 Hz and 0.035 m/s. Source hashes, all raw cases and geometry are archived. The model omits clutter, reflectance, motion distortion, extra self-occlusion and real-time TF/control latency; it uses an exact prior.

Gray/blocked regions immediately adjacent to walls cannot be eliminated as usable robot placements; the goal here is to move them out of the desired parking/approach corridor. This is an offline recommendation, not a physical safety guarantee. The V default opening is saved as 110 degrees (55 per arm); no live world, detector pose offset or robot motion was changed.
'''
(ROOT/'README.md').write_text(text);(ROOT/'report.py').write_text(Path(__file__).read_text());print(json.dumps({'selected':chosen,'geometry':placement},indent=2))
