#!/usr/bin/env python3
"""Report an offline V sweep with its fixed trapezoid baseline; no ROS access."""
import collections,csv,json,math,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(sys.argv[1]);meta=json.loads((ROOT/'metadata.json').read_text())
rows=[json.loads(line) for line in (ROOT/'samples.jsonl').read_text().splitlines()]
shape=meta.get('shape','trapezoid');angles=meta['angles_deg']
v_length=meta['panel_lengths_m'][0]
def panel_length(sh):return v_length if sh=='v' else .5
if shape != 'v':
    raise SystemExit('Use the matching archived trapezoid report for trapezoid-only sweeps.')
scenarios=[(shape,a) for a in angles]
if meta.get('comparison_baseline'):scenarios.append(('trapezoid',32.5))

def selected(kind,sh,a):return [r for r in rows if r['kind']==kind and r.get('shape','trapezoid')==sh and r['angle_deg']==a]
def visible(r):return r.get('visible_faces',r.get('visible_three_faces',False))
def longest(flags):
    best=run=0
    for flag in flags:
        run=0 if flag else run+1;best=max(best,run)
    return best

stats=[]
for sh,angle in scenarios:
    grid=selected('grid',sh,angle);common=[r for r in grid if r['common_clear']];clear=[r for r in grid if r['clear']]
    accepted=[r for r in common if r['matched']];conf=[r['confidence'] for r in accepted if r.get('confidence') is not None]
    stat={'shape':sh,'panel_length_m':panel_length(sh),'angle_deg':angle,'opening_deg':angle*2 if sh=='v' else None,
          'entrance_width_m':(.5 if sh=='trapezoid' else 0.)+2*panel_length(sh)*math.sin(math.radians(angle)),
          'depth_m':panel_length(sh)*math.cos(math.radians(angle)),
          'common_clear_placements':len({r['index'] for r in common}),'common_clear_samples':len(common),
          'match_fraction':float(np.mean([r['matched'] for r in common])),
          'good_fraction':float(np.mean([r['good'] for r in common])),
          'visibility_fraction':float(np.mean([visible(r) for r in common])),
          'angle_clear_placements':len({r['index'] for r in clear}),
          'angle_clear_good_fraction':float(np.mean([r['good'] for r in clear])),
          'incorrect_accepted':sum(r['matched'] and not r['good'] for r in common),
          'reasons':dict(collections.Counter(r['reason'] for r in common)),
          'confidence_median':float(np.median(conf)) if conf else None}
    if accepted:
        stat['position_p95_cm']=float(np.percentile([r['position_error_m']*100 for r in accepted],95))
        stat['yaw_p95_deg']=float(np.percentile([r['yaw_error_deg'] for r in accepted],95))
    stat['paths']=[]
    for path in range(3):
        p=sorted([r for r in selected('path',sh,angle) if r['repeat']==path],key=lambda r:r['index'])
        free=[r for r in p if r['clear']]
        # Infeasible positions break a run and are never called sensor misses.
        match_flags=[r['matched'] if r['clear'] else True for r in p]
        useful_flags=[r['good'] if r['clear'] else True for r in p]
        stat['paths'].append({'y_offset_m':[-.1,0,.1][path],'samples':len(p),
            'collision_samples':len(p)-len(free),'feasible_to_final':len(p)==len(free),
            'match_fraction_on_clear':float(np.mean([r['matched'] for r in free])) if free else None,
            'good_fraction_on_clear':float(np.mean([r['good'] for r in free])) if free else None,
            'longest_missed_run_on_clear_s':longest(match_flags)/6,
            'longest_unusable_run_on_clear_s':longest(useful_flags)/6,
            'last_clear_x_rear_m':min((r['x_rear'] for r in free),default=None)})
    stats.append(stat)
ranked=sorted([s for s in stats if s['shape']==shape],key=lambda s:s['good_fraction'],reverse=True)
baseline=next((s for s in stats if s['shape']!=shape),None)
summary={'shape':shape,'ranked':ranked,'baseline':baseline,'grid_xy_positions':247,
         'grid_step_m':.1,'cases':len(rows),'useful_threshold':{'position_cm':3,'yaw_deg':3},
         'population':'Common collision-free placements across all tested V angles and 32.5-degree trapezoid' if baseline else 'Common collision-free placements across all tested angles'}
(ROOT/'summary.json').write_text(json.dumps(summary,indent=2))
fields=['shape','panel_length_m','angle_deg','opening_deg','entrance_width_m','depth_m','common_clear_placements','match_fraction','good_fraction','visibility_fraction','position_p95_cm','yaw_p95_deg','incorrect_accepted','angle_clear_placements','angle_clear_good_fraction']
with (ROOT/'ranking.csv').open('w') as f:
    w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(stats)

# Heat maps pool only common-clear headings, hence the same cells/denominators.
fig,axes=plt.subplots(2,3,figsize=(15,9),constrained_layout=True,sharex=True,sharey=True)
for ax,(map_shape,a) in zip(axes.ravel(),scenarios):
    length=panel_length(map_shape)
    grid=selected('grid',map_shape,a);cells=collections.defaultdict(list)
    for r in grid:
        if r['common_clear']:cells[(r['x_world'],r['y_world'])].append(r['good'])
    invalid={(r['x_world'],r['y_world']) for r in grid}-set(cells)
    ax.scatter([c[0] for c in invalid],[c[1] for c in invalid],color='.85',marker='s',s=28)
    scatter=ax.scatter([c[0] for c in cells],[c[1] for c in cells],c=[np.mean(v) for v in cells.values()],cmap='RdYlGn',vmin=0,vmax=1,s=28,marker='s')
    flare=math.radians(a);rear_half=.25 if map_shape=='trapezoid' else 0.
    for sign in (-1,1):ax.plot([-1.425,-1.425+length*math.cos(flare)],[3+sign*rear_half,3+sign*(rear_half+length*math.sin(flare))],'k-',lw=2)
    if map_shape=='trapezoid':ax.plot([-1.425,-1.425],[2.75,3.25],'k-',lw=2)
    ax.scatter([-1.,-.2],[3.,3.],marker='+',c=['blue','cyan'],s=50)
    score=next(s for s in stats if s['shape']==map_shape and s['angle_deg']==a)
    ax.set_title((f'V {a:g}° / {a*2:g}° opening' if map_shape=='v' else f'Trapezoid {a:g}° baseline')+f"\n{length*100:g} cm panels — useful coverage {score['good_fraction']*100:.1f}%")
    ax.set_xlabel('World X (m)');ax.set_ylabel('World Y (m)');ax.set_aspect('equal');ax.grid(alpha=.15)
fig.colorbar(scatter,ax=axes.ravel().tolist(),label='Accepted within 3 cm / 3°, pooled common-clear headings and noise repeats')
fig.suptitle(f'{shape.upper()} — {v_length*100:g} cm arms; baseline: 50 cm trapezoid panels; 10 cm grid; 360 rays over 355°, 1 cm range noise\nGray: no common-clear heading. + marks the nominal parking and staging references, not guaranteed clearance.')
fig.savefig(ROOT/'coverage_maps.png',dpi=160);plt.close(fig)

fig,axes=plt.subplots(1,3,figsize=(15,4.8),constrained_layout=True)
ordered=[next(s for s in stats if s['shape']==shape and s['angle_deg']==a) for a in angles]
for key,label in [('visibility_fraction','Raw face support'),('match_fraction','Detector match'),('good_fraction','Within 3 cm / 3°')]:
    axes[0].plot(angles,[s[key]*100 for s in ordered],'o-',label=label)
if baseline:axes[0].axhline(baseline['good_fraction']*100,c='black',ls='--',label='32.5° trapezoid, same placements')
axes[0].set_ylabel('Common-clear coverage (%)');axes[0].legend(fontsize=8)
axes[1].plot(angles,[s['position_p95_cm'] for s in ordered],'o-',label='Position p95 (cm)')
axes[1].plot(angles,[s['yaw_p95_deg'] for s in ordered],'o-',label='Yaw p95 (degrees)');axes[1].set_ylabel('Accepted-pose error');axes[1].legend(fontsize=8)
for path in range(3):
    axes[2].plot(angles,[s['paths'][path]['longest_missed_run_on_clear_s'] for s in ordered],'o-',label=f'y offset {[-.1,0,.1][path]:+.1f} m')
    for s in ordered:
        if not s['paths'][path]['feasible_to_final']:axes[2].scatter(s['angle_deg'],s['paths'][path]['longest_missed_run_on_clear_s'],marker='x',c='red',s=70,zorder=5)
axes[2].set_ylabel('Longest missed run on clear path (s)');axes[2].legend(fontsize=8)
for ax in axes:ax.set_xlabel('Flare per arm/side (degrees)');ax.set_xticks(angles);ax.grid(alpha=.2)
fig.suptitle(f'{shape.upper()} sweep; red × means the path intersects a wall before the nominal final pose')
fig.savefig(ROOT/'comparison.png',dpi=160);plt.close(fig)

text=f"""# V flare sweep: {v_length*100:g} cm panels

No ROS commands were issued. The trapezoid world and runtime configuration remain at the previously selected 32.5 degrees with three 50 cm panels.

## Geometry and method

The V uses two {v_length:.2f} m panels, each 0.05 m thick, meeting at an inner apex. Tested per-arm flares are {', '.join(f'{a:g}' for a in angles)} degrees (included opening is twice flare). The fixed baseline uses three 0.50 m panels with 32.5 degree side flare. Rear centre/apex stays world (-1.425,3); the scored parking reference remains 0.425 m along +X. Pose-estimation scores do not imply that this reference is physically reachable.

The same 10 cm grid spans world X=-1.4…0.4, Y=2.4…3.6: 247 XY positions, five headings (-20,-10,0,10,20 degrees), three paired noise repetitions. The sensor has 360 rays over 355 degrees, 0.3–24 m range, 1 cm Gaussian range noise, and a 5 cm forward offset. Nearest-hit intersections use all four edges of thick panel boxes, including occlusion by too-close surfaces.

Collision/touching placements use the actual Nav2 footprint. The primary population is exactly the placements collision-free for all tested V angles AND the baseline. Its denominator differs from earlier runs; compare shapes only within this run. Per-angle feasible-population metrics are saved separately.

The actual V matcher uses expected_angle=2*flare, with all other detector thresholds unchanged. The baseline uses joint trapezoid refinement. Useful means reference-point error <=3 cm and yaw error <=3 degrees. V confidence is unavailable, so measured error and coverage are reported. The unchanged 0.95 m ROI contains the tested panels.

## Results

| Shape | Panel length | Flare | Included V opening | Useful coverage | Position p95 | Heading p95 |
|---|---|---|---|---|---|---|
"""
for stat in stats:
    opening=f"{stat['opening_deg']:g}°" if stat['opening_deg'] is not None else '—'
    text+=f"| {stat['shape']} | {stat['panel_length_m']*100:g} cm | {stat['angle_deg']:g}° | {opening} | {100*stat['good_fraction']:.1f}% | {stat['position_p95_cm']:.2f} cm | {stat['yaw_p95_deg']:.2f}° |\n"
text+=f"\nBest tested useful coverage: **{ranked[0]['angle_deg']:g}° per arm / {ranked[0]['opening_deg']:g}° included opening**. Common population: {ranked[0]['common_clear_placements']} placements ×3 repeats per scenario.\n"
text+='\n## Parking clearance and synthetic gaps\n\n'
for stat in stats:
    feasible=[p['y_offset_m'] for p in stat['paths'] if p['feasible_to_final']]
    text+=f"- {stat['shape']} {stat['angle_deg']:g}°: paths reaching the nominal final reference without footprint collision: Y offsets {feasible}. Collision sample counts across (-0.1,0,+0.1) m paths: {[p['collision_samples'] for p in stat['paths']]}.\n"
text+="""
Paths use rear-relative X=1.225→0.425, Y offsets -0.1/0/+0.1 m, 3.5 cm/s and ideal 6 Hz scans. Collision samples are excluded from detector misses and break runs; each path reports its last clear position. Gaps are consecutive missed scans /6, including boundary runs on the valid path. A short gap does not prove a path is traversable. These are single seeded paths, not temporal reliability guarantees.

## Artifacts and limitations

coverage_maps.png includes each V angle and the fixed baseline. comparison.png, ranking.csv, summary.json, samples.csv and samples.jsonl preserve all metrics. metadata.json contains exact settings and source hashes. simulation.py, geometry.py, refinement.py, sweep_flare.py and report.py preserve the implementation used.

This is an idealized offline test, with an exact prior and no unrelated clutter, reflectance effects, beam divergence, motion distortion, extra self-occlusion, TF or processing latency. Three noise realizations are used per placement. This estimates performance for the current matcher/model, not physical reliability or a validated optimal angle. No runtime V dimensions or default angle were changed.
"""
(ROOT/'README.md').write_text(text);(ROOT/'report.py').write_text(Path(__file__).read_text())
print(json.dumps(summary,indent=2))
