"""Generate a reviewable Nav2 params file from one named detector catalogue."""
import argparse
import json
from copy import deepcopy
import math
from pathlib import Path
import yaml


def merge_nav2(base, catalogue, settings=None):
    result=deepcopy(base)
    c=catalogue['dock_detector']['ros__parameters']
    ids=c['dock_ids']
    if not ids or len(ids)!=len(set(ids)):
        raise ValueError('dock_ids must be nonempty and unique')
    server=result.setdefault('docking_server',{}).setdefault('ros__parameters',{})
    server.pop('dock_backwards',None)
    server.update(enable_stamped_cmd_vel=True, fixed_frame=c.get('fixed_frame','odom'),
                  dock_plugins=['lidar_dock'], docks=ids,
                  dock_approach_timeout=90., initial_perception_timeout=8.,
                  dock_prestaging_tolerance=.1)
    server['lidar_dock']=dict(plugin='dock_detector_nav2::LidarDock',
        dock_direction='backward', rotate_to_dock=False,
        staging_x_offset=-.8, staging_yaw_offset=math.pi, docking_threshold=.025,
        use_external_detection_pose=True, use_stall_detection=False,
        external_detection_timeout=.5, external_detection_translation_x=0.,
        external_detection_translation_y=0., external_detection_rotation_roll=0.,
        external_detection_rotation_pitch=0., external_detection_rotation_yaw=0.,
        filter_coef=.1, selection_service='dock_detector/select_dock',
        detection_topic='dock_detector/detection', selection_timeout=3.)
    controller=server.setdefault('controller',{})
    controller.update(v_linear_min=.01,v_linear_max=.035,v_angular_max=.08,
                      use_collision_detection=True)
    for dock_id in ids:
        d=c['docks'][dock_id]
        pose=[float(d[k]) for k in ('expected_dock_x','expected_dock_y','expected_dock_yaw_deg')]
        if not all(math.isfinite(v) for v in pose):raise ValueError('Dock prior must be finite')
        pose[2]=math.radians(pose[2])
        # Nav2 passes the optional database .id field to getRefinedPose,
        # not the lookup key. Set it explicitly to our catalogue key.
        server[dock_id]=dict(type='lidar_dock',id=dock_id,frame=c.get('expected_frame','map'),pose=pose)
    if settings:
        def overlay(target, source):
            for key, value in source.items():
                if isinstance(value, dict) and isinstance(target.get(key), dict):
                    overlay(target[key], value)
                else:
                    target[key] = deepcopy(value)
        overlay(server, settings['docking_server']['ros__parameters'])
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--base',type=Path,required=True,help='Existing Nav2 parameter YAML')
    p.add_argument('--catalogue',type=Path,required=True)
    p.add_argument('--settings',type=Path,help='Optional docking_server tuning YAML')
    p.add_argument('--output',type=Path,required=True,help='New output file (exclusive creation)')
    args=p.parse_args()
    result=merge_nav2(yaml.safe_load(args.base.read_text()),yaml.safe_load(args.catalogue.read_text()),
                      yaml.safe_load(args.settings.read_text()) if args.settings else None)
    with args.output.open('x') as f:yaml.safe_dump(json.loads(json.dumps(result)),f,sort_keys=False)
    print(f'Wrote {args.output}; review map alignment, footprint/costmap topics and velocity remapping before launching.')

if __name__=='__main__':main()
