#!/usr/bin/env python3
"""Independent bag audit for the declared three-AAV handover qualification.

Uses the existing alignment grid/gap policy. Scene is obstacle-free; this is
sampled inter-platform/domain evidence, never unknown-environment safety proof.
"""
import argparse
import json
import math
import hashlib
import struct
from pathlib import Path

import rosbag
from qn_aav_simulator.time_alignment import (
    DEFAULT_ALIGNMENT_WINDOW_S, ModelTimeSample, TimeAlignmentMonitor, _interpolate)


def audit(path,include_marine=False,scene_file=None,scene_bag=None):
    members=list(range(3))+(['usv','uuv'] if include_marine else [])
    scene=None
    expected_cloud_hash=None
    observed_cloud_hashes=set()
    map_publishers=set()
    radii={i:.25 for i in range(3)}
    if scene_file is not None:
        import yaml
        from qn_aav_simulator.experiment_verdict import StaticSceneGeometry
        config=yaml.safe_load(scene_file.read_text())
        config=config.get('scene',config)
        scene=StaticSceneGeometry.from_mapping(config)
        if scene is None:raise ValueError('requested scene audit requires declared enabled geometry')
        from qn_aav_simulator.experiment_verdict import box_sample_points
        expected=b''.join(struct.pack('<fff',*point) for _,kind,c,s in scene.objects if kind=='SOLID'
                          for point in box_sample_points(c,s,float(config.get('resolution',.2))))
        expected_cloud_hash=hashlib.sha256(expected).hexdigest()
        if include_marine:
            from qn_aav_simulator.pvs_backend import PvsBackend
            radii['usv']=PvsBackend('otter',(0.,0.,0.)).collision_radius_m
            radii['uuv']=PvsBackend('remus100',(0.,0.,-2.)).collision_radius_m
    odom={i:[] for i in members}
    diagnostics={i:[] for i in members}
    timeline=[]
    action_starts=[]
    result_goal_starts=[]
    action_finishes=[]
    invalid_native_trajectories=[]
    with rosbag.Bag(str(path)) as bag:
        for topic,msg,received,connection in bag.read_messages(return_connection_header=True):
            if topic.endswith('_qn/odometry'):
                i=int(topic.split('_')[1])
                p=msg.pose.pose.position
                odom[i].append((msg.header.stamp.to_sec(),(p.x,p.y,p.z),msg.header.frame_id))
            elif topic.endswith('_qn/diagnostics'):
                i=int(topic.split('_')[1])
                values={v.key:v.value for s in msg.status for v in s.values}
                if 'model_time_s' in values:
                    diagnostics[i].append((msg.header.stamp.to_sec(),values))
            elif include_marine and topic in ('/usv/odometry','/uuv/odometry'):
                i=topic.split('/')[1];p=msg.pose.pose.position
                odom[i].append((msg.header.stamp.to_sec(),(p.x,p.y,p.z),msg.header.frame_id))
            elif include_marine and topic in ('/usv/diagnostics','/uuv/diagnostics'):
                i=topic.split('/')[1]
                values={v.key:v.value for s in msg.status for v in s.values}
                if 'model_time_s' in values:diagnostics[i].append((msg.header.stamp.to_sec(),values))
            elif topic=='/drone_0_planning/safety_status':
                values={v.key:v.value for s in msg.status for v in s.values}
                timeline.append((received.to_sec(),'status',values))
            elif topic=='/drone_0_planning/trajectory':
                timeline.append((received.to_sec(),'trajectory',int(msg.traj_id)))
                caller=connection.get('callerid','')
                if isinstance(caller,bytes):caller=caller.decode('utf-8',errors='replace')
                valid=(msg.order==5 and bool(msg.duration) and
                       all(math.isfinite(v) and v>0 for v in msg.duration) and
                       all(len(c)==6*len(msg.duration) and all(math.isfinite(v) for v in c)
                           for c in (msg.coef_x,msg.coef_y,msg.coef_z)))
                if not valid and caller.endswith('_ego_planner_node'):
                    invalid_native_trajectories.append(dict(id=int(msg.traj_id),received=received.to_sec(),caller=caller))
            elif topic.endswith('/goal') and hasattr(msg,'goal_id'):
                action_starts.append(received.to_sec())
            elif topic.endswith('/result') and hasattr(msg,'status'):
                action_finishes.append(received.to_sec())
                # A Result confirms the GoalID of the accepted native Action.
                # Its original stamp remains available even if a bag started
                # before the Action publisher connected but missed /goal.
                goal_stamp=msg.status.goal_id.stamp.to_sec()
                if math.isfinite(goal_stamp) and 0<goal_stamp<=received.to_sec():
                    result_goal_starts.append(goal_stamp)
            elif scene is not None and topic=='/scene/global_cloud':
                caller=connection.get('callerid','')
                if isinstance(caller,bytes):caller=caller.decode('utf-8',errors='replace')
                map_publishers.add(caller)
                observed_cloud_hashes.add((msg.header.frame_id,hashlib.sha256(bytes(msg.data)).hexdigest()))
    scene_once_count=0
    if scene_bag is not None:
        with rosbag.Bag(str(scene_bag)) as bag:
            for topic,msg,_,connection in bag.read_messages(return_connection_header=True):
                if topic!='/scene/global_cloud':continue
                scene_once_count+=1
                caller=connection.get('callerid','')
                if isinstance(caller,bytes):caller=caller.decode('utf-8',errors='replace')
                map_publishers.add(caller)
                observed_cloud_hashes.add((msg.header.frame_id,hashlib.sha256(bytes(msg.data)).hexdigest()))
    failures=[]
    if scene is not None and (observed_cloud_hashes!={(scene.frame,expected_cloud_hash)} or
                              map_publishers!={'/scene_publisher'} or
                              (scene_bag is not None and scene_once_count!=1)):
        failures.append('actual scene cloud differs from declared SOLID geometry or publisher')
    if invalid_native_trajectories:failures.append('native planner published invalid polynomial')
    interval_source='goal topic' if action_starts else 'result GoalID stamp' if result_goal_starts else None
    interval_start=min(action_starts) if action_starts else min(result_goal_starts) if result_goal_starts else None
    interval_end=max(action_finishes) if action_finishes else None
    if interval_start is None or interval_end is None:failures.append('missing Action interval')
    monitor=TimeAlignmentMonitor(tuple(str(i) for i in members))
    for i,series in diagnostics.items():
        series.sort(key=lambda x:x[0])
        for stamp,values in series:
            monitor.note_sample(ModelTimeSample(str(i),stamp,float(values['model_time_s'])))
        steps=[int(v['used_outer_step'] if i in range(3) else v['steps']) for _,v in series]
        if not steps or any(b<=a for a,b in zip(steps,steps[1:])):
            failures.append('missing/non-increasing actual model steps: '+str(i))
    alignment=monitor.report().as_dict()
    if not alignment['within_thresholds']:failures.append('model time alignment failed')
    if any(not s for s in odom.values()):failures.append('missing member odometry')
    minimum=None
    minimum_center=None
    fleet_clearance=None
    scene_minima={}
    missing=0
    missing_inside=[]
    missing_outside=[]
    measured=0
    prepared={}
    for i,series in odom.items():
        series.sort(key=lambda x:x[0])
        prepared[i]=([v[0] for v in series],[[v[1][k] for v in series] for k in range(3)])
        if not series or {v[2] for v in series}!={'world'}:failures.append('world frame mismatch: '+str(i))
    for point in monitor.aligned_grid():
        positions=[]
        for i in members:
            times,axes=prepared[i]
            positions.append(tuple(_interpolate(times,axis,point.ros_time_s,
                2*DEFAULT_ALIGNMENT_WINDOW_S) for axis in axes))
        inside=interval_start is not None and interval_end is not None and interval_start<=point.ros_time_s<=interval_end
        if any(v is None for p in positions for v in p):
            if inside:
                missing+=1
                missing_inside.append(dict(ros_time_s=point.ros_time_s,
                                           members=[str(member) for member,position in zip(members,positions)
                                                    if any(value is None for value in position)]))
            else:missing_outside.append(point.ros_time_s)
            continue
        if not inside:continue
        clearance=min(math.dist(positions[i],positions[j])-.5
                      for i in range(3) for j in range(i+1,3))
        minimum=clearance if minimum is None else min(minimum,clearance)
        centre=min(math.dist(positions[i],positions[j]) for i in range(len(members)) for j in range(i+1,len(members)))
        minimum_center=centre if minimum_center is None else min(minimum_center,centre)
        if scene is not None:
            pair=min(math.dist(positions[i],positions[j])-radii[members[i]]-radii[members[j]]
                     for i in range(len(members)) for j in range(i+1,len(members)))
            fleet_clearance=pair if fleet_clearance is None else min(fleet_clearance,pair)
            for member,position in zip(members,positions):
                for name,value in scene.clearances(position,radii[member]):
                    key=str(member)+'/'+name
                    scene_minima[key]=min(scene_minima.get(key,value),value)
        measured+=1
    if missing or not measured:failures.append('missing aligned position samples')
    if minimum is None or minimum<.5:failures.append('sampled inter-platform clearance failed')
    if scene is not None:
        if not scene_minima or any(v<scene.clearance for v in scene_minima.values()):
            failures.append('declared scene clearance failed')
        if fleet_clearance is None or fleet_clearance<.5:
            failures.append('declared fleet-proxy pair clearance failed')
    paused=False
    latched=False
    paused_publications=[]
    epochs=[]
    for stamp,kind,value in sorted(timeline,key=lambda x:x[0]):
        if kind=='status':
            paused=value.get('ordinary_reference_paused')=='true'
            latched=value.get('latched')=='true'
            key=(value.get('reference_generation'),paused,latched)
            if not epochs or tuple(epochs[-1]['context'])!=key:
                epochs.append(dict(received=stamp,context=key,floor=value.get('reference_floor_id')))
        elif paused and not latched:
            paused_publications.append(dict(received=stamp,trajectory_id=value))
    if paused_publications:failures.append('ordinary trajectory published after pause ACK')
    if not epochs or not any(kind=='trajectory' for _,kind,_ in timeline):
        failures.append('missing planner status or trajectory evidence')
    own=diagnostics[0]
    if not own or any(v.get('domain_violation')!='false' or v.get('air_domain_violation')!='false' for _,v in own):
        failures.append('missing domain evidence or recorded violation')
    for i in (1,2):
        if not diagnostics[i] or any(v.get('air_domain_violation')!='false' for _,v in diagnostics[i]):
            failures.append('missing AIR domain evidence or recorded violation: '+str(i))
    air_min_envelope={}
    for i in range(3):
        heights=[float(v['position_z'])-.25 for _,v in diagnostics[i]
                 if i!=0 or v.get('reference_source')=='AIR_SWARM']
        air_min_envelope[str(i)]=min(heights) if heights else None
        if not heights:failures.append('missing AIR-scope reference/state interval: '+str(i))
        elif min(heights)<0.:failures.append('AIR body envelope below declared surface: '+str(i))
    if include_marine:
        for i,mode in [('usv','SURFACE'),('uuv','WATER')]:
            if not diagnostics[i] or any(v.get('actual_mode')!=mode for _,v in diagnostics[i]):
                failures.append('native domain evidence failed: '+i)
            if scene is not None and any(float(v.get('collision_radius_m','nan'))!=radii[i] for _,v in diagnostics[i]):
                failures.append('runtime collision proxy differs from native-model geometry: '+i)
    returned=[]
    for stamp,v in own:
        if (v.get('reference_source')=='AIR_SWARM' and
                int(v.get('adopted_reference_generation','-1'))>=3):
            retired=int(v.get('reference_air_floor_id','-1'))
            used=int(v.get('source_trajectory_id','-1'))
            returned.append((stamp,used,retired))
            if retired<0 or used<=retired:failures.append('retired AIR reference adopted after return')
    return dict(scope=('sampled declared scene and collision proxies; NOT equipment-envelope certification' if scene else
                      'five-instance coordinates/time/native domains; marine hull clearance NOT VERIFIED'
                      if include_marine else 'sampled three-AAV handover; no obstacle/unknown-environment guarantee'),
        passed=not failures,failures=failures,alignment=alignment,
        aligned_position_samples=measured,missing_position_samples=missing,
        missing_position_details=missing_inside,
        execution_interval_ros_s=[interval_start,interval_end],
        execution_start_source=interval_source,
        missing_position_samples_outside_execution=missing_outside,
        minimum_surface_clearance_m=minimum,platform_radius_m=.25,required_clearance_m=.5,
        clearance_scope='three AAVs only',all_members_minimum_center_distance_m=minimum_center,
        declared_proxy_radii_m={str(k):v for k,v in radii.items()},
        fleet_proxy_minimum_clearance_m=fleet_clearance,scene_minimum_clearances_m=scene_minima,
        declared_solid_cloud_sha256=expected_cloud_hash,observed_scene_clouds=sorted(observed_cloud_hashes),
        scene_once_messages=scene_once_count if scene_bag is not None else None,
        reference_epochs=epochs,ordinary_publications_while_paused=paused_publications,
        invalid_native_trajectories=invalid_native_trajectories,air_minimum_body_envelope_m=air_min_envelope,
        returned_air_adoption_samples=len(returned),
        actual_modes=sorted({v.get('actual_mode','') for _,v in own}),
        source_generations=sorted({(v.get('reference_source'),v.get('adopted_reference_generation')) for _,v in own}))


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('bag',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--include-marine',action='store_true')
    parser.add_argument('--scene',type=Path)
    parser.add_argument('--scene-bag',type=Path)
    args=parser.parse_args()
    result=audit(args.bag,args.include_marine,args.scene,args.scene_bag)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['passed'] else 1)
