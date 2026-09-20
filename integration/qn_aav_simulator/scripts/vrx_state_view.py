#!/usr/bin/env python3
"""Display existing qn/PVS state in VRX; this is NOT a dynamics backend.

Proxy models have no collisions, gravity, controllers, buoyancy or sensors.
Gazebo receives their pose; no Gazebo pose/clock enters the platform executors.
The declared ENU translation places the local scene near the official launch.
"""
import copy
import json
from pathlib import Path
import threading
import time

import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel,SetModelState,GetModelState


MEMBERS=('drone_0','drone_1','drone_2','usv','uuv')
OFFSET=(-500.,156.,0.)


def visual_model(member):
    if member.startswith('drone_'):
        geometry='<mesh><uri>file:///workspace/src/src/Utils/odom_visualization/meshes/hummingbird.mesh</uri><scale>2 2 2</scale></mesh>'
        pose='0 0 0 0 0 0'
    elif member=='usv':
        # WAM-V is only the visual proxy, not the Otter model/geometry source.
        geometry='<mesh><uri>model://WAM-V-Base/mesh/WAM-V-Base.dae</uri><scale>0.4 0.4 0.4</scale></mesh>'
        pose='0 0 0 0 0 0'
    else:
        geometry='<cylinder><radius>0.15</radius><length>1.6</length></cylinder>'
        pose='0 0 0 0 1.57079632679 0'
    return ('<sdf version="1.6"><model name="heformation_'+member+'"><static>true</static>'
            '<link name="visual_only"><gravity>false</gravity><visual name="body"><pose>'+pose+
            '</pose><geometry>'+geometry+'</geometry></visual></link></model></sdf>')


def main():
    rospy.init_node('vrx_state_view')
    if rospy.get_param('/use_sim_time',False):
        raise RuntimeError('external qn/PVS view requires unchanged ROS wall time')
    output=Path(rospy.get_param('~output_dir','/experiments/current'))
    latest={};lock=threading.Lock();counters={m:0 for m in MEMBERS};spawned=set();subscribers=[]
    def received(msg,member):
        if msg.header.frame_id!='world':return
        with lock:latest[member]=(copy.deepcopy(msg.pose.pose),time.monotonic(),msg.header.stamp.to_sec())
    for member in MEMBERS:
        prefix='/'+member+('_qn' if member.startswith('drone_') else '')
        subscribers.append(rospy.Subscriber(prefix+'/odometry',Odometry,lambda m,k=member:received(m,k),queue_size=1))
    for service in ['/gazebo/spawn_sdf_model','/gazebo/set_model_state','/gazebo/get_model_state']:
        rospy.wait_for_service(service,timeout=120)
    spawn=rospy.ServiceProxy('/gazebo/spawn_sdf_model',SpawnModel)
    update=rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)
    query=rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
    max_error=0.;last_write=0.;failures=[]
    while not rospy.is_shutdown():
        with lock:rows=dict(latest)
        for member,(pose,received_at,stamp) in rows.items():
            if time.monotonic()-received_at>1.:
                continue  # retain last view; do not invent stopped/completed state
            pose=copy.deepcopy(pose)
            pose.position.x+=OFFSET[0];pose.position.y+=OFFSET[1];pose.position.z+=OFFSET[2]
            name='heformation_'+member
            if member not in spawned:
                result=spawn(name,visual_model(member),'',pose,'world')
                if not result.success:
                    rospy.logerr_throttle(5.,'VRX display spawn: %s',result.status_message)
                    continue
                spawned.add(member)
            result=update(ModelState(model_name=name,pose=pose,reference_frame='world'))
            if not result.success:
                failures.append(result.status_message);continue
            counters[member]+=1
            if counters[member]%20==0:
                actual=query(name,'world')
                if actual.success:
                    error=sum((getattr(actual.pose.position,a)-getattr(pose.position,a))**2 for a in 'xyz')**.5
                    max_error=max(max_error,error)
                else:failures.append(actual.status_message)
        if time.monotonic()-last_write>2.:
            last_write=time.monotonic()
            report=dict(scope='one-way actual-state visualization only; VRX environment is NOT the planning/safety map',
                        translation_enu_m=OFFSET,updates=counters,max_position_roundtrip_error_m=max_error,
                        state_age_s={k:time.monotonic()-v[1] for k,v in rows.items()},
                        use_sim_time=rospy.get_param('/use_sim_time',False),failures=failures[-20:])
            output.mkdir(parents=True,exist_ok=True)
            (output/'vrx-state-view.json').write_text(json.dumps(report,indent=2)+'\n')
        time.sleep(.1)


if __name__=='__main__':main()
