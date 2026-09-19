#!/usr/bin/env python3
"""Two native PVS Actions in one ROS run; explicit qualification scope."""
import argparse
import json
from pathlib import Path
import time
import actionlib
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath,Odometry
from qn_aav_simulator.msg import PlatformTaskAction,PlatformTaskGoal,PlatformSegment


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--otter-trim',action='store_true')
    args=p.parse_args()
    rospy.init_node('probe_pvs_actions',anonymous=True)
    samples={'usv':[],'uuv':[]}
    clients={}
    for name,mode,depth in [('usv','SURFACE',0.),('uuv','WATER',-2.)]:
        def callback(msg,key=name):
            position=msg.pose.pose.position
            velocity=msg.twist.twist.linear
            samples[key].append([msg.header.stamp.to_sec(),position.x,position.y,position.z,velocity.x,velocity.y,velocity.z])
        rospy.Subscriber('/'+name+'/odometry',Odometry,callback,queue_size=1000)
        client=actionlib.SimpleActionClient('/'+name+'/platform_task',PlatformTaskAction)
        if not client.wait_for_server(rospy.Duration(30)):
            raise RuntimeError(name+' native Action missing')
        clients[name]=client
    for name,mode,depth in [('usv','SURFACE',0.),('uuv','WATER',-2.)]:
        path=RosPath()
        path.header.frame_id='map'
        for x in (-5.,0.):
            point=PoseStamped()
            point.pose.position.x=x
            point.pose.position.z=depth
            point.pose.orientation.w=1.
            path.poses.append(point)
        segment=PlatformSegment(operation=mode+'_PATH',path=path,duration=rospy.Duration(20))
        clients[name].send_goal(PlatformTaskGoal(task_id=name+'-pass',segments=[segment],
            terminal_behavior='TRIM_PROPULSION' if name=='usv' and args.otter_trim else 'COAST_STOP',
            execution_timeout=rospy.Duration(120)))
    report={'scope':'Otter/REMUS native numerical Action qualification; no business observation or fleet safety verdict','results':{}}
    deadline=time.monotonic()+135
    pending=set(clients)
    while pending and time.monotonic()<deadline:
        for name in list(pending):
            if clients[name].wait_for_result(rospy.Duration(.05)):
                result=clients[name].get_result()
                report['results'][name]={'status':clients[name].get_state(),
                    'result':None if result is None else {field:getattr(result,field) for field in result.__slots__}}
                pending.remove(name)
    time.sleep(2.)
    report['samples']=samples
    report['passed']=not pending and all(v['status']==3 and v['result']['terminal_verified'] and
        v['result']['task_completed'] and not v['result']['resource_locked'] for v in report['results'].values())
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'pvs-probe.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'passed':report['passed'],'results':report['results']},indent=2))
    return 0 if report['passed'] else 1

if __name__=='__main__':
    raise SystemExit(main())
