#!/usr/bin/env python3
"""Three native participants accept first, then execute existing finite paths.

Qualification evidence only: this does not claim online communications or MRTA.
"""
import argparse
import json
import math
from pathlib import Path
import time
from concurrent.futures import ThreadPoolExecutor

import actionlib
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry,Path as RosPath
from geometry_msgs.msg import PoseStamped
from qn_aav_simulator.msg import PlatformTaskAction,PlatformTaskGoal,PlatformSegment
from qn_aav_simulator.srv import StartPreparedAction


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    rospy.init_node('prepared_fragments_probe',anonymous=True)
    endpoints={'drone_0':'/drone_0_qn_aav','usv':'/usv','uuv':'/uuv'}
    states={};diagnostics={};subs=[];clients={}
    def odom(msg,key):
        p=msg.pose.pose.position;states[key]=(p.x,p.y,p.z)
    def diag(msg,key):diagnostics[key]={v.key:v.value for s in msg.status for v in s.values}
    def wait_for(check,seconds,why):
        end=time.monotonic()+seconds
        while not rospy.is_shutdown():
            if check():return
            if time.monotonic()>=end:raise RuntimeError('timeout: '+why)
            time.sleep(.05)
        raise RuntimeError('ROS shutdown: '+why)
    report={'scope':'native preparation/start boundary only','passed':False,'results':{}}
    try:
        for key,endpoint in endpoints.items():
            prefix='/drone_0_qn' if key=='drone_0' else endpoint
            subs.extend([rospy.Subscriber(prefix+'/odometry',Odometry,lambda m,k=key:odom(m,k),queue_size=1),
                rospy.Subscriber(prefix+'/diagnostics',DiagnosticArray,lambda m,k=key:diag(m,k),queue_size=1)])
            client=actionlib.SimpleActionClient(endpoint+'/platform_task',PlatformTaskAction)
            assert client.wait_for_server(rospy.Duration(20));clients[key]=client
        wait_for(lambda:len(states)==3 and rospy.get_param('/aav_1_action_server/ready',False),120,'initial readiness')
        initial=dict(states)
        for key,client in clients.items():
            x,y,z=initial[key]
            if key=='drone_0':
                raw=[('ENTER_WATER',[(x,y,z),(x,y,-.6)],14.),
                     ('WATER_PATH',[(x,y,-.6),(x+.7,y,-.6)],7.),
                     ('EXIT_WATER',[(x+.7,y,-.6),(x+.7,y,z)],14.)]
                terminal='FIXED_REFERENCE'
            else:
                level=0. if key=='usv' else z
                raw=[('SURFACE_PATH' if key=='usv' else 'WATER_PATH',[(x,y,level),(x+5.,y,level)],20.)]
                terminal='TRIM_PROPULSION' if key=='usv' else 'COAST_STOP'
            goal=PlatformTaskGoal(task_id='prepared-'+key,terminal_behavior=terminal,
                execution_timeout=rospy.Duration(150),prepare_only=True)
            for operation,points,duration in raw:
                path=RosPath();path.header.frame_id='world'
                for xyz in points:
                    point=PoseStamped();point.pose.orientation.w=1.
                    point.pose.position.x,point.pose.position.y,point.pose.position.z=xyz;path.poses.append(point)
                goal.segments.append(PlatformSegment(operation=operation,path=path,duration=rospy.Duration(duration)))
            client.send_goal(goal)
        wait_for(lambda:all(diagnostics.get(k,{}).get('execution_phase')=='PREPARED' for k in clients),20,'all fragments prepared')
        before={k:dict(diagnostics[k]) for k in clients};position=dict(states)
        start=time.monotonic()
        wait_for(lambda:time.monotonic()-start>=2.,3,'prepared observation interval')
        assert all(clients[k].get_state()==1 and diagnostics[k]['execution_phase']=='PREPARED' for k in clients)
        assert all(math.dist(position[k],states[k])<.2 for k in clients)
        report['all_prepared_without_execution']=True
        services={k:rospy.ServiceProxy(e+'/start_prepared',StartPreparedAction) for k,e in endpoints.items()}
        ids={k:before[k]['reference_goal_id' if k=='drone_0' else 'active_goal_id'] for k in clients}
        generations={k:int(before[k]['reference_generation']) for k in clients}
        assert all(not services[k]('old-goal',generations[k]).accepted for k in clients)
        assert all(not services[k](ids[k],generations[k]+1).accepted for k in clients)
        with ThreadPoolExecutor(max_workers=3) as pool:
            replies=list(pool.map(lambda k:services[k](ids[k],generations[k]),clients))
        assert all(r.accepted for r in replies)
        assert all(services[k](ids[k],generations[k]).accepted for k in clients)
        report['wrong_identity_rejected_and_release_idempotent']=True
        deadline=time.monotonic()+150
        for k,client in clients.items():
            assert client.wait_for_result(rospy.Duration(max(.1,deadline-time.monotonic())))
            result=client.get_result()
            report['results'][k]={'state':client.get_state(),'completed':result.task_completed,
                'terminal_verified':result.terminal_verified,'locked':result.resource_locked,'reason':result.reason}
            assert client.get_state()==3 and result.task_completed and result.terminal_verified and not result.resource_locked
        report['passed']=True
    except Exception as error:
        report['error']=str(error);raise
    finally:
        args.output.mkdir(parents=True,exist_ok=True)
        (args.output/'prepared-fragments.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report),flush=True)


if __name__=='__main__':main()
