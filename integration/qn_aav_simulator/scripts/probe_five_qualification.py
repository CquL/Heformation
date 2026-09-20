#!/usr/bin/env python3
"""Five actual native instances, fixed development actions in one ENU scene.

No allocation, observation, limited delivery or full environment-safety claim.
"""
import argparse
import json
from pathlib import Path
import time

import actionlib
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry,Path as RosPath
from diagnostic_msgs.msg import DiagnosticArray
from qn_aav_simulator.msg import FormationAction,FormationGoal,PlatformTaskAction,PlatformTaskGoal,PlatformSegment
from probe_swarm_roundtrip import Probe


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--scenario',choices=['normal','coast-obstacle','coast-preflight','pending-cancel','query-budget'],default='normal');args=p.parse_args()
    rospy.init_node('five_qualification_probe',anonymous=True)
    reference=Probe(args.output,'normal',final_fault_probe=False)
    identities=['drone_0','drone_1','drone_2','usv','uuv']
    samples={name:[] for name in identities};diagnostics={name:[] for name in identities}
    subscribers=[]
    for name in identities:
        prefix='/'+name+('_qn' if name.startswith('drone_') else '')
        def odom(msg,key=name):
            q=msg.pose.pose.position
            samples[key].append(dict(stamp=msg.header.stamp.to_sec(),position=[q.x,q.y,q.z],frame=msg.header.frame_id))
        def diag(msg,key=name):
            v={v.key:v.value for s in msg.status for v in s.values};v['stamp']=msg.header.stamp.to_sec()
            diagnostics[key].append(v)
        subscribers.extend([rospy.Subscriber(prefix+'/odometry',Odometry,odom,queue_size=1000),
            rospy.Subscriber(prefix+'/diagnostics',DiagnosticArray,diag,queue_size=1000)])
    clients={}
    report=dict(scope='G2 five-instance fixed-action integration; no MRTA/observation/delivery/full-safety verdict',scenario=args.scenario,passed=False,results={})
    try:
        reference.wait(lambda:all(rospy.get_param('/aav_'+str(i)+'_action_server/ready',False) for i in (1,2,3))
                       and all(samples.values()),180,'five instances and AIR readiness')
        if args.scenario in ('pending-cancel','query-budget'):
            pending_client=actionlib.SimpleActionClient('/uuv/platform_task',PlatformTaskAction)
            assert pending_client.wait_for_server(rospy.Duration(10))
            path=RosPath();path.header.frame_id='world'
            for xyz in [(-5.,8.,-2.),(0.,8.,-2.)]:
                pose=PoseStamped();pose.pose.orientation.w=1.
                pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=xyz;path.poses.append(pose)
            before=int(diagnostics['uuv'][-1]['steps'])
            generation=diagnostics['uuv'][-1]['reference_generation']
            pending_client.send_goal(PlatformTaskGoal(task_id='cancel-in-query',terminal_behavior='COAST_STOP',
                execution_timeout=rospy.Duration(150),segments=[PlatformSegment(operation='WATER_PATH',path=path,duration=rospy.Duration(20))]))
            if args.scenario=='query-budget':
                state,result=reference.result(pending_client,'query-budget',10)
                assert state==5 and result.reason.startswith('NATIVE_PREDICTION_UNKNOWN:') and not result.resource_locked
                reference.wait(lambda:int(diagnostics['uuv'][-1]['steps'])>before+2,2,'unblocked model clock')
                assert diagnostics['uuv'][-1]['reference_generation']==generation
                report['passed']=True
                return
            reference.wait(lambda:bool(diagnostics['uuv'][-1].get('pending_goal_id')),5,'pending native query')
            pending_client.cancel_goal()
            state,result=reference.result(pending_client,'cancel-in-query',10)
            assert state==8 and result.reason=='CANCELLED_BEFORE_ACCEPTANCE' and not result.resource_locked
            assert int(diagnostics['uuv'][-1]['steps'])>before
            assert diagnostics['uuv'][-1]['reference_generation']==generation
        if rospy.get_param('/scene/geometry_enabled',False):
            # These goals are rejected before physical commitment, not because
            # another action already owns the member.
            invalid=actionlib.SimpleActionClient('/uuv/platform_task',PlatformTaskAction)
            assert invalid.wait_for_server(rospy.Duration(10))
            cases=[('rock',(6.,1.,-3.)),('exclusion',(-12.,10.,-2.))]
            for label,end in cases:
                path=RosPath();path.header.frame_id='world'
                for xyz in [samples['uuv'][-1]['position'],end]:
                    pose=PoseStamped();pose.pose.orientation.w=1.
                    pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=xyz;path.poses.append(pose)
                invalid.send_goal(PlatformTaskGoal(task_id='reject-'+label,terminal_behavior='COAST_STOP',
                    execution_timeout=rospy.Duration(60),segments=[PlatformSegment(operation='WATER_PATH',path=path,duration=rospy.Duration(20))]))
                state,result=reference.result(invalid,'reject-'+label,10)
                assert state==5 and 'SCENE_' in result.reason and not result.resource_locked
        for number,y in [(2,4.),(3,8.)]:
            name='aav_'+str(number)
            client=actionlib.SimpleActionClient('/'+name+'/formation_action',FormationAction)
            assert client.wait_for_server(rospy.Duration(10))
            goal=FormationGoal(task_id=name+'-air',hold_duration=rospy.Duration(4))
            goal.formation_center.header.frame_id='world'
            goal.formation_center.header.stamp=rospy.Time.now()
            goal.formation_center.point.x=-28.;goal.formation_center.point.y=y;goal.formation_center.point.z=.8
            clients[name]=client;client.send_goal(goal)
        for name,mode,y,z,terminal in [('usv','SURFACE',-8.,0.,'TRIM_PROPULSION'),('uuv','WATER',8.,-2.,'COAST_STOP')]:
            client=actionlib.SimpleActionClient('/'+name+'/platform_task',PlatformTaskAction)
            assert client.wait_for_server(rospy.Duration(10))
            path=RosPath();path.header.frame_id='world'
            for x in (-5.,0.):
                pose=PoseStamped();pose.header.frame_id='world';pose.pose.orientation.w=1.
                pose.pose.position.x=x;pose.pose.position.y=y;pose.pose.position.z=z;path.poses.append(pose)
            goal=PlatformTaskGoal(task_id=name+'-pass',terminal_behavior=terminal,execution_timeout=rospy.Duration(150),
                segments=[PlatformSegment(operation=mode+'_PATH',path=path,duration=rospy.Duration(20))])
            clients[name]=client;client.send_goal(goal)
        reference.run()
        deadline=time.monotonic()+50
        for name,client in clients.items():
            state,result=reference.result(client,name,max(0.,deadline-time.monotonic()))
            report['results'][name]={field:getattr(result,field) for field in result.__slots__}
            for k,v in report['results'][name].items():
                if hasattr(v,'to_sec'):report['results'][name][k]=v.to_sec()
            if name=='uuv' and args.scenario=='coast-preflight':
                assert state==5 and not result.task_completed and not result.resource_locked
                assert result.reason.startswith('NATIVE_PREDICTION_INFEASIBLE:') and 'coast_rock' in result.reason
                assert all(abs(row['position'][0]+5.)<.2 for row in samples['uuv'])
            elif name=='uuv' and args.scenario=='coast-obstacle':
                assert state==4 and not result.task_completed and result.resource_locked
                assert result.reason=='SCENE_SAFETY_VIOLATION'
                assert any('coast_rock' in v.get('scene_failure','') for v in diagnostics['uuv'])
                path=RosPath();path.header.frame_id='world'
                start=samples['uuv'][-1]['position']
                for xyz in [start,[start[0]+1.,start[1],start[2]]]:
                    pose=PoseStamped();pose.pose.orientation.w=1.
                    pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=xyz;path.poses.append(pose)
                client.send_goal(PlatformTaskGoal(task_id='blocked-after-safety-failure',terminal_behavior='COAST_STOP',
                    execution_timeout=rospy.Duration(60),segments=[PlatformSegment(operation='WATER_PATH',path=path,duration=rospy.Duration(20))]))
                retry_state,retry=reference.result(client,'uuv-locked-retry',10)
                assert retry_state==5 and retry.resource_locked and retry.reason=='MEMBER_BUSY_OR_LOCKED'
            else:
                assert state==3
                if name.startswith('aav'):assert result.safety_outcome==1 and result.experiment_validity==1
                else:assert result.task_completed and result.terminal_verified and not result.resource_locked
        assert all({r['frame'] for r in records}=={'world'} for records in samples.values())
        assert all(len(records)>100 for records in samples.values())
        report['passed']=True
    except Exception as exc:
        report['error']=str(exc)
        raise
    finally:
        report.update(events=reference.events,samples=samples,diagnostics=diagnostics)
        (args.output/'five-integration.json').write_text(json.dumps(report,indent=2)+'\n')
        print('FIVE_INTEGRATION',args.scenario,report['passed'],flush=True)


if __name__=='__main__':main()
