#!/usr/bin/env python3
"""Real Swarm -> same qn native fragment -> NEW Swarm trajectory evidence.

Qualification scene: world local frame, existing 0.35 AIR configuration,
straight WATER motion at the previously measured -0.6 m depth. No payload claim.
"""
import argparse
import json
import math
from pathlib import Path
import threading
import time

import actionlib
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath, Odometry
from quadrotor_msgs.msg import PositionCommand
from std_srvs.srv import Trigger
from qn_aav_simulator.msg import FormationAction,FormationGoal,PlatformTaskAction,PlatformTaskGoal,PlatformSegment
from qn_aav_simulator.srv import TakeReference


class Probe:
    def __init__(self,output,scenario):
        self.output=output
        self.scenario=scenario
        self.lock=threading.RLock()
        self.rows=[]
        self.latest={}
        self.status={}
        self.positions={}
        self.events=[]
        self.subs=[rospy.Subscriber('/drone_0_qn/diagnostics',DiagnosticArray,self.diag,queue_size=1000),
                   rospy.Subscriber('/drone_0_planning/safety_status',DiagnosticArray,self.planner,queue_size=1000)]
        self.subs.extend(rospy.Subscriber('/drone_'+str(i)+'_qn/odometry',Odometry,self.odom,callback_args=i,queue_size=20) for i in range(3))
        self.air=actionlib.SimpleActionClient('/aav_1/formation_action',FormationAction)
        self.native=actionlib.SimpleActionClient('/drone_0_qn_aav/platform_task',PlatformTaskAction)
        self.old_reference=rospy.Publisher('/drone_0_planning/pos_cmd',PositionCommand,queue_size=1)
        self.old_goal=rospy.Publisher('/drone_0_member_goal',PoseStamped,queue_size=1)

    def diag(self,message):
        for status in message.status:
            values={v.key:v.value for v in status.values}
            if 'model_time_s' not in values:continue
            values['ros_stamp']=message.header.stamp.to_sec()
            with self.lock:
                self.rows.append(values)
                self.latest=values

    def planner(self,message):
        with self.lock:
            for status in message.status:self.status={v.key:v.value for v in status.values}

    def odom(self,message,member):
        point=message.pose.pose.position
        with self.lock:self.positions[member]=(message.header.stamp.to_sec(),(point.x,point.y,point.z))

    def wait(self,predicate,timeout,description):
        deadline=time.monotonic()+timeout
        while not rospy.is_shutdown() and time.monotonic()<deadline:
            if predicate():return
            time.sleep(.02)
        raise RuntimeError(description+' timed out; qn='+str(self.latest)+' planner='+str(self.status))

    def air_goal(self,label,point,ready=True):
        if ready:self.wait(lambda:rospy.get_param('/aav_1_action_server/ready',False),180,'AIR readiness')
        goal=FormationGoal(task_id=label,hold_duration=rospy.Duration(4))
        goal.formation_center.header.frame_id='world'
        goal.formation_center.header.stamp=rospy.Time.now()
        goal.formation_center.point.x,goal.formation_center.point.y,goal.formation_center.point.z=point
        self.air.send_goal(goal)

    def result(self,client,label,timeout=180):
        deadline=time.monotonic()+timeout
        lost=None
        while not client.wait_for_result(rospy.Duration(.2)):
            if time.monotonic()>deadline:raise RuntimeError(label+' missing Result')
            if label=='air-before' and self.latest.get('reference_context_ready')=='false':
                if lost is None:lost=time.monotonic()
                if time.monotonic()-lost>3:
                    raise RuntimeError('AIR planner confirmation lost; inspect native process/stack before retrying')
            else:lost=None
        result=client.get_result()
        if result is None:raise RuntimeError(label+' Result is None')
        values={field:getattr(result,field) for field in result.__slots__}
        for k,v in values.items():
            if hasattr(v,'to_sec'):values[k]=v.to_sec()
        self.events.append(dict(label=label,state=client.get_state(),result=values,received=rospy.Time.now().to_sec()))
        print(label,client.get_state(),values,flush=True)
        return client.get_state(),result

    def fragment(self):
        with self.lock:
            x,y,z=self.positions[0][1]
        goal=PlatformTaskGoal(task_id='native-roundtrip',terminal_behavior='FIXED_REFERENCE',execution_timeout=rospy.Duration(150))
        # The first point is the actual handover position; only references change.
        end=(x+.7,y,z)
        specs=[('ENTER_WATER',[(x,y,z),(x,y,-.6)],14.),
               ('WATER_PATH',[(x,y,-.6),(end[0],y,-.6)],7.),
               ('EXIT_WATER',[(end[0],y,-.6),end],14.)]
        for operation,points,duration in specs:
            path=RosPath()
            path.header.frame_id='world'
            for point in points:
                pose=PoseStamped()
                pose.header.frame_id='world'
                pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=point
                pose.pose.orientation.w=1.
                path.poses.append(pose)
            goal.segments.append(PlatformSegment(operation=operation,path=path,duration=rospy.Duration(duration)))
        return goal,end

    def inject_old(self,trajectory_id,stamp):
        command=PositionCommand()
        command.header.stamp=rospy.Time.now()
        command.header.frame_id='world'
        command.trajectory_id=trajectory_id
        command.trajectory_flag=1
        command.position.x=99.
        command.position.z=.8
        self.old_reference.publish(command)
        point=PoseStamped()
        point.header.frame_id='world'
        point.header.stamp=rospy.Time.from_sec(stamp)
        point.pose.position.x=99.
        point.pose.position.z=.8
        point.pose.orientation.w=1.
        self.old_goal.publish(point)

    def run(self):
        assert self.air.wait_for_server(rospy.Duration(30))
        assert self.native.wait_for_server(rospy.Duration(30))
        self.air_goal('air-before',(-28.,6.,.8))
        if self.scenario=='cancel-air':
            self.wait(lambda:math.sqrt(sum(float(self.latest.get('world_velocity_'+a,0))**2 for a in 'xyz'))>.05,30,'AIR moving')
            self.air.cancel_goal()
            state,_=self.result(self.air,'air-canceled')
            assert state==2
            self.wait(lambda:self.latest.get('platform_resource_locked','').lower()=='true',5,'local fault lock')
            return
        state,result=self.result(self.air,'air-before')
        assert state==3 and result.safety_outcome==1 and result.experiment_validity==1
        self.wait(lambda:self.latest.get('reference_active')=='false',5,'AIR resource release')
        before_id=int(self.latest['source_trajectory_id'])
        old_stamp=rospy.Time.now().to_sec()
        goal,end=self.fragment()
        self.native.send_goal(goal)
        self.wait(lambda:self.latest.get('actual_mode')=='WATER',60,'actual WATER')
        assert self.status.get('ordinary_reference_paused')=='true'
        self.inject_old(before_id,old_stamp)
        self.air_goal('forbidden-during-native',(-25.,6.,.8),ready=False)
        state,_=self.result(self.air,'air-rejected-during-native',10)
        assert state==5
        response=rospy.ServiceProxy('/drone_0_qn_aav/take_reference',TakeReference)(
            'foreign-air','AIR_SWARM',int(self.latest['reference_generation']))
        assert not response.accepted
        if self.scenario=='cancel-native':
            self.native.cancel_goal()
            self.native.cancel_goal()
        elif self.scenario=='missing-planner':
            import rosnode
            success,failure=rosnode.kill_nodes(['/drone_0_ego_planner_node'])
            assert success and not failure
        state,result=self.result(self.native,'native-fragment')
        if self.scenario!='normal':
            assert state==(2 if self.scenario=='cancel-native' else 4)
            assert not result.task_completed and result.resource_locked
            return
        assert state==3 and result.task_completed and result.terminal_verified
        assert result.actual_mode=='AIR'
        assert float(self.latest['min_height_m'])<0 and float(self.latest['max_medium_flag'])==1.
        assert self.latest['air_domain_violation']=='false'
        assert self.latest['domain_violation']=='false'
        self.air_goal('air-after',(end[0]+2.,end[1],.8))
        state,result=self.result(self.air,'air-after')
        assert state==3 and result.safety_outcome==1 and result.experiment_validity==1
        self.wait(lambda:self.latest.get('reference_active')=='false',5,'second AIR release')
        assert self.latest['reference_source']=='AIR_SWARM'
        assert int(self.latest['source_trajectory_id'])>before_id
        assert self.status['ordinary_reference_paused']=='false'
        self.inject_old(before_id,old_stamp)
        time.sleep(.5)
        assert float(self.latest['reference_position_x'])<0
        assert self.latest['air_domain_violation']=='false'
        # A real safety latch must survive the completed NORMAL roundtrip.
        rospy.ServiceProxy('/drone_0_planning/safety_hold',Trigger)()
        self.wait(lambda:self.latest.get('platform_resource_locked','').lower()=='true',5,'persistent safety lock')
        response=rospy.ServiceProxy('/drone_0_qn_aav/take_reference',TakeReference)(
            'forbidden-after-fault','AIR_SWARM',int(self.latest['reference_generation']))
        assert not response.accepted


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--scenario',choices=['normal','cancel-air','cancel-native','missing-planner'],default='normal')
    args=p.parse_args()
    rospy.init_node('probe_swarm_roundtrip',anonymous=True)
    probe=Probe(args.output,args.scenario)
    result={'scenario':args.scenario,'scope':'G1 single-member roundtrip; other AAVs remain present/idle; no five-platform task claim'}
    try:
        probe.run()
        result['passed']=True
    except Exception as error:
        result.update(passed=False,error=str(error))
        raise
    finally:
        with probe.lock:
            result.update(events=probe.events,final=probe.latest,planner=probe.status,samples=probe.rows)
        args.output.mkdir(parents=True,exist_ok=True)
        (args.output/'roundtrip.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        print('ROUNDTRIP',args.scenario,result['passed'],flush=True)


if __name__=='__main__':main()
