#!/usr/bin/env python3
"""Opt-in PVS qualification node: native motion and a finite path Action.

COAST_STOP means zero propulsion followed by measured low velocity, not position
holding. No collision-avoidance or finite-communication claim is made here.
"""
import math
import threading
import time
import actionlib
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from qn_aav_simulator.msg import PlatformTaskAction, PlatformTaskFeedback, PlatformTaskResult
from qn_aav_simulator.pvs_backend import PvsBackend


class PvsNode:
    def __init__(self):
        self.lock=threading.RLock()
        self.model=rospy.get_param('~model')
        self.agent_id=rospy.get_param('~agent_id')
        self.mode='SURFACE' if self.model=='otter' else 'WATER'
        self.dt=float(rospy.get_param('~outer_dt_s',.01))
        self.effort=float(rospy.get_param('~propulsion_effort'))
        initialization=rospy.get_param('~initialization_mode','NATIVE_ZERO')
        self.backend=PvsBackend(self.model,tuple(rospy.get_param('~initial_position')),
                                initialization_mode=initialization)
        self.terminal_behavior='TRIM_PROPULSION' if initialization=='STATIC_TRIM' else 'COAST_STOP'
        self.work=None
        self.locked=False
        self.generation=0
        self.retired=set()
        self.speed_limit=float(rospy.get_param('~terminal_speed_mps',.03))
        self.hold_seconds=float(rospy.get_param('~terminal_duration_s',4.))
        if any(not math.isfinite(v) or v<=0 for v in (self.dt,self.effort,self.speed_limit,self.hold_seconds)) or self.dt>.05:
            raise ValueError('invalid PVS qualification step/control/terminal limits')
        self.qualification_only=rospy.get_param('~qualification_only',True)
        if not self.qualification_only:
            raise ValueError('PVS production qualification is not yet registered')
        self.odom=rospy.Publisher('~odometry',Odometry,queue_size=1)
        self.diag=rospy.Publisher('~diagnostics',DiagnosticArray,queue_size=1)
        self.server=actionlib.ActionServer('~platform_task',PlatformTaskAction,self.goal,self.cancel,auto_start=False)
        self.server.start()

    def goal(self,handle):
        goal=handle.get_goal()
        ident=handle.get_goal_id().id
        with self.lock:
            try:
                if self.work is not None or self.locked or ident in self.retired:
                    raise ValueError('MEMBER_BUSY_OR_LOCKED')
                if self.backend.snapshot()['actual_mode']!=self.mode:
                    raise ValueError('ACTUAL_STATE_OUTSIDE_NATIVE_DOMAIN')
                if goal.terminal_behavior!=self.terminal_behavior:
                    raise ValueError('terminal behavior differs from the configured native trim/coast experiment')
                paths=[]
                for segment in goal.segments:
                    if segment.operation!=self.mode+'_PATH' or segment.path.header.frame_id!='map':
                        raise ValueError('operation/frame incompatible with native model')
                    points=[(p.pose.position.x,p.pose.position.y,p.pose.position.z) for p in segment.path.poses]
                    if len(points)<2 or any(not all(math.isfinite(v) for v in p) for p in points):
                        raise ValueError('finite nonempty path required')
                    if any((p[2]!=0 if self.mode=='SURFACE' else not -100<=p[2]<0) for p in points):
                        raise ValueError('path outside native medium')
                    if paths and math.dist(paths[-1][-1],points[0])>1e-9:
                        raise ValueError('fragment paths are disconnected')
                    if any(math.dist(a,b)==0 for a,b in zip(points,points[1:])):
                        raise ValueError('zero-length path leg')
                    paths.append(points)
                if not 1<=len(paths)<=16:
                    raise ValueError('fragment requires 1..16 segments')
                if math.dist(paths[0][0],self.backend.snapshot()['position'])>.2:
                    raise ValueError('path start must match actual position')
                timeout=goal.execution_timeout.to_sec()
                if not math.isfinite(timeout) or timeout<=0:
                    raise ValueError('finite positive timeout required')
            except ValueError as exc:
                handle.set_rejected(PlatformTaskResult(task_id=goal.task_id,goal_id=ident,
                    actual_mode=self.mode,reason=str(exc),resource_locked=self.locked))
                return
            self.generation+=1
            self.work=dict(handle=handle,id=ident,task=goal.task_id,paths=paths,segment=0,point=1,
                           coast=False,settled=None,cause='',deadline=time.monotonic()+min(timeout,180.))
            handle.set_accepted('finite native path fragment accepted')

    def cancel(self,handle):
        with self.lock:
            if self.work and handle.get_goal_id().id==self.work['id'] and not self.work['cause']:
                self.work.update(cause='CANCEL_REQUEST',coast=True,settled=None)
                self.locked=True

    def finish(self,verified,reason):
        w=self.work
        normal=verified and not w['cause']
        self.locked=self.locked or not normal
        self.retired.add(w['id'])
        result=PlatformTaskResult(task_id=w['task'],goal_id=w['id'],task_completed=normal,
            terminal_verified=verified,actual_mode=self.backend.snapshot()['actual_mode'],reason=reason,
            resource_locked=self.locked,model_time_s=self.backend.time_s)
        self.work=None
        if normal:w['handle'].set_succeeded(result)
        elif w['cause']=='CANCEL_REQUEST' and verified:w['handle'].set_canceled(result)
        else:w['handle'].set_aborted(result)

    def step(self):
        with self.lock:
            w=self.work
            target=None
            effort=0.
            if w and not w['coast']:
                points=w['paths'][w['segment']]
                start,target=points[w['point']-1:w['point']+1]
                direction=tuple(b-a for a,b in zip(start,target))
                position=self.backend.snapshot()['position']
                progress=sum((position[i]-start[i])*direction[i] for i in range(3))
                if progress>=sum(v*v for v in direction):
                    if w['point']+1<len(points):w['point']+=1
                    elif w['segment']+1<len(w['paths']):
                        w['segment']+=1
                        w['point']=1
                    else:w['coast']=True
                effort=0. if w['coast'] else self.effort
                if w['coast']:
                    target=None
            state=self.backend.step(self.dt,target,effort)
            if w and state['actual_mode']!=self.mode:
                self.finish(False,'ACTUAL_STATE_OUTSIDE_NATIVE_DOMAIN')
                w=None
            if w:
                speed=math.sqrt(sum(v*v for v in state['world_velocity']))
                if w['coast'] and speed<=self.speed_limit:
                    if w['settled'] is None:w['settled']=self.backend.time_s
                else:w['settled']=None
                if w['settled'] is not None and self.backend.time_s-w['settled']>=self.hold_seconds:
                    self.finish(True,w['cause'] or self.terminal_behavior+'_VERIFIED_IN_QUALIFICATION')
                elif time.monotonic()>=w['deadline']:
                    self.finish(False,'OBSERVATION_TIMEOUT_UNVERIFIED')
                elif self.backend.steps%10==0:
                    w['handle'].publish_feedback(PlatformTaskFeedback(segment_index=w['segment'],
                        operation=self.terminal_behavior if w['coast'] else self.mode+'_PATH',
                        reference_source='PVS_NATIVE',actual_mode=self.mode,
                        reference_generation=self.generation,model_time_s=self.backend.time_s))
            stamp=rospy.Time.now()
            odom=Odometry()
            odom.header.stamp=stamp
            odom.header.frame_id='map'
            odom.child_frame_id=self.agent_id+'/base_link'
            odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z=state['position']
            odom.pose.pose.orientation.w,odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z=state['quaternion_wxyz']
            odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z=state['body_velocity']
            odom.twist.twist.angular.x,odom.twist.twist.angular.y,odom.twist.twist.angular.z=state['body_angular_velocity']
            self.odom.publish(odom)
            values={**state,'resource_locked':self.locked,
                    'qualification_only':True,'reference_source':'PVS_NATIVE',
                    'terminal_behavior':self.terminal_behavior,'active_goal_id':self.work['id'] if self.work else '',
                    'safety_outcome':'NOT_VERIFIED'}
            status=DiagnosticStatus(name=self.agent_id+'/pvs',hardware_id=self.agent_id,
                message='native model qualification',values=[KeyValue(str(k),str(v)) for k,v in values.items()])
            self.diag.publish(DiagnosticArray(header=odom.header,status=[status]))

    def run(self):
        rate=rospy.Rate(1./self.dt)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


if __name__=='__main__':
    rospy.init_node('pvs_platform')
    PvsNode().run()
