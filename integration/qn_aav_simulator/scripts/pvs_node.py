#!/usr/bin/env python3
"""Opt-in PVS qualification node: native motion and a finite path Action.

COAST_STOP means zero propulsion followed by measured low velocity, not position
holding. Declared static geometry is checked during motion and coasting; this
detects violations and locks the member, not autonomous collision avoidance.
"""
import math
import threading
import time
import copy
import json
import actionlib
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from qn_aav_simulator.msg import PlatformTaskAction, PlatformTaskFeedback, PlatformTaskResult
from qn_aav_simulator.pvs_backend import PvsBackend,advance_path_target,NATIVE_START_TOLERANCE_M
from qn_aav_simulator.experiment_verdict import StaticSceneGeometry
from mrta_python.executors import ExecutorTravelTimeProvider,bounded_travel_query


class PvsNode:
    def __init__(self):
        self.lock=threading.RLock()
        self.model=rospy.get_param('~model')
        self.agent_id=rospy.get_param('~agent_id')
        # Both names denote a declared ENU scene origin; no numeric frame
        # transform is hidden here. Native NED/FRD conversion remains in PVS.
        self.world_frame=str(rospy.get_param('~world_frame','map'))
        if not self.world_frame:raise ValueError('declared scene frame is required')
        self.mode='SURFACE' if self.model=='otter' else 'WATER'
        self.dt=float(rospy.get_param('~outer_dt_s',.01))
        self.effort=float(rospy.get_param('~propulsion_effort'))
        initialization=rospy.get_param('~initialization_mode','NATIVE_ZERO')
        self.backend=PvsBackend(self.model,tuple(rospy.get_param('~initial_position')),
                                initialization_mode=initialization)
        self.terminal_behavior=self.backend.terminal_behavior
        self.scene=StaticSceneGeometry.from_mapping(rospy.get_param('/scene',{}))
        if self.scene and self.scene.frame!=self.world_frame:raise ValueError('scene frame mismatch')
        self.scene_failure=''
        from qn_aav_simulator.task_line import load_request
        from qn_aav_simulator.observation_coverage import ObstacleBox
        request_file=rospy.get_param('/mission/request_file','')
        self.observation_request=load_request(request_file) if request_file else None
        self.observation_obstacles=tuple(ObstacleBox(c,s) for _,kind,c,s in self.scene.objects if kind=='SOLID') if self.scene else ()
        self.products=rospy.Publisher('~local_products',String,queue_size=100)
        self.domain_failure=False
        self.work=None
        self.pending=None
        self.last_prediction={}
        self.native_preflight=bool(rospy.get_param('~native_preflight',self.scene is not None))
        self.planning_budget_s=float(rospy.get_param('/formation_mission_runner/planning_budget_s',10.))
        if not math.isfinite(self.planning_budget_s) or self.planning_budget_s<=0:
            raise ValueError('positive finite planning budget required')
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
        from qn_aav_simulator.srv import StartPreparedAction
        self.start_service=rospy.Service('~start_prepared',StartPreparedAction,self.start_prepared)
        self.server.start()

    def goal(self,handle):
        goal=handle.get_goal()
        ident=handle.get_goal_id().id
        with self.lock:
            try:
                if self.work is not None or self.pending is not None or self.locked or ident in self.retired:
                    raise ValueError('MEMBER_BUSY_OR_LOCKED')
                if self.backend.snapshot()['actual_mode']!=self.mode:
                    raise ValueError('ACTUAL_STATE_OUTSIDE_NATIVE_DOMAIN')
                if goal.terminal_behavior!=self.terminal_behavior:
                    raise ValueError('terminal behavior differs from the configured native trim/coast experiment')
                paths=[]
                for segment in goal.segments:
                    if segment.operation!=self.mode+'_PATH' or segment.path.header.frame_id!=self.world_frame:
                        raise ValueError('operation/frame incompatible with native model')
                    points=[(p.pose.position.x,p.pose.position.y,p.pose.position.z) for p in segment.path.poses]
                    if len(points)<2 or any(not all(math.isfinite(v) for v in p) for p in points):
                        raise ValueError('finite nonempty path required')
                    if any((p[2]!=0 if self.mode=='SURFACE' else not -100<=p[2]<0) for p in points):
                        raise ValueError('path outside native medium')
                    if paths and math.dist(paths[-1][-1],points[0])>1e-9:
                        raise ValueError('fragment paths are disconnected')
                    stationary=len(goal.segments)==1 and len(points)==2 and points[0]==points[1]
                    if not stationary and any(math.dist(a,b)==0 for a,b in zip(points,points[1:])):
                        raise ValueError('zero-length path leg')
                    if self.scene:
                        reason=self.scene.path_violation(points,self.backend.collision_radius_m)
                        if reason:raise ValueError(reason)
                    paths.append(points)
                if not 1<=len(paths)<=16:
                    raise ValueError('fragment requires 1..16 segments')
                if math.dist(paths[0][0],self.backend.snapshot()['position'])>NATIVE_START_TOLERANCE_M:
                    raise ValueError('path start must match actual position')
                timeout=goal.execution_timeout.to_sec()
                if not math.isfinite(timeout) or timeout<=0:
                    raise ValueError('finite positive timeout required')
                wait=getattr(goal,'terminal_wait',None)
                terminal_wait_s=wait.to_sec() if wait is not None else 0.
                if not math.isfinite(terminal_wait_s) or not 0<=terminal_wait_s<min(timeout,180.):
                    raise ValueError('terminal wait must fit the observation deadline')
                from qn_aav_simulator.observation_coverage import LocalObservationWindow
                ids=getattr(goal,'observation_ids',())
                observations=(LocalObservationWindow(self.observation_request,ids,self.agent_id,ident,
                    self.observation_obstacles) if ids else None)
            except ValueError as exc:
                handle.set_rejected(PlatformTaskResult(task_id=goal.task_id,goal_id=ident,
                    actual_mode=self.backend.snapshot()['actual_mode'],reason=str(exc),resource_locked=self.locked,
                    model_time_s=self.backend.time_s))
                return
            if self.native_preflight:
                query_deadline=time.monotonic()+self.planning_budget_s
                token=dict(handle=handle,id=ident,task=goal.task_id,paths=paths,timeout=timeout,
                    generation=self.generation,backend=copy.deepcopy(self.backend),
                    deadline=query_deadline,prepare_only=bool(getattr(goal,'prepare_only',False)),observations=observations,
                    terminal_wait_s=terminal_wait_s)
                self.pending=token
                # ActionServer invokes callbacks while holding its lock. The
                # query must run outside BOTH locks so cancel and integration
                # remain live. A token prevents a late query committing a new goal.
                threading.Thread(target=self._preflight,args=(token,),daemon=True).start()
            else:self._accept(handle,ident,goal.task_id,paths,timeout,bool(getattr(goal,'prepare_only',False)),observations,terminal_wait_s)

    def _accept(self,handle,ident,task,paths,timeout,prepare_only=False,observations=None,terminal_wait_s=0.):
        self.generation+=1
        self.work=dict(handle=handle,id=ident,task=task,paths=paths,segment=0,point=1,
                       coast=False,settled=None,cause='',model_start=self.backend.time_s,waiting_commit=prepare_only,
                       deadline=time.monotonic()+min(timeout,180.),observations=observations,ros_start=rospy.Time.now().to_sec(),
                       terminal_wait_s=terminal_wait_s)
        if self.last_prediction.get('goal_id')==ident:
            self.last_prediction['accepted_model_time_s']=self.backend.time_s
        handle.set_accepted('finite native path fragment accepted')

    def _preflight(self,token):
        try:
            prediction=bounded_travel_query(ExecutorTravelTimeProvider.query_native_fragment,
                (token['backend'],token['paths'],self.effort,self.scene,token['deadline'],
                 self.dt,self.speed_limit,self.hold_seconds+token['terminal_wait_s'],min(token['timeout'],180.)),token['deadline'])
        except Exception as exc:
            prediction=dict(status='UNKNOWN',reason=str(exc))
        # Same order as actionlib goal/cancel callbacks; never model->actionlib.
        with self.server.lock,self.lock:
            if self.pending is not token:return
            self.pending=None
            self.last_prediction={k:v for k,v in prediction.items() if k not in ('trajectory','terminal_backend','source_fingerprint','settled_model_time_s','terminal_wait_s')}
            self.last_prediction['goal_id']=token['id']
            state=self.backend.snapshot()
            reason=''
            if self.locked or self.generation!=token['generation']:
                reason='STATE_OR_COMMITMENT_CHANGED'
            elif (state['actual_mode']!=self.mode or
                  math.dist(state['position'],token['paths'][0][0])>NATIVE_START_TOLERANCE_M):
                reason='STATE_CHANGED_DURING_QUERY'
            elif prediction['status']!='FEASIBLE':
                reason='NATIVE_PREDICTION_'+prediction['status']+': '+prediction['reason']
            if reason:
                token['handle'].set_rejected(PlatformTaskResult(task_id=token['task'],goal_id=token['id'],
                    actual_mode=state['actual_mode'],reason=reason,resource_locked=self.locked,
                    model_time_s=self.backend.time_s))
            else:self._accept(token['handle'],token['id'],token['task'],token['paths'],token['timeout'],token['prepare_only'],token['observations'],token['terminal_wait_s'])

    def start_prepared(self,request):
        from qn_aav_simulator.srv import StartPreparedActionResponse
        with self.server.lock,self.lock:
            work=self.work
            reason=''
            if work is None or work['id']!=request.goal_id:reason='GOAL_NOT_ACTIVE'
            elif request.expected_generation!=self.generation:reason='GENERATION_MISMATCH'
            elif self.locked or work['cause']:reason='MEMBER_LOCKED'
            elif time.monotonic()>=work['deadline']:reason='PREPARATION_EXPIRED'
            if reason:return StartPreparedActionResponse(False,reason)
            if not work['waiting_commit']:return StartPreparedActionResponse(True,'ALREADY_STARTED')
            state=self.backend.snapshot()
            if state['actual_mode']!=self.mode:return StartPreparedActionResponse(False,'ACTUAL_MODE_MISMATCH')
            if math.dist(work['paths'][0][0],state['position'])>NATIVE_START_TOLERANCE_M:
                return StartPreparedActionResponse(False,'START_STATE_CHANGED')
            work['waiting_commit']=False
            work['model_start']=self.backend.time_s
            work['ros_start']=rospy.Time.now().to_sec()
            return StartPreparedActionResponse(True,'START_ACCEPTED')

    def cancel(self,handle):
        with self.lock:
            if self.pending is not None and handle.get_goal_id().id==self.pending['id']:
                token=self.pending;self.pending=None
                self.retired.add(token['id'])
                handle.set_canceled(PlatformTaskResult(task_id=token['task'],goal_id=token['id'],
                    reason='CANCELLED_BEFORE_ACCEPTANCE',actual_mode=self.backend.snapshot()['actual_mode'],
                    resource_locked=self.locked,model_time_s=self.backend.time_s))
                return
            if self.work and handle.get_goal_id().id==self.work['id'] and not self.work['cause']:
                self.work.update(cause='CANCEL_REQUEST',coast=True,settled=None,waiting_commit=False)
                self.locked=True

    def finish(self,verified,reason):
        w=self.work
        normal=verified and not w['cause']
        observations=w.get('observations')
        if normal and observations is not None and observations.emitted!=set(observations.points):
            normal=False;reason='OBSERVATION_NOT_SATISFIED'
        self.locked=self.locked or not normal
        self.retired.add(w['id'])
        if self.last_prediction.get('goal_id')==w['id']:
            self.last_prediction['actual_duration_s']=self.backend.time_s-w['model_start']
            self.last_prediction['actual_terminal_position']=self.backend.snapshot()['position']
        result=PlatformTaskResult(task_id=w['task'],goal_id=w['id'],task_completed=normal,
            terminal_verified=verified,actual_mode=self.backend.snapshot()['actual_mode'],reason=reason,
            resource_locked=self.locked,model_time_s=self.backend.time_s)
        self.work=None
        if normal:w['handle'].set_succeeded(result)
        elif w['cause']=='CANCEL_REQUEST' and verified:w['handle'].set_canceled(result)
        else:w['handle'].set_aborted(result)

    def step(self):
        with self.server.lock,self.lock:
            w=self.work
            target=None
            effort=0.
            if w and not w['coast'] and not w['waiting_commit']:
                w['segment'],w['point'],target,w['coast']=advance_path_target(
                    w['paths'],w['segment'],w['point'],self.backend.snapshot()['position'])
                effort=0. if w['coast'] else self.effort
                if w['coast']:
                    target=None
            state=self.backend.step(self.dt,target,effort)
            if self.scene:
                reason=self.scene.violation(state['position'],self.backend.collision_radius_m)
                if reason:
                    self.scene_failure=self.scene_failure or reason
                    self.locked=True
                    if w and w['cause']!='SCENE_SAFETY_VIOLATION':
                        w.update(cause='SCENE_SAFETY_VIOLATION',coast=True,settled=None,waiting_commit=False)
            if state['actual_mode']!=self.mode:
                self.domain_failure=True
                self.locked=True
                if w:
                    self.finish(False,'ACTUAL_STATE_OUTSIDE_NATIVE_DOMAIN')
                    w=None
            if w:
                observations=w.get('observations')
                if observations is not None:
                    from qn_aav_simulator.time_alignment import DEFAULT_MAX_ABS_DRIFT_S
                    now=rospy.Time.now().to_sec()
                    valid=(not w['cause'] and not w['waiting_commit'] and not self.locked and
                           abs((self.backend.time_s-w['model_start'])-(now-w['ros_start']))<=DEFAULT_MAX_ABS_DRIFT_S)
                    for event in observations.sample(self.backend.time_s,state['position'],state['actual_mode'],now,valid):
                        self.products.publish(String(data=json.dumps(event,allow_nan=False)))
                speed=math.sqrt(sum(v*v for v in state['world_velocity']))
                if w['coast'] and speed<=self.speed_limit:
                    if w['settled'] is None:w['settled']=self.backend.time_s
                else:w['settled']=None
                extra_wait=w.get('terminal_wait_s',0.) if not w['cause'] else 0.
                if w['settled'] is not None and self.backend.time_s-w['settled']>=self.hold_seconds+extra_wait:
                    self.finish(True,w['cause'] or self.terminal_behavior+'_VERIFIED_IN_QUALIFICATION')
                elif time.monotonic()>=w['deadline']:
                    self.finish(False,'OBSERVATION_TIMEOUT_UNVERIFIED')
                elif self.backend.steps%10==0:
                    w['handle'].publish_feedback(PlatformTaskFeedback(segment_index=w['segment'],
                    operation='PREPARED' if w['waiting_commit'] else self.terminal_behavior if w['coast'] else self.mode+'_PATH',
                        reference_source='PVS_NATIVE',actual_mode=self.mode,
                        reference_generation=self.generation,model_time_s=self.backend.time_s))
            stamp=rospy.Time.now()
            odom=Odometry()
            odom.header.stamp=stamp
            odom.header.frame_id=self.world_frame
            odom.child_frame_id=self.agent_id+'/base_link'
            odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z=state['position']
            odom.pose.pose.orientation.w,odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z=state['quaternion_wxyz']
            odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z=state['body_velocity']
            odom.twist.twist.angular.x,odom.twist.twist.angular.y,odom.twist.twist.angular.z=state['body_angular_velocity']
            self.odom.publish(odom)
            values={**state,'resource_locked':self.locked,
                    'pending_goal_id':self.pending['id'] if self.pending else '',
                    'native_prediction':json.dumps(self.last_prediction),
                    'reference_generation':self.generation,
                    'execution_phase':('PREPARED' if self.work and self.work['waiting_commit'] else
                        self.terminal_behavior if self.work and self.work['coast'] else
                        self.mode+'_PATH' if self.work else 'IDLE'),
                    'scene_failure':self.scene_failure,
                    'scene_check':'FAIL' if self.scene_failure else 'DISCRETE_SAMPLED' if self.scene else 'NOT_CONFIGURED',
                    'qualification_only':True,'reference_source':'PVS_NATIVE',
                    'terminal_behavior':self.terminal_behavior,'active_goal_id':self.work['id'] if self.work else '',
                    'domain_failure':self.domain_failure,
                    'safety_outcome':'FAIL' if self.scene_failure or self.domain_failure else 'NOT_VERIFIED'}
            status=DiagnosticStatus(name=self.agent_id+'/pvs',hardware_id=self.agent_id,
                message='native model qualification',values=[KeyValue(str(k),str(v)) for k,v in values.items()])
            self.diag.publish(DiagnosticArray(header=odom.header,status=[status]))

    def run(self):
        if rospy.get_param('/use_sim_time',False):
            raise ValueError('PVS wall-time execution requires use_sim_time=false; no external clock adapter is configured')
        next_tick=time.monotonic()
        while not rospy.is_shutdown():
            self.step()
            next_tick+=self.dt
            delay=next_tick-time.monotonic()
            if delay>0:time.sleep(delay)


if __name__=='__main__':
    rospy.init_node('pvs_platform')
    PvsNode().run()
