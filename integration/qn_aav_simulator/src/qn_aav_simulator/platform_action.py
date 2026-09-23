"""qn-local finite Action worker, advanced by completed plant steps.

Opt-in qualification endpoint. It shares the existing qn node lock and never
creates/resets a second plant. Production qualification is explicit configuration.
"""
import math
import time
import json
import actionlib
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from qn_aav_simulator.msg import PlatformTaskAction, PlatformTaskFeedback, PlatformTaskResult, FormationActionResult
from qn_aav_simulator.srv import TakeReference, TakeReferenceResponse
from qn_aav_simulator.platform_execution import ReferenceOwnership, Segment, actual_mode, validate_fragment, PlannerAcknowledgement, segment_terminal_ready
from qn_aav_simulator.qn_dynamics import medium_flag
from qn_aav_simulator.time_alignment import DEFAULT_MAX_ABS_DRIFT_S
from qn_aav_simulator.experiment_verdict import StaticSceneGeometry


class LocalPlatformAction:
    def __init__(self,node):
        self.node=node
        self.owner=ReferenceOwnership()
        self.work=None
        self.fault_hold_modes=None
        self.fault_hold_operation=''
        self.fault_transition=None
        self.scene=StaticSceneGeometry.from_mapping(rospy.get_param('/scene',{}))
        if self.scene and self.scene.frame!=node.world_frame:raise ValueError('scene frame mismatch')
        self.scene_radius=float(rospy.get_param('~platform_radius_m',.25))
        self.scene_failure=''
        from qn_aav_simulator.task_line import load_request
        from qn_aav_simulator.observation_coverage import ObstacleBox
        from std_msgs.msg import String
        request_file=rospy.get_param('/mission/request_file','')
        self.observation_request=load_request(request_file) if request_file else None
        self.observation_obstacles=tuple(ObstacleBox(c,s) for _,kind,c,s in self.scene.objects if kind=='SOLID') if self.scene else ()
        self.products=rospy.Publisher('~local_products',String,queue_size=100)
        self.hold_point=node.state.position
        self.last_air_id=-1
        self.air_floor=-1
        self.floor_at_claim=-1
        self.air_adopted=True
        self.air_acknowledged=False
        self.applied_source='INITIAL_HOLD'
        self.applied_generation=0
        self.terminal_mode=actual_mode(medium_flag(node.state.position[2],node.backend.constants.hg_m))
        if self.terminal_mode!='AIR':
            self.owner.source='PLATFORM'
            self.air_adopted=False
        self.handover_enabled=bool(rospy.get_param('~reference_handover_enabled',False))
        self.planner=PlannerAcknowledgement()
        self.planner_stamp=0.
        self.ack_timeout=float(rospy.get_param('~reference_status_timeout_s',1.))
        self.qualification_only=bool(rospy.get_param('~qualification_only',True))
        self.transition_fault_behavior=str(rospy.get_param('~transition_fault_behavior','FIXED_REFERENCE'))
        if self.transition_fault_behavior not in ('FIXED_REFERENCE','COMPLETE_ACCEPTED_VERTICAL_SEGMENT'):
            raise ValueError('unsupported transition fault behavior')
        if not self.qualification_only:
            raise ValueError('Swarm round-trip and mode fault qualification are incomplete; this endpoint is experimental only')
        self.position_tolerance=float(rospy.get_param('~platform_position_tolerance_m',.2))
        self.speed_tolerance=float(rospy.get_param('~platform_speed_tolerance_mps',.03))
        self.hold_duration=float(rospy.get_param('~platform_terminal_duration_s',4.))
        self.wall_limit=float(rospy.get_param('~platform_observation_timeout_s',180.))
        if any(not math.isfinite(v) or v<=0 for v in [self.position_tolerance,self.speed_tolerance,self.hold_duration,self.wall_limit]):
            raise ValueError('platform observation limits must be finite and positive')
        self.server=actionlib.ActionServer('~platform_task',PlatformTaskAction,self.goal,self.cancel,auto_start=False)
        self.service=rospy.Service('~take_reference',TakeReference,self.claim)
        from qn_aav_simulator.srv import StartPreparedAction
        self.start_service=rospy.Service('~start_prepared',StartPreparedAction,self.start_prepared)
        self.subscribers=[]
        if self.handover_enabled:
            self.subscribers.append(rospy.Subscriber('/'+node.agent_id+'_planning/safety_status',
                DiagnosticArray,self._planner_status,queue_size=1))
            endpoints=rospy.get_param('~air_action_endpoints',[
                '/aav_'+str(node.drone_id+1)+'/formation_action','/aav_formation/formation_action'])
            self.subscribers.extend(rospy.Subscriber(endpoint+'/result',FormationActionResult,
                self._air_result,queue_size=10) for endpoint in endpoints)
        self.server.start()

    def _planner_status(self,message):
        stamp=message.header.stamp.to_sec()
        if not 0<=rospy.Time.now().to_sec()-stamp<=self.ack_timeout:return
        for status in message.status:
            if status.hardware_id!=self.node.agent_id:continue
            values={v.key:v.value for v in status.values}
            with self.node.lock:
                if stamp<self.planner_stamp:return
                self.planner_stamp=stamp
                self.planner.note(values,time.monotonic())
                if self.planner.latched:
                    if self.work:self._begin_disposition('PLANNER_SAFETY_LATCH')
                    else:self.owner.locked=True
                if self.owner.source=='AIR_SWARM' and not self.air_acknowledged and self.planner_ready(False):
                    self.air_floor=max(self.floor_at_claim,int(values.get('reference_floor_id',-1)))
                    self.air_acknowledged=True

    def _air_result(self,message):
        with self.node.lock:
            if self.owner.source!='AIR_SWARM' or message.status.goal_id.id!=self.owner.goal_id:return
            result=message.result
            verified=(message.status.status==3 and result.task_outcome==1 and
                      result.safety_outcome==1 and result.experiment_validity==1 and result.reason==0)
            self.owner.finish(message.status.goal_id.id,verified)
            if verified:self.terminal_mode='AIR'

    def planner_ready(self,paused):
        return not self.handover_enabled or self.planner.matches(
            self.owner.generation,paused,time.monotonic(),self.ack_timeout)

    def allowed_modes(self):
        # A Result ends observation, not the persistent local fault behavior.
        # Retain the authorized fault phase instead of reclassifying a
        # transition hold as AIR from its last instantaneous medium sample.
        if self.owner.source=='PLATFORM' and self.fault_hold_modes is not None:
            return self.fault_hold_modes
        if self.work and self.owner.source=='PLATFORM':
            if self.work.get('fault_allowed') is not None:return self.work['fault_allowed']
            if self.work.get('waiting_start'):return frozenset({self.terminal_mode})
            operation=self.work['segments'][self.work['index']].operation
            return frozenset({'WATER'}) if operation=='WATER_PATH' else frozenset({'AIR','TRANSITION','WATER'})
        return frozenset({'AIR' if self.owner.source=='AIR_SWARM' else self.terminal_mode})

    def note_adopted(self,snapshot):
        self.applied_source=snapshot.reference_source
        self.applied_generation=snapshot.reference_generation
        if self.scene and self.owner.source=='PLATFORM':
            reason=self.scene.violation(self.node.state.position,self.scene_radius)
            if reason:
                self.scene_failure=self.scene_failure or reason
                if self.work:self._begin_disposition('SCENE_SAFETY_VIOLATION')
                else:self.owner.locked=True
        if self.node.domain_history.violation:
            if self.work:self._begin_disposition('DOMAIN_VIOLATION')
            else:self.owner.locked=True
        if self.work and self.work.get('observations') is not None:
            from std_msgs.msg import String
            work=self.work;now=rospy.Time.now().to_sec();t=self.node.clock.model_time_s
            valid=(not work['cause'] and not work['waiting_start'] and not self.owner.locked and
                   self.applied_source=='PLATFORM' and self.applied_generation==self.owner.generation and
                   abs((t-work['clock_start'])-(now-work['ros_start']))<=DEFAULT_MAX_ABS_DRIFT_S)
            for event in work['observations'].sample(t,self.node.state.position,self.mode(),now,valid):
                self.products.publish(String(data=json.dumps(event,allow_nan=False)))

    def mode(self):
        return actual_mode(self.node.state.medium_flag)

    def claim(self,request):
        with self.node.lock:
            if self.work is not None and request.goal_id != self.work['id']:
                return TakeReferenceResponse(False,self.owner.generation,'MEMBER_BUSY')
            if request.source=='AIR_SWARM' and self.mode()!='AIR':
                return TakeReferenceResponse(False,self.owner.generation,'ACTUAL_MODE_NOT_AIR')
            if request.source=='AIR_SWARM' and self.owner.source=='PLATFORM' and not self.handover_enabled:
                return TakeReferenceResponse(False,self.owner.generation,'SWARM_ROUND_TRIP_NOT_QUALIFIED')
            if self.planner.latched or self.node.domain_history.violation:
                return TakeReferenceResponse(False,self.owner.generation,'PERSISTENT_LOCAL_FAULT')
            if request.source=='AIR_SWARM' and math.sqrt(sum(v*v for v in self.node.state.velocity))>self.speed_tolerance:
                return TakeReferenceResponse(False,self.owner.generation,'AIR_ENTRY_NOT_SETTLED')
            # External clients cannot install an unvalidated platform program.
            if request.source=='PLATFORM' and self.work is None:
                return TakeReferenceResponse(False,self.owner.generation,'NO_ACCEPTED_PLATFORM_ACTION')
            before=self.owner.generation
            accepted,reason=self.owner.claim(request.goal_id,request.source,request.expected_generation)
            if accepted and before!=self.owner.generation:
                self._flush_reference()
                if request.source=='AIR_SWARM':
                    self.floor_at_claim=self.last_air_id
                    self.air_floor=self.last_air_id
                    self.air_adopted=False
                    self.air_acknowledged=not self.handover_enabled
            return TakeReferenceResponse(accepted,self.owner.generation,reason)

    def _flush_reference(self):
        from qn_aav_simulator.qn_telemetry import CommandAdoptionBuffer
        self.node.commands=CommandAdoptionBuffer(self.node.command_buffer_size)
        self.node.latest_command=None
        self.hold_point=self.node.state.position

    def accepts_air(self,trajectory_id):
        self.last_air_id=max(self.last_air_id,trajectory_id)
        if self.owner.source!='AIR_SWARM':return False
        # A persistent safety latch must still allow its exact stop trajectory.
        if self.planner.latched:
            accepted=(self.planner.values.get('reference_published')=='true' and
                      str(trajectory_id)==self.planner.values.get('trajectory_id'))
        else:
            accepted=self.planner_ready(False) and trajectory_id>self.air_floor
        if accepted:self.air_adopted=True
        return accepted

    def goal(self,handle):
        goal=handle.get_goal()
        ident=handle.get_goal_id().id
        with self.node.lock:
            try:
                if self.work is not None or self.owner.locked or self.owner.active or self.node.domain_history.violation:
                    raise ValueError('MEMBER_BUSY_OR_LOCKED')
                if self.handover_enabled and not self.planner_ready(self.owner.source=='PLATFORM'):
                    raise ValueError('PLANNER_CONTEXT_NOT_CONFIRMED')
                segments=[]
                for raw in goal.segments:
                    if raw.path.header.frame_id!=self.node.world_frame:
                        raise ValueError('path frame must match the declared scene frame: '+self.node.world_frame)
                    points=tuple((p.pose.position.x,p.pose.position.y,p.pose.position.z) for p in raw.path.poses)
                    segments.append(Segment(raw.operation,points,raw.duration.to_sec()))
                    if self.scene:
                        reason=self.scene.path_violation(points,self.scene_radius)
                        if reason:raise ValueError(reason)
                permitted=frozenset(('ENTER_WATER','WATER_PATH','EXIT_WATER'))
                validate_fragment(segments,self.mode(),goal.terminal_behavior,permitted)
                for segment in segments:
                    end_mode=actual_mode(medium_flag(segment.points[-1][2],self.node.backend.constants.hg_m))
                    if end_mode!=segment.target_mode:
                        raise ValueError('segment endpoint is outside its required actual medium')
                    if (self.transition_fault_behavior=='COMPLETE_ACCEPTED_VERTICAL_SEGMENT' and
                            segment.operation in ('ENTER_WATER','EXIT_WATER') and
                            any(p[:2]!=segment.points[0][:2] for p in segment.points)):
                        raise ValueError('transition fault continuation requires an accepted vertical segment')
                if math.dist(segments[0].points[0],self.node.state.position)>self.position_tolerance:
                    raise ValueError('path start does not match actual position')
                if math.sqrt(sum(v*v for v in self.node.state.velocity))>self.speed_tolerance:
                    raise ValueError('fragment entry is not settled')
                timeout=goal.execution_timeout.to_sec()
                if not math.isfinite(timeout) or timeout<=0:
                    raise ValueError('positive finite execution timeout required')
                wait=getattr(goal,'terminal_wait',None)
                terminal_wait_s=wait.to_sec() if wait is not None else 0.
                if not math.isfinite(terminal_wait_s) or not 0<=terminal_wait_s<min(timeout,self.wall_limit):
                    raise ValueError('terminal wait must fit the observation deadline')
                from qn_aav_simulator.observation_coverage import LocalObservationWindow
                ids=getattr(goal,'observation_ids',())
                observations=(LocalObservationWindow(self.observation_request,ids,self.node.agent_id,ident,
                    self.observation_obstacles) if ids else None)
                ok,reason=self.owner.claim(ident,'PLATFORM',self.owner.generation)
                if not ok:
                    raise ValueError(reason)
            except ValueError as exc:
                handle.set_rejected(PlatformTaskResult(task_id=goal.task_id,goal_id=ident,
                    reason=str(exc),actual_mode=self.mode(),resource_locked=self.owner.locked,
                    model_time_s=self.node.clock.model_time_s))
                return
            self._flush_reference()
            self.work=dict(handle=handle,id=ident,task=goal.task_id,segments=segments,index=0,
                start=self.node.clock.model_time_s,settled=None,
                clock_start=self.node.clock.model_time_s,ros_start=rospy.Time.now().to_sec(),
                waiting_start=self.handover_enabled or bool(getattr(goal,'prepare_only',False)),
                waiting_commit=bool(getattr(goal,'prepare_only',False)),fault_allowed=None,
                deadline=time.monotonic()+min(timeout,self.wall_limit),cause='',last_feedback=-1.,observations=observations,
                terminal_wait_s=terminal_wait_s)
            handle.set_accepted('finite local fragment accepted')

    def cancel(self,handle):
        with self.node.lock:
            if self.work is None or handle.get_goal_id().id!=self.work['id']:
                return
            self._begin_disposition('CANCEL_REQUEST')

    def start_prepared(self,request):
        from qn_aav_simulator.srv import StartPreparedActionResponse
        with self.server.lock,self.node.lock:
            work=self.work
            reason=''
            if work is None or work['id']!=request.goal_id:reason='GOAL_NOT_ACTIVE'
            elif request.expected_generation!=self.owner.generation:reason='GENERATION_MISMATCH'
            elif self.owner.locked or work['cause']:reason='MEMBER_LOCKED'
            elif time.monotonic()>=work['deadline']:reason='PREPARATION_EXPIRED'
            elif not self.planner_ready(True):reason='PLANNER_CONTEXT_NOT_CONFIRMED'
            if reason:return StartPreparedActionResponse(False,reason)
            if not work.get('waiting_commit',False):
                return StartPreparedActionResponse(True,'ALREADY_STARTED')
            work['waiting_commit']=False
            # The next real model tick establishes the motion start; no reset
            # of dynamics, observation deadline or ownership generation.
            return StartPreparedActionResponse(True,'START_ACCEPTED')

    def _begin_disposition(self,reason):
        if not self.work:return
        if reason in ('DOMAIN_VIOLATION','SCENE_SAFETY_VIOLATION') and self.fault_transition is not None:
            # State validity overrides an earlier cancellation continuation.
            # Freeze the last accepted reference, never an unreliable position.
            segment,start=self.fault_transition
            self.hold_point=segment.reference(self.node.clock.model_time_s-start)
            self.fault_transition=None
            self.work['cause']=reason
            self.work['settled']=None
            return
        if not self.work['cause']:
            segment=self.work['segments'][self.work['index']]
            if (self.transition_fault_behavior=='COMPLETE_ACCEPTED_VERTICAL_SEGMENT' and
                    reason in ('CANCEL_REQUEST','PLANNER_CONTEXT_UNAVAILABLE') and
                    self.mode()=='TRANSITION' and not self.work['waiting_start'] and
                    segment.operation in ('ENTER_WATER','EXIT_WATER')):
                # Finish only this already accepted conversion, never the
                # remaining job. Keep its original clock, endpoint and path.
                self.fault_transition=(segment,self.work['start'])
            self.work['fault_allowed']=self.allowed_modes()
            self.fault_hold_modes=self.work['fault_allowed']
            self.fault_hold_operation=self.work['segments'][self.work['index']].operation
            self.work['settled']=None
            self.work['waiting_start']=False
            self.work['waiting_commit']=False
            self.hold_point=self.node.state.position
            self.owner.latch_fault()
            self.work['cause']=reason
        elif self.work['cause']=='CANCEL_REQUEST' and reason!='CANCEL_REQUEST':
            self.work['cause']=reason

    def _finish(self,terminal_verified,reason):
        work=self.work
        normal=not work['cause'] and terminal_verified
        observations=work.get('observations')
        observation_missing=normal and observations is not None and observations.emitted!=set(observations.points)
        if observation_missing:normal=False;reason='OBSERVATION_NOT_SATISFIED'
        if terminal_verified:self.terminal_mode=self.mode()
        # Missing business coverage does not invalidate a verified physical
        # terminal. Faults and unverified terminals still retain the local lock.
        self.owner.finish(work['id'],terminal_verified and not work['cause'])
        if terminal_verified and observations is not None and not work['cause']:
            self.products.publish(String(data=json.dumps(observations.terminal_report(rospy.Time.now().to_sec()),allow_nan=False)))
        result=PlatformTaskResult(task_id=work['task'],goal_id=work['id'],
            task_completed=normal,terminal_verified=terminal_verified,
            actual_mode=self.mode(),reason=reason,resource_locked=self.owner.locked,
            model_time_s=self.node.clock.model_time_s)
        self.work=None
        if normal:
            work['handle'].set_succeeded(result)
        elif work['cause']=='CANCEL_REQUEST' and terminal_verified:
            work['handle'].set_canceled(result)
        else:
            work['handle'].set_aborted(result)

    def tick(self):
        if self.owner.source=='AIR_SWARM' and self.air_adopted:
            return None
        work=self.work
        t=self.node.clock.model_time_s
        # The observation worker can time out before physical disposition ends.
        # Its Result must not discard the committed transition reference.
        target=(self.fault_transition[0].reference(t-self.fault_transition[1])
                if self.fault_transition is not None else self.hold_point)
        if work is not None:
            if work['waiting_start'] and not work.get('waiting_commit',False) and self.planner_ready(True):
                work['waiting_start']=False
                work['start']=t
            if not work['waiting_start'] and not work['cause'] and not self.planner_ready(True):
                self._begin_disposition('PLANNER_CONTEXT_UNAVAILABLE')
            segment=work['segments'][work['index']]
            elapsed=t-work['start']
            finishing=self.fault_transition is not None
            if finishing:
                target=self.fault_transition[0].reference(t-self.fault_transition[1])
            else:
                target=self.hold_point if work['cause'] or work['waiting_start'] else segment.reference(elapsed)
            terminal_due=not work['waiting_start'] and (
                elapsed>=segment.duration if finishing else bool(work['cause']) or elapsed>=segment.duration)
            consistent=(self.mode() in self.allowed_modes() if work['cause'] and not finishing
                        else self.mode()==segment.target_mode)
            drift=abs((t-work['clock_start'])-(rospy.Time.now().to_sec()-work['ros_start']))
            time_valid=drift<=DEFAULT_MAX_ABS_DRIFT_S
            adopted=self.applied_source=='PLATFORM' and self.applied_generation==self.owner.generation
            settled=(terminal_due and consistent and
                adopted and time_valid and
                math.dist(self.node.state.position,target)<=self.position_tolerance and
                math.sqrt(sum(v*v for v in self.node.state.velocity))<=self.speed_tolerance)
            if not work['cause'] and not work['waiting_start']:
                settled=(adopted and time_valid and segment_terminal_ready(segment,elapsed,
                    self.node.state.position,self.node.state.velocity,self.mode(),
                    self.position_tolerance,self.speed_tolerance))
            if settled:
                if work['settled'] is None:
                    work['settled']=t
            else:
                work['settled']=None
            extra_wait=work.get('terminal_wait_s',0.) if not work['cause'] and work['index']==len(work['segments'])-1 else 0.
            if work['settled'] is not None and t-work['settled']>=self.hold_duration+extra_wait:
                self.hold_point=target
                if work['cause'] or work['index']==len(work['segments'])-1:
                    self._finish(True,work['cause'] or 'COMPLETED_LOCAL_FRAGMENT')
                else:
                    work['index']+=1
                    work['start']=t
                    work['settled']=None
            elif time.monotonic()>=work['deadline']:
                self.hold_point=target
                self._finish(False,'OBSERVATION_TIMEOUT_UNVERIFIED')
            if self.work is not None and t-work['last_feedback']>=.1:
                work['last_feedback']=t
                work['handle'].publish_feedback(PlatformTaskFeedback(
                    segment_index=work['index'],operation=('PREPARED' if work.get('waiting_commit',False) and self.planner_ready(True) else
                        'WAIT_PLANNER_ACK' if work['waiting_start'] else
                        'FAULT_FINISH_'+segment.operation if finishing else work['segments'][work['index']].operation),
                    reference_source=self.owner.source,actual_mode=self.mode(),
                    reference_generation=self.owner.generation,model_time_s=t))
        return dict(position=target,velocity=(0.,0.,0.),acceleration=(0.,0.,0.),yaw_rad=0.,
                    stamp_s=rospy.Time.now().to_sec(),received_ros_time_s=rospy.Time.now().to_sec(),
                    trajectory_id=self.owner.generation if self.owner.source=='PLATFORM' else max(0,self.air_floor),
                    trajectory_flag=1 if self.owner.source=='PLATFORM' else 0,
                    reference_source='PLATFORM',reference_generation=self.owner.generation)

    def diagnostics(self):
        return [('reference_source',self.applied_source),('reference_generation',str(self.owner.generation)),
                ('transition_fault_behavior',self.transition_fault_behavior),
                ('scene_failure',self.scene_failure),
                ('execution_phase',('PREPARED' if self.work and self.work.get('waiting_commit',False) and self.planner_ready(True) else
                    'WAIT_PLANNER_ACK' if self.work and self.work.get('waiting_start',False) else
                    'FAULT_FINISH_'+self.fault_hold_operation if self.fault_transition is not None and
                    self.node.clock.model_time_s<self.fault_transition[1]+self.fault_transition[0].duration else
                    'FAULT_HOLD_AFTER_'+self.fault_hold_operation if self.fault_hold_modes is not None else
                    self.work['segments'][self.work['index']].operation if self.work is not None else
                    'AIR_SAFETY_HOLD' if self.owner.source=='AIR_SWARM' and self.planner.latched else
                    'AIR_MOVE' if self.owner.source=='AIR_SWARM' and self.owner.active else 'IDLE')),
                ('requested_reference_source',self.owner.source),
                ('adopted_reference_generation',str(self.applied_generation)),
                ('reference_active',str(self.owner.active).lower()),
                ('platform_action_active',str(self.work is not None).lower()),
                ('reference_handover_enabled',str(self.handover_enabled).lower()),
                ('reference_context_ready',str(self.planner_ready(self.owner.source=='PLATFORM')).lower()),
                ('reference_air_floor_id',str(self.air_floor)),
                ('reference_goal_id',self.owner.goal_id),('actual_mode',self.mode()),
                ('platform_resource_locked',str(self.owner.locked)),
                ('platform_qualification_only',str(self.qualification_only))]
