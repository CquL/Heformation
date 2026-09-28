"""qn-local finite Action worker, advanced by completed plant steps.

Opt-in qualification endpoint. It shares the existing qn node lock and never
creates/resets a second plant or rolls out a candidate before acceptance.
Admission checks the actual entry and path; completion uses physical feedback.
Production qualification is explicit configuration.
"""
import math
import hashlib
import time
import json
import threading
from collections import deque
import actionlib
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from qn_aav_simulator.msg import PlatformTaskAction, PlatformTaskFeedback, PlatformTaskResult, FormationActionResult
from qn_aav_simulator.srv import TakeReference, TakeReferenceResponse
from qn_aav_simulator.platform_execution import ReferenceOwnership, Segment, actual_mode, validate_fragment, PlannerAcknowledgement, segment_terminal_ready, QN_PLATFORM_POSITION_TOLERANCE_M
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
        self.online_mapping=getattr(self.observation_request,'execution_mode','')=='ONLINE_MAPPING'
        self.allow_visual_timing_relaxation=bool(rospy.get_param('/mission/allow_visual_timing_relaxation',False))
        self.visual_timing_relaxed=False
        self.action_model_clock_drift_s=0.
        self.survey_map=None
        self.survey_lock=threading.Lock()
        self.survey_poses=deque(maxlen=400)
        self.survey_stamp=-math.inf
        if self.online_mapping:
            from qn_aav_simulator.observation_coverage import LocalSurveyMap
            self.survey_map=LocalSurveyMap()
            for box in rospy.get_param('/scene/known_free_deployment',[]):
                self.survey_map.declare_free_box(box['low'],box['high'])
            # Navigation policy is known in advance; physical SOLID geometry
            # remains unknown until measured and is never seeded here.
            for obj in rospy.get_param('/scene/objects',[]):
                if obj['kind']=='FORBIDDEN':
                    low=tuple(c-v/2 for c,v in zip(obj['center'],obj['size']))
                    high=tuple(c+v/2 for c,v in zip(obj['center'],obj['size']))
                    self.survey_map.declare_forbidden_box(low,high)
        self.observation_obstacles=tuple(ObstacleBox(c,s) for _,kind,c,s in self.scene.objects if kind=='SOLID') if self.scene else ()
        self.products=rospy.Publisher('~local_products',String,queue_size=100)
        self.hold_point=node.state.position
        self.hold_yaw=0.
        self.water_terminal_hold=False
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
        self.position_tolerance=float(rospy.get_param('~platform_position_tolerance_m',QN_PLATFORM_POSITION_TOLERANCE_M))
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
        if self.online_mapping:
            from sensor_msgs.msg import PointCloud2
            prefix='/'+node.agent_id+('_qn' if node.agent_id.startswith('drone_') else '')
            self.subscribers.append(rospy.Subscriber(prefix+'/survey_cloud',PointCloud2,self._survey_cloud,queue_size=1))
        if self.handover_enabled:
            self.subscribers.append(rospy.Subscriber('/'+node.agent_id+'_planning/safety_status',
                DiagnosticArray,self._planner_status,queue_size=1))
            endpoints=rospy.get_param('~air_action_endpoints',[
                '/aav_'+str(node.drone_id+1)+'/formation_action','/aav_formation/formation_action'])
            self.subscribers.extend(rospy.Subscriber(endpoint+'/result',FormationActionResult,
                self._air_result,queue_size=10) for endpoint in endpoints)
        self.server.start()

    def _survey_cloud(self,message):
        """Build measured map and query paths outside the physical-step lock."""
        from sensor_msgs import point_cloud2
        stamp=message.header.stamp.to_sec()
        if message.header.frame_id!=self.node.world_frame:return
        with self.node.lock:
            # AIR has its own active observation consumer. Do not duplicate
            # Python voxel integration here while no native fragment owns it.
            # Preserve the map; a later Goal still needs a fresh measured scan.
            if self.work is None:return
            if not self.survey_poses or stamp<=self.survey_stamp:return
            captured=next((row for row in reversed(self.survey_poses) if row[0]==stamp),None)
            if captured is None:return
            pose_time,origin=captured
        rows=tuple(point_cloud2.read_points(message,field_names=('x','y','z','intensity'),skip_nans=True))
        with self.survey_lock:
            self.survey_map.integrate(origin,[row[:3] for row in rows],[row[3]>.5 for row in rows],stamp,
                mode=actual_mode(medium_flag(origin[2],self.node.backend.constants.hg_m)))
            self.survey_stamp=stamp
            with self.node.lock:
                work=self.work
                if work is None or work['cause'] or work['waiting_start']:return
                index=work['index'];segment=work['segments'][index]
                position=tuple(self.node.state.position)
                reference=tuple(work['local_ref'])
                observations=work.get('observations')
                if work.get('region_id') and observations is not None:
                    from std_msgs.msg import String
                    for event in observations.sample_mapping(self.survey_map,self.mode(),stamp):
                        self.products.publish(String(data=json.dumps(event,allow_nan=False)))
                remaining=(observations.unobserved_mapping_points(self.survey_map,self.mode())
                    if work.get('region_id') and segment.operation=='WATER_PATH' and
                    observations is not None else None)
            goals=([segment.points[-1]] if not remaining else
                sorted(remaining,key=lambda point:math.dist(position,point)))
            radius=self.scene_radius+(self.scene.clearance if self.scene else 0.)+self.position_tolerance
            waypoint=None
            query_end=time.monotonic()+.04
            if rospy.Time.now().to_sec()-stamp<=1.:
                for goal in goals:
                    if segment.operation!='WATER_PATH':
                        # The accepted conversion is a fixed vertical line.
                        # Rebinding x/y to every measured pose removes the
                        # restoring reference and turns drift into a new goal.
                        if self.survey_map.segment_clear(position,goal,radius):waypoint=goal
                    elif (not remaining and
                          math.dist(position,segment.points[-1])<=self.position_tolerance and
                          math.dist(reference,segment.points[-1])<=self.position_tolerance):
                        # Already in the declared terminal region. Capture the
                        # accepted reference and decelerate; chasing a point a
                        # few centimetres behind creates an unnecessary pi turn.
                        waypoint=reference
                    else:
                        waypoint=self.survey_map.next_target(position,
                            (goal[0],goal[1],segment.points[-1][2]),radius,max_step_m=1.)
                    if waypoint is not None or time.monotonic()>=query_end:break
            # An observation or query that raced a Goal/segment change cannot
            # become the new reference. The qn state itself is never replaced.
            with self.node.lock:
                if self.work is not work or work['index']!=index or work['cause']:return
                if math.dist(self.node.state.position,position)>self.position_tolerance:return
                work['local_plan_seq']=work.get('local_plan_seq',0)+1
                work['local_plan_stamp']=stamp
                work['local_wait']=waypoint is None and bool(goals)
                work['local_waypoint']=tuple(waypoint) if waypoint is not None else work['local_ref']
                work['local_survey_done']=remaining is not None and not remaining

    def _mapping_reference(self,work,segment,t):
        """Advance the accepted short reference; no map search on the model tick."""
        position=self.node.state.position
        dt=max(0.,t-work.get('local_tick',t));work['local_tick']=t
        reference=work['local_ref'];waypoint=work['local_waypoint']
        if rospy.Time.now().to_sec()-work.get('local_plan_stamp',-math.inf)>1.:
            waypoint=reference;work['local_wait']=True
        distance=math.dist(reference,waypoint)
        if distance>1e-9:
            length=sum(math.dist(a,b) for a,b in zip(segment.points,segment.points[1:]))
            speed=((.13 if getattr(self.node,'platform_type','AAV')=='UUV' else .1)
                   if segment.operation=='WATER_PATH' else length/segment.duration if length else 0.)
            travel=min(distance,speed*dt)
            reference=tuple(reference[k]+travel*(waypoint[k]-reference[k])/distance for k in range(3))
            horizontal=math.hypot(waypoint[0]-work['local_ref'][0],waypoint[1]-work['local_ref'][1])
            if segment.operation=='WATER_PATH' and horizontal>self.position_tolerance:
                desired=math.atan2(waypoint[1]-work['local_ref'][1],waypoint[0]-work['local_ref'][0])
                limit=self.node.backend.water_heading_rate_limit_radps*dt
                self.hold_yaw+=max(-limit,min(limit,math.remainder(desired-self.hold_yaw,2*math.pi)))
        work['local_ref']=reference
        if work.get('region_id') and segment.operation=='WATER_PATH':
            done=(work.get('local_survey_done',False) and
                  math.dist(position,segment.points[-1])<=self.position_tolerance)
        elif segment.operation!='WATER_PATH':
            done=math.dist(position,segment.points[-1])<=self.position_tolerance
        else:done=math.dist(position,segment.points[-1])<=self.position_tolerance
        return reference,done

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
            # The explicitly enabled visual run may release a physically
            # verified AIR terminal with timing validity INCOMPLETE. This does
            # not rewrite the original time audit or accept safety failures.
            validity_ok=(result.experiment_validity==1 or
                (result.experiment_validity==3 and
                 bool(rospy.get_param('/mission/allow_visual_timing_relaxation',False))))
            verified=(message.status.status==3 and result.task_outcome==1 and
                      result.safety_outcome==1 and validity_ok and result.reason==0)
            self.owner.finish(message.status.goal_id.id,verified)
            if verified:self.terminal_mode='AIR'

    def planner_ready(self,paused):
        return not self.handover_enabled or self.planner.matches(
            self.owner.generation,paused,time.monotonic(),self.ack_timeout)

    def allowed_modes(self):
        if getattr(self.node,'platform_type','AAV')=='UUV':return frozenset({'WATER'})
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
        if self.online_mapping:
            self.survey_poses.append((self.node.state.timestamp_s,tuple(self.node.state.position)))
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
        if self.work and self.work.get('observations') is not None and not self.work.get('region_id'):
            from std_msgs.msg import String
            work=self.work;now=rospy.Time.now().to_sec();t=self.node.clock.model_time_s
            valid=(not work['cause'] and not work['waiting_start'] and not self.owner.locked and
                   self.applied_source=='PLATFORM' and self.applied_generation==self.owner.generation and
                   (abs((t-work['clock_start'])-(now-work['ros_start']))<=DEFAULT_MAX_ABS_DRIFT_S or
                    self.allow_visual_timing_relaxation))
            for event in work['observations'].sample(t,self.node.state.position,self.mode(),now,valid):
                self.products.publish(String(data=json.dumps(event,allow_nan=False)))

    def mode(self):
        return actual_mode(self.node.state.medium_flag)

    def claim(self,request):
        with self.node.lock:
            if getattr(self.node,'platform_type','AAV')=='UUV' and request.source=='AIR_SWARM':
                return TakeReferenceResponse(False,self.owner.generation,'UUV_WATER_ONLY')
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
                    self.water_terminal_hold=False
                    self.floor_at_claim=self.last_air_id
                    self.air_floor=self.last_air_id
                    self.air_adopted=False
                    self.air_acknowledged=not self.handover_enabled
            return TakeReferenceResponse(accepted,self.owner.generation,reason)

    def _flush_reference(self):
        from qn_aav_simulator.qn_telemetry import CommandAdoptionBuffer
        previous=self.node.latest_command
        if previous is not None and previous.get('reference_source')=='AIR_SWARM':
            self.hold_yaw=float(previous['yaw_rad'])
        else:
            w,x,y,z=self.node.state.orientation_quat_wxyz
            self.hold_yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
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
                    if self.scene and not self.online_mapping:
                        reason=self.scene.path_violation(points,self.scene_radius)
                        if reason:raise ValueError(reason)
                permitted=(frozenset(('WATER_PATH',)) if getattr(self.node,'platform_type','AAV')=='UUV'
                           else frozenset(('ENTER_WATER','WATER_PATH','EXIT_WATER')))
                validate_fragment(segments,self.mode(),goal.terminal_behavior,permitted)
                for segment in segments:
                    if segment.operation=='WATER_PATH' and any(
                            actual_mode(medium_flag(p[2],self.node.backend.constants.hg_m))!='WATER'
                            for p in segment.points):
                        raise ValueError('water path leaves the required actual medium')
                    end_mode=actual_mode(medium_flag(segment.points[-1][2],self.node.backend.constants.hg_m))
                    if end_mode!=segment.target_mode:
                        raise ValueError('segment endpoint is outside its required actual medium')
                    if (self.transition_fault_behavior=='COMPLETE_ACCEPTED_VERTICAL_SEGMENT' and
                            segment.operation in ('ENTER_WATER','EXIT_WATER') and
                            any(p[:2]!=segment.points[0][:2] for p in segment.points)):
                        raise ValueError('transition fault continuation requires an accepted vertical segment')
                region_id=str(getattr(goal,'region_id',''))
                if region_id and not self.online_mapping:raise ValueError('REGION_REQUIRES_ONLINE_MAPPING')
                if self.online_mapping:
                    actual=tuple(self.node.state.position)
                    normalized=[]
                    for segment in segments:
                        end=segment.points[-1]
                        if segment.operation in ('ENTER_WATER','EXIT_WATER'):end=(actual[0],actual[1],end[2])
                        normalized.append(Segment(segment.operation,(actual,end),segment.duration))
                        actual=end
                    segments=normalized
                reason=self._entry_reason(segments)
                if reason:raise ValueError(reason)
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
            self.water_terminal_hold=False
            if self.online_mapping and segments[0].operation in ('ENTER_WATER','EXIT_WATER'):
                # A vertical mode conversion keeps the measured entry heading,
                # not the final geometric path's possibly different heading.
                w,x,y,z=self.node.state.orientation_quat_wxyz
                self.hold_yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
                self.water_terminal_hold=segments[0].operation=='EXIT_WATER'
            self.work=dict(handle=handle,id=ident,task=goal.task_id,segments=segments,index=0,
                start=self.node.clock.model_time_s,settled=None,
                clock_start=self.node.clock.model_time_s,ros_start=rospy.Time.now().to_sec(),
                waiting_start=self.handover_enabled or bool(getattr(goal,'prepare_only',False)),
                waiting_commit=bool(getattr(goal,'prepare_only',False)),fault_allowed=None,
                deadline=time.monotonic()+min(timeout,self.wall_limit),cause='',last_feedback=-1.,observations=observations,
                terminal_wait_s=terminal_wait_s,region_id=region_id,
                local_ref=tuple(self.node.state.position),local_waypoint=tuple(self.node.state.position),
                local_tick=self.node.clock.model_time_s,local_done=False,local_wait=True)
            handle.set_accepted('finite local fragment accepted')

    def _entry_reason(self,segments):
        state=self.node.state
        if self.node.domain_history.violation:return 'PERSISTENT_DOMAIN_VIOLATION'
        if self.scene_failure:return 'PERSISTENT_SCENE_SAFETY_FAILURE'
        if not all(math.isfinite(v) for v in (*state.position,*state.velocity)):
            return 'NONFINITE_ACTUAL_STATE'
        required='AIR' if segments[0].operation=='ENTER_WATER' else 'WATER'
        if self.mode()!=required:return 'ACTUAL_ENTRY_MODE_MISMATCH'
        if not self.online_mapping and math.dist(segments[0].points[0],state.position)>self.position_tolerance:
            return 'START_STATE_CHANGED'
        if math.sqrt(sum(v*v for v in state.velocity))>self.speed_tolerance:
            return 'FRAGMENT_ENTRY_NOT_SETTLED'
        if self.scene:
            reason=self.scene.violation(state.position,self.scene_radius)
            if reason:return reason
            for segment in (() if self.online_mapping else segments):
                reason=self.scene.path_violation(segment.points,self.scene_radius)
                if reason:return reason
        return ''

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
            reason=self._entry_reason(work['segments'])
            if reason:return StartPreparedActionResponse(False,reason)
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
                if self.online_mapping:
                    # A pre-observation wait is not elapsed conversion motion.
                    # Continue only the last observed-clear vertical reference,
                    # with its remaining length and the unchanged native rate.
                    start=self.work['local_ref'];end=self.work['local_waypoint']
                    length=math.dist(segment.points[0],segment.points[-1])
                    if length>0 and math.dist(start,end)>1e-9:
                        remaining=math.dist(start,end)/(length/segment.duration)
                        self.fault_transition=(Segment(segment.operation,(start,end),remaining),
                                               self.node.clock.model_time_s)
                else:self.fault_transition=(segment,self.work['start'])
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
        if terminal_verified and not work['cause'] and self.mode()=='WATER':
            # A completed native terminal keeps its accepted position reference.
            # Stop generating forward LOS recapture from millimetre residuals;
            # the same plant damps its remaining velocity and holds depth/yaw.
            self.water_terminal_hold=True
            w,x,y,z=self.node.state.orientation_quat_wxyz
            self.hold_yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
        # Missing business coverage does not invalidate a verified physical
        # terminal. Faults and unverified terminals still retain the local lock.
        self.owner.finish(work['id'],terminal_verified and not work['cause'])
        if terminal_verified and observations is not None and not work['cause']:
            from std_msgs.msg import String
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
        if self.observation_request is not None:
            from std_msgs.msg import String
            from qn_aav_simulator.observation_coverage import action_terminal_event
            terminal='SUCCEEDED' if normal else 'CANCELED' if work['cause']=='CANCEL_REQUEST' and terminal_verified else 'ABORTED'
            notice=action_terminal_event(
                self.observation_request,work['id'],self.node.agent_id,rospy.Time.now().to_sec(),
                terminal,result.task_completed,result.terminal_verified,result.resource_locked,
                result.reason)
            try:notice['terminal_state_digest']=hashlib.sha256(self.node.backend.execution_state_bytes()).hexdigest()
            except (TypeError,ValueError,OverflowError):pass  # terminal Result still travels; state remains unknown
            self.products.publish(String(data=json.dumps(notice,allow_nan=False)))

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
            elif self.online_mapping and not work['cause'] and not work['waiting_start']:
                target,mapping_done=self._mapping_reference(work,segment,t)
            else:
                target=self.hold_point if work['cause'] or work['waiting_start'] else segment.reference(elapsed)
            terminal_due=not work['waiting_start'] and (
                elapsed>=segment.duration if finishing else bool(work['cause']) or elapsed>=segment.duration)
            consistent=(self.mode() in self.allowed_modes() if work['cause'] and not finishing
                        else self.mode()==segment.target_mode)
            drift=abs((t-work['clock_start'])-(rospy.Time.now().to_sec()-work['ros_start']))
            self.action_model_clock_drift_s=drift
            time_valid=drift<=DEFAULT_MAX_ABS_DRIFT_S
            self.visual_timing_relaxed=self.visual_timing_relaxed or (
                self.allow_visual_timing_relaxation and not time_valid)
            # User-authorized visual mode relaxes only model-vs-ROS elapsed
            # drift. Physical settling and its duration use actual model ticks;
            # fresh input stamps, Goal identity and safety are still mandatory.
            terminal_time_ok=time_valid or self.allow_visual_timing_relaxation
            adopted=self.applied_source=='PLATFORM' and self.applied_generation==self.owner.generation
            settled=(terminal_due and consistent and
                adopted and terminal_time_ok and
                math.dist(self.node.state.position,target)<=self.position_tolerance and
                math.sqrt(sum(v*v for v in self.node.state.velocity))<=self.speed_tolerance)
            if self.online_mapping and not work['cause'] and not work['waiting_start']:
                settled=(mapping_done and adopted and terminal_time_ok and consistent and
                    math.dist(self.node.state.position,target)<=self.position_tolerance and
                    math.sqrt(sum(v*v for v in self.node.state.velocity))<=self.speed_tolerance)
            elif not work['cause'] and not work['waiting_start']:
                settled=(adopted and terminal_time_ok and segment_terminal_ready(segment,elapsed,
                    self.node.state.position,self.node.state.velocity,self.mode(),
                    self.position_tolerance,self.speed_tolerance))
            if settled:
                if work['settled'] is None:
                    work['settled']=t
                    if self.online_mapping and not work['cause'] and self.mode()=='WATER':
                        # The original position/speed/coverage/adoption checks
                        # have become true. Verify the following four seconds
                        # using the actual stationary input, rather than letting
                        # LOS rotate toward a millimetre capture error while a
                        # different AIR yaw is waiting at the mode boundary.
                        w,x,y,z=self.node.state.orientation_quat_wxyz
                        self.hold_yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
                        self.water_terminal_hold=True
            else:
                work['settled']=None
                if (self.online_mapping and not work['cause'] and segment.operation=='WATER_PATH'):
                    # Leaving the unchanged terminal region resumes ordinary
                    # guidance; the hold flag is never a success substitute.
                    self.water_terminal_hold=False
            extra_wait=work.get('terminal_wait_s',0.) if not work['cause'] and work['index']==len(work['segments'])-1 else 0.
            if work['settled'] is not None and t-work['settled']>=self.hold_duration+extra_wait:
                self.hold_point=target
                if work['cause'] or work['index']==len(work['segments'])-1:
                    self._finish(True,work['cause'] or 'COMPLETED_LOCAL_FRAGMENT')
                else:
                    work['index']+=1
                    work['start']=t
                    work['settled']=None
                    if self.online_mapping:
                        following=work['segments'][work['index']]
                        # The preceding four seconds verified its stationary
                        # position/speed and measured heading input. Keep that
                        # x/y and heading through a vertical mode handoff; do
                        # not restore the old geometric path heading.
                        begin=tuple(target)
                        end=following.points[-1]
                        if following.operation in ('ENTER_WATER','EXIT_WATER'):end=(begin[0],begin[1],end[2])
                        work['segments'][work['index']]=Segment(following.operation,(begin,end),following.duration)
                        if following.operation=='WATER_PATH':self.water_terminal_hold=False
                        work.update(local_ref=target,local_waypoint=target,local_done=False,
                                    local_plan_stamp=-math.inf,local_tick=t,local_survey_done=False)
            elif time.monotonic()>=work['deadline']:
                self.hold_point=target
                self._finish(False,'OBSERVATION_TIMEOUT_UNVERIFIED')
            if self.work is not None and t-work['last_feedback']>=.1:
                work['last_feedback']=t
                work['handle'].publish_feedback(PlatformTaskFeedback(
                    segment_index=work['index'],operation=('PREPARED' if work.get('waiting_commit',False) and self.planner_ready(True) else
                        'WAIT_PLANNER_ACK' if work['waiting_start'] else
                        'FAULT_FINISH_'+segment.operation if finishing else
                        'WAIT_LOCAL_OBSERVATION' if self.online_mapping and work.get('local_wait') else
                        'REGION_MAPPING' if work.get('region_id') else work['segments'][work['index']].operation),
                    reference_source=self.owner.source,actual_mode=self.mode(),
                    reference_generation=self.owner.generation,model_time_s=t))
        return dict(position=target,velocity=(0.,0.,0.),acceleration=(0.,0.,0.),
                    yaw_rad=self.hold_yaw if self.owner.source=='AIR_SWARM' or self.online_mapping else 0.,
                    stamp_s=rospy.Time.now().to_sec(),received_ros_time_s=rospy.Time.now().to_sec(),
                    trajectory_id=self.owner.generation if self.owner.source=='PLATFORM' else max(0,self.air_floor),
                    trajectory_flag=1 if self.owner.source=='PLATFORM' else 0,
                    reference_source='PLATFORM',reference_generation=self.owner.generation,
                    water_terminal_hold=self.water_terminal_hold and self.owner.source=='PLATFORM')

    def diagnostics(self):
        return [('reference_source',self.applied_source),('reference_generation',str(self.owner.generation)),
                ('transition_fault_behavior',self.transition_fault_behavior),
                ('scene_failure',self.scene_failure),
                ('water_terminal_hold',str(self.water_terminal_hold).lower()),
                ('visual_timing_relaxed',str(self.visual_timing_relaxed).lower()),
                ('action_model_clock_drift_s',str(self.action_model_clock_drift_s)),
                ('local_plan_seq',str(self.work.get('local_plan_seq',0) if self.work else 0)),
                ('local_plan_stamp',str(self.work.get('local_plan_stamp',0.) if self.work else 0.)),
                ('local_target',json.dumps(self.work.get('local_waypoint')) if self.work else 'null'),
                ('local_wait',str(self.work.get('local_wait',False) if self.work else False).lower()),
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
                ('platform_speed_tolerance_mps',str(self.speed_tolerance)),
                ('reference_context_ready',str(self.planner_ready(self.owner.source=='PLATFORM')).lower()),
                ('reference_air_floor_id',str(self.air_floor)),
                ('reference_goal_id',self.owner.goal_id),('actual_mode',self.mode()),
                ('platform_resource_locked',str(self.owner.locked)),
                ('platform_qualification_only',str(self.qualification_only))]
