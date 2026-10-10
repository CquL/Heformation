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
import hashlib
from collections import deque
import actionlib
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger,TriggerResponse
from qn_aav_simulator.msg import PlatformTaskAction, PlatformTaskFeedback, PlatformTaskResult
from qn_aav_simulator.pvs_backend import PvsBackend,advance_path_target,NATIVE_START_TOLERANCE_M
from qn_aav_simulator.experiment_verdict import StaticSceneGeometry
from mrta_python.executors import ExecutorTravelTimeProvider,bounded_travel_query,close_query_worker


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
        self.simulation_speed=(float(rospy.get_param('/mission/simulation_speed',1.))
                               if rospy.get_param('/use_sim_time',False) else 1.)
        self.local_query_budget_s=float(rospy.get_param('~local_query_budget_s',.25))
        self.local_query_worker={}
        rospy.on_shutdown(lambda:close_query_worker(self.local_query_worker))
        if not math.isfinite(self.local_query_budget_s) or self.local_query_budget_s<=0:
            raise ValueError('local_query_budget_s must be finite and positive')
        self.effort=float(rospy.get_param('~propulsion_effort'))
        initialization=rospy.get_param('~initialization_mode','NATIVE_ZERO')
        self.backend=PvsBackend(self.model,tuple(rospy.get_param('~initial_position')),
                                heading_rad=float(rospy.get_param('~initial_heading_rad',0.)),
                                initialization_mode=initialization)
        rospy.set_param('/mission/collision_radii/'+self.agent_id,self.backend.collision_radius_m)
        self.peer_history={member:deque(maxlen=256) for member in
            ('drone_0','drone_1','drone_2','usv','uuv') if member!=self.agent_id}
        self.peer_subscribers=[rospy.Subscriber(
            '/'+member+('_qn' if member.startswith('drone_') else '')+'/odometry',
            Odometry,self._peer_odom,callback_args=member,queue_size=10)
            for member in self.peer_history]
        self.terminal_behavior=self.backend.terminal_behavior
        declared_scene=rospy.get_param('/scene',{})
        self.nominal_speed=float(declared_scene.get('inspection_motion_profile',{}).get('usv_nominal_speed_mps',.2))
        self.scene=StaticSceneGeometry.from_mapping(declared_scene)
        if self.scene and self.scene.frame!=self.world_frame:raise ValueError('scene frame mismatch')
        self.return_site=declared_scene.get('return_sites',{}).get(self.agent_id)
        if self.return_site is not None:
            point=self.return_site.get('position',())
            radius=self.return_site.get('radius_m')
            if (len(point)!=3 or not all(math.isfinite(v) for v in point) or
                    not isinstance(radius,(int,float)) or not math.isfinite(radius) or radius<=0):
                raise ValueError('invalid declared local return site')
        self.scene_failure=''
        from qn_aav_simulator.task_line import load_request
        from qn_aav_simulator.observation_coverage import ObstacleBox
        request_file=rospy.get_param('/mission/request_file','')
        self.observation_request_file=str(request_file)
        self.observation_request=load_request(request_file) if request_file else None
        self.observation_request_ids={self.observation_request.request_id} if self.observation_request else set()
        self.online_mapping=getattr(self.observation_request,'execution_mode','') in ('ONLINE_MAPPING','INSPECTION_CONTROL')
        self.allow_visual_timing_relaxation=bool(rospy.get_param('/mission/allow_visual_timing_relaxation',False))
        self.visual_timing_relaxed=False
        self.action_model_clock_drift_s=0.
        self.survey_map=None
        self.survey_lock=threading.Lock()
        self.local_plan_lock=threading.Lock()
        self.survey_poses=deque(maxlen=400)
        self.survey_stamp=-math.inf
        if self.online_mapping:
            from qn_aav_simulator.observation_coverage import LocalSurveyMap
            self.survey_map=LocalSurveyMap()
            for box in declared_scene.get('known_free_deployment',[]):
                self.survey_map.declare_free_box(box['low'],box['high'])
            for obj in declared_scene.get('objects',[]):
                if obj['kind']=='FORBIDDEN':
                    low=tuple(c-v/2 for c,v in zip(obj['center'],obj['size']))
                    high=tuple(c+v/2 for c,v in zip(obj['center'],obj['size']))
                    self.survey_map.declare_forbidden_box(low,high)
        self.observation_obstacles=tuple(ObstacleBox(c,s) for _,kind,c,s in self.scene.objects if kind=='SOLID') if self.scene else ()
        self.products=rospy.Publisher('~local_products',String,queue_size=100)
        self.state_digest_service=rospy.Service('~state_digest',Trigger,self.state_digest)
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
        self.diag=rospy.Publisher('~diagnostics',DiagnosticArray,queue_size=1,latch=bool(rospy.get_param('/use_sim_time',False)))
        self.server=actionlib.ActionServer('~platform_task',PlatformTaskAction,self.goal,self.cancel,auto_start=False)
        from qn_aav_simulator.srv import StartPreparedAction
        self.start_service=rospy.Service('~start_prepared',StartPreparedAction,self.start_prepared)
        if self.online_mapping:
            from sensor_msgs.msg import PointCloud2
            self.survey_subscriber=rospy.Subscriber('~survey_cloud',PointCloud2,self._survey_cloud,queue_size=1)
        self.server.start()

    def _peer_odom(self,message,member):
        from qn_aav_simulator.odometry import parse_standard_odometry,OdometryContractError
        if message.header.frame_id!=self.world_frame:return
        try:sample=parse_standard_odometry(message,member)
        except (OdometryContractError,ValueError,TypeError):return
        if not all(math.isfinite(value) for value in (*sample.position,*sample.velocity)):return
        with self.lock:
            history=self.peer_history[member]
            if not history or sample.stamp>history[-1].stamp:history.append(sample)

    def _peer_snapshot(self,position,stamp,horizon_s):
        from qn_aav_simulator.observation_coverage import local_peer_snapshot
        with self.lock:histories={member:tuple(rows) for member,rows in self.peer_history.items()}
        return local_peer_snapshot(histories,rospy.get_param_cached('/mission/collision_radii',{}),
            self.agent_id,stamp,position,self.backend.collision_radius_m,horizon_s,
            math.inf,
            rospy.get_param_cached('/mission/peer_speed_bounds',{}))

    def _survey_cloud(self,message):
        """Measured map and bounded A* stay outside the native-step lock."""
        from sensor_msgs import point_cloud2
        stamp=message.header.stamp.to_sec()
        if message.header.frame_id!=self.world_frame:return
        with self.lock:
            # Keep the prior/map but integrate only for a real accepted Goal.
            # Acceptance does not authorize motion from an old map timestamp.
            if self.work is None:return
            if not self.survey_poses or stamp<=self.survey_stamp:return
            captured=next((row for row in reversed(self.survey_poses) if row[0]==stamp),None)
            if captured is None:return
            pose_time,origin=captured
        rows=tuple(point_cloud2.read_points(message,field_names=('x','y','z','intensity'),skip_nans=True))
        with self.survey_lock:
            epoch=getattr(self,'observation_epoch_pending',None)
            if epoch is not None:
                self.survey_map.begin_observation_epoch(epoch)
                self.observation_epoch_pending=None
            self.survey_map.integrate(origin,[row[:3] for row in rows],[row[3]>.5 for row in rows],stamp,
                mode=self.mode)
            self.survey_stamp=stamp
        # Keep the subscriber free to integrate current measured input while
        # the bounded native query advances an immutable map/model copy.
        # One worker owns the existing local search; no scan queues a second.
        if self.local_plan_lock.acquire(blocking=False):
            threading.Thread(target=self._plan_mapping_command,daemon=True).start()

    def _plan_mapping_command(self):
        try:
            self._plan_mapping_command_snapshot()
        finally:
            self.local_plan_lock.release()

    def _plan_mapping_command_snapshot(self):
        with self.survey_lock:
            stamp=self.survey_stamp
            with self.lock:
                epoch=getattr(self,'observation_epoch_pending',None)
                if epoch is not None:
                    self.survey_map.begin_observation_epoch(epoch)
                    self.observation_epoch_pending=None
                work=self.work
                if work is None or work['cause'] or work['waiting_commit'] or work['coast']:return
                index=work['segment'];position=tuple(self.backend.snapshot()['position'])
                goals=[work['paths'][index][-1]]
                observations=work.get('observations')
                remaining=None
                if work.get('region_id') and observations is not None:
                    for event in observations.sample_mapping(self.survey_map,self.mode,stamp):
                        self.products.publish(String(data=json.dumps(event,allow_nan=False)))
                    remaining=observations.unobserved_mapping_points(self.survey_map,self.mode)
                    if remaining:goals=sorted(remaining,key=lambda point:math.dist(position,point))
            snapshot_began=time.monotonic()
            observed_map=self.survey_map.navigation_snapshot()
            snapshot_wall_s=time.monotonic()-snapshot_began
        target=None
        # Geometry proposes a route for the physical clearance envelope.
        # The native command+coast rollout below handles transient motion.
        # An extra static start margin would trap a stopped, physically safe
        # vessel just inside that optional margin and prevent its retreat.
        radius=self.backend.collision_radius_m+(self.scene.clearance if self.scene else 0.)
        try:
            observed_map.peer_states=self._peer_snapshot(position,rospy.Time.now().to_sec(),
                max(1.,2*self.backend.vehicle.L)/max(self.nominal_speed,1e-6)+self.hold_seconds)
            peer_error=''
        except ValueError as error:
            peer_error=str(error)
        observed_map.peer_body_radius_m=self.backend.collision_radius_m
        observed_map.peer_speed_mps=self.nominal_speed
        query_end=time.monotonic()+.04
        if not peer_error and rospy.Time.now().to_sec()-stamp<=1.:
            for goal in goals:
                target=observed_map.next_target(position,(goal[0],goal[1],position[2]),
                    radius,max_step_m=max(1.,2*self.backend.vehicle.L))
                if target is not None or time.monotonic()>=query_end:break
        with self.lock:
            if self.work is not work or work['segment']!=index or work['cause'] or work['coast']:return
            if math.dist(self.backend.snapshot()['position'],position)>NATIVE_START_TOLERANCE_M:return
            if self.backend.steps<work.get('local_command_end_step',-1):return
            # The live plant and prediction now share an exact native coast
            # prefix. Only a future model-step activation may adopt a new
            # command; no position-only stale-snapshot acceptance remains.
            self._end_local_command(work)
            work['local_pending_command']=False
            work['local_command_until']=-math.inf
            work['local_command_end_step']=-1
            work['local_wait']=True
            work['local_peer_status']=peer_error or 'FRESH_PEER_MOTION_CHECKED'
            if peer_error:return
            native=copy.deepcopy(self.backend)
            generation=self.generation
            requested=work['efforts'][index]
            stopped=math.sqrt(sum(v*v for v in native.snapshot()['world_velocity']))<=self.speed_limit
            retreat_first=(self.model=='otter' and stopped and
                'NATIVE_BRAKING_PATH_NOT_OBSERVED_FREE' in work.get('local_rejected_commands',''))
        # Geometry proposes a point, but the underactuated Otter must be
        # able to execute the next command AND its native coast backup.
        # The small local query runs outside the integration lock.
        motion_end=time.monotonic()+self.local_query_budget_s
        selected=None;selected_effort=0.;selected_leg=None;has_command=False
        prediction=dict(status='UNKNOWN',reason='NO_LOCAL_COMMAND')
        # At a tight observed corner, steering at zero additional surge is
        # a real native command too. It still needs the same complete coast
        # check; simply falling back to coast can leave a stopped vessel
        # unable to align with the next safe leg.
        facility=getattr(self.observation_request,'execution_mode','')=='INSPECTION_CONTROL'
        # Live native steps continue during the wall-clock query. Keep
        # the original coast margin beyond that declared query duration,
        # converted to the same model-time units as the future digest.
        # A deadline is an upper computation bound, not a mandatory wait.
        # Reuse the last complete query's measured wall latency; a late result
        # still fails the original exact future-step/state-digest adoption.
        latency=min(self.local_query_budget_s,work.get('local_query_wall_s',self.local_query_budget_s))
        coast_delay=self.simulation_speed*latency+(.5 if facility else .25)
        local_duration=(max(1.,math.hypot(target[0]-position[0],target[1]-position[1])/self.nominal_speed)
                        if target is not None else 1.)
        # Define a desired local leg duration from the declared nominal speed;
        # the native pulse AND stopping rollout must still prove it admissible.
        horizons=tuple(dict.fromkeys((local_duration,1.,.5,.25)))
        candidates=([(target,effort,position,horizon) for horizon in horizons
                     for effort in (requested,requested*.5)] if target is not None else [])
        if target is not None:
            candidates.extend((target,0.,position,horizon) for horizon in horizons)
        if self.model=='otter':
            # One bounded reverse pulse preserves the last native heading.
            # It is selected only if its real reverse-thrust/coast rollout
            # is known-free, never by kinematic backstepping/teleporting.
            retreat=(None,-requested*.5,None,.25)
            candidates.insert(0 if retreat_first else len(candidates),retreat)
        candidates.append((None,0.,None,.25))
        rejected=[]
        attempted=[]
        # A difficult first candidate must not consume every callback's
        # budget and starve the remaining admissible steering/retreat
        # choices. Only the search cursor survives; every candidate is
        # re-evaluated from this callback's full current native state.
        first_candidate=work.get('local_candidate_cursor',0)%len(candidates)
        query_started=time.monotonic()
        try:
            outcome=bounded_travel_query(PvsBackend.query_local_commands,
                (native,observed_map,candidates,first_candidate,
                 self.scene.clearance if self.scene else 0.,motion_end,
                 self.dt,self.speed_limit,self.hold_seconds,coast_delay),motion_end,worker=self.local_query_worker)
            prediction=outcome['prediction'];selected=outcome['selected']
            selected_effort=outcome['selected_effort'];selected_leg=outcome['selected_leg']
            has_command=outcome['has_command'];selected_index=outcome['selected_index']
            next_candidate=outcome['next_candidate'];attempted=outcome['attempted'];rejected=outcome['rejected']
            prefix_wall_s=outcome['prefix_wall_s'];prefix_phase=outcome['prefix_phase']
        except Exception as error:
            prediction=dict(status='UNKNOWN',reason='LOCAL_QUERY_WORKER:'+str(error),query_phase='ISOLATED_WORKER')
            selected_index=None;next_candidate=(first_candidate+1)%len(candidates)
            prefix_wall_s=0.;prefix_phase='ISOLATED_WORKER'
        with self.lock:
            if self.work is not work or self.generation!=generation or work['segment']!=index or work['cause'] or work['coast']:return
            # Steering/coast is admissible but is not forward progress.
            # A target-unavailable fallback has a different short list;
            # it must not fold/reset the directional search cursor.
            if target is not None:
                work['local_candidate_cursor']=0 if has_command and selected_effort>0. else next_candidate
            work.update(local_prediction_status=prediction['status'],local_prediction_reason=prediction['reason'],
                local_query_wall_s=time.monotonic()-query_started,
                local_rejected_commands=';'.join(rejected),local_query_budget_wall_s=self.local_query_budget_s,
                local_map_snapshot_wall_s=snapshot_wall_s,
                local_coast_prefix_wall_s=prefix_wall_s,
                local_coast_prefix_phase=prefix_phase,
                local_query_phase=prediction.get('query_phase',''),
                local_query_execution='EXISTING_ISOLATED_BOUNDED_WORKER',
                local_candidate_wall_s=prediction.get('query_wall_s',0.),
                local_candidate_model_elapsed_s=prediction.get('query_model_elapsed_s',0.),
                local_attempted_candidates=';'.join(attempted),
                local_query_coast_prefix_s=coast_delay,
                local_query_model_elapsed_s=self.backend.time_s-native.time_s)
            activation=prediction.get('activation_model_time_s',native.time_s+coast_delay)
            activation_step=prediction.get('activation_step',native.steps+int(math.ceil(coast_delay/self.dt-1e-9)))
            if self.backend.steps>=activation_step:
                work['local_motion_status']='STALE_NATIVE_QUERY'
                return
            work.update(local_plan_seq=work.get('local_plan_seq',0)+1,
                local_target=selected,local_leg_start=selected_leg,local_plan_stamp=stamp,
                local_has_command=False,local_pending_command=has_command,
                local_activation=activation,local_activation_step=activation_step,
                local_command_end_step=prediction.get('command_end_step',-1) if has_command else -1,
                local_activation_digest=prediction.get('activation_state_digest',''),
                local_command_until=activation+prediction.get('command_duration_s',.25) if has_command else -math.inf,local_effort=selected_effort,
                local_motion_status=prediction['status']+':'+prediction['reason'],
                local_rejected_commands=';'.join(rejected),
                local_geometric_target=target,
                local_stop_s=prediction.get('predicted_stop_s',0.),
                local_motion_trace=prediction.get('local_motion_trace',()),
                local_wait=not has_command or (selected is None and selected_effort==0.),
                local_survey_done=remaining is not None and not remaining)

    def _end_local_command(self,work,reason='QUERY_COAST'):
        """Exactly the once-only heading-input handoff in the coast forecast."""
        if work.get('local_has_command',False):
            work.update(local_actual_end_step=self.backend.steps,
                local_actual_end_model_s=self.backend.time_s,local_actual_end_reason=reason)
        if work.pop('local_freeze_on_coast',False) and self.model=='otter':
            self.backend.freeze_heading_reference()
        work['local_has_command']=False

    def _mapping_target(self,work,position):
        """Apply one observed-map target without resetting the native plant.

        Missing/stale observations command native coast/trim. They do not
        instantaneously stop the physical vessel or complete the Goal.
        """
        goal=work['paths'][work['segment']][-1]
        tolerance=(work['support_radius_m'] if work.get('support_radius_m') else
            .6*self.return_site['radius_m'] if self.return_site and
            tuple(goal)==tuple(self.return_site['position']) else NATIVE_START_TOLERANCE_M)
        done=(math.dist(position,goal)<=tolerance and
              (not work.get('region_id') or work.get('local_survey_done',False)))
        if done and not work.get('local_has_command',False):
            if work['segment']+1<len(work['paths']):
                work['segment']+=1;work['point']=1
                work['segment_started']=self.backend.time_s
                work['local_plan_stamp']=-math.inf
            else:work['coast']=True
            self._end_local_command(work,'TARGET_REACHED')
            return None
        if work.get('local_pending_command',False):
            activation=work['local_activation']
            if self.backend.steps<work['local_activation_step']:
                work['local_has_command']=False
                work['local_wait']=True
                return None
            input_age=rospy.Time.now().to_sec()-self.survey_stamp
            input_limit=(2. if getattr(self.observation_request,'execution_mode','')=='INSPECTION_CONTROL' else 1.)
            if input_age>input_limit:
                # An unstarted command never changes the heading input. Keep
                # the already used coast prefix, and await actual fresh input.
                work.update(local_pending_command=False,local_has_command=False,
                    local_command_end_step=-1,local_command_until=-math.inf,
                    local_motion_status='ACTIVATION_INPUT_STALE',local_wait=True)
                return None
            expected=work['local_activation_digest']
            actual_digest=hashlib.sha256(self.backend.execution_state_bytes()).hexdigest()
            matched=(self.backend.steps==work['local_activation_step'] and
                abs(self.backend.time_s-activation)<self.dt*.5 and
                actual_digest==expected)
            work['local_pending_command']=False
            if not matched:
                work.update(local_has_command=False,local_command_until=-math.inf,local_command_end_step=-1,
                            local_motion_status='ACTIVATION_STATE_CHANGED',local_wait=True)
                return None
            from qn_aav_simulator.observation_coverage import peer_segment_clear
            trace=work.get('local_motion_trace',())
            try:
                peers=self._peer_snapshot(position,rospy.Time.now().to_sec(),trace[-1][0] if trace else .25)
                previous_position=position;previous_time=0.
                for elapsed,point in trace:
                    if not peer_segment_clear(previous_position,point,self.backend.collision_radius_m,
                            peers,previous_time,elapsed):raise ValueError('PEER_ACTIVATION_CONFLICT')
                    previous_position=point;previous_time=elapsed
            except ValueError as error:
                work.update(local_has_command=False,local_wait=True,local_motion_status=str(error))
                return None
            work['local_has_command']=True
            work['local_freeze_on_coast']=True
            work['local_wait']=work.get('local_target') is None and work.get('local_effort',0.)==0.
            work.update(local_actual_activation_step=self.backend.steps,
                local_actual_activation_model_s=self.backend.time_s,
                local_actual_activation_digest=actual_digest,
                local_actual_activation_input_stamp_s=self.survey_stamp)
        # The future coast prefix is computation, not missing sensor input.
        # Actual scans continue updating this map while a committed native
        # window waits/executes. Assess that real input timestamp, rather than
        # expiring every future command from its older query-start timestamp.
        stale_input=(rospy.Time.now().to_sec()-self.survey_stamp>
                (2. if getattr(self.observation_request,'execution_mode','')=='INSPECTION_CONTROL' else 1.))
        now=rospy.Time.now().to_sec()
        if work.get('local_has_command') and now-work.get('local_peer_checked_s',-math.inf)>=.1:
            from qn_aav_simulator.observation_coverage import peer_segment_clear
            trace=work.get('local_motion_trace',())
            elapsed=self.backend.time_s-work.get('local_actual_activation_model_s',self.backend.time_s)
            remaining=tuple((when-elapsed,point) for when,point in trace if when>elapsed)
            try:
                peers=self._peer_snapshot(position,now,remaining[-1][0] if remaining else .25)
                previous_position=position;previous_time=0.
                for when,point in remaining:
                    if not peer_segment_clear(previous_position,point,self.backend.collision_radius_m,
                            peers,previous_time,when):raise ValueError('PEER_COMMAND_CONFLICT')
                    previous_position=point;previous_time=when
            except ValueError as error:
                self._end_local_command(work,'PEER_COMMAND_RECHECK')
                work.update(local_wait=True,local_motion_status=str(error),local_peer_status=str(error))
                return None
            work['local_peer_checked_s']=now
        if stale_input or self.backend.steps>=work.get('local_command_end_step',-1):
            work['local_wait']=True
            self._end_local_command(work,'ACTUAL_SCAN_STALE' if stale_input else 'PULSE_WINDOW_COMPLETE')
            return None
        return work.get('local_target')

    def state_digest(self,_request):
        with self.lock:
            report=dict(agent_id=self.agent_id,model_time_s=self.backend.time_s,
                ros_stamp_s=rospy.Time.now().to_sec(),
                digest=hashlib.sha256(self.backend.execution_state_bytes()).hexdigest(),
                active_goal_id=self.work['id'] if self.work else '',resource_locked=self.locked)
            if self.model=='otter':report['native_state']=self.backend.numeric_state_claim()
        return TriggerResponse(True,json.dumps(report,allow_nan=False))

    def goal(self,handle):
        goal=handle.get_goal()
        ident=handle.get_goal_id().id
        with self.lock:
            try:
                if self.work is not None or self.pending is not None or self.locked or ident in self.retired:
                    raise ValueError('MEMBER_BUSY_OR_LOCKED')
                self._refresh_observation_request()
                if not ident:raise ValueError('INVALID_GOAL_ID')
                if self.generation>=2147483647:raise ValueError('GENERATION_EXHAUSTED')
                if goal.terminal_behavior!=self.terminal_behavior:
                    raise ValueError('terminal behavior differs from the configured native trim/coast experiment')
                paths=[];efforts=[];durations=[]
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
                    duration=segment.duration.to_sec()
                    if not math.isfinite(duration) or duration<0:
                        raise ValueError('invalid native segment duration')
                    stationary=len(points)==2 and points[0]==points[1]
                    if stationary and duration and (self.model!='otter' or
                            self.terminal_behavior!='TRIM_PROPULSION'):
                        raise ValueError('bounded intermediate wait requires stationary Otter trim')
                    if stationary and len(goal.segments)>1 and duration==0:
                        raise ValueError('intermediate stationary path requires finite duration')
                    if not stationary and any(math.dist(a,b)==0 for a,b in zip(points,points[1:])):
                        raise ValueError('zero-length path leg')
                    if self.scene and not self.online_mapping:
                        reason=self.scene.path_violation(points,self.backend.collision_radius_m)
                        if reason:raise ValueError(reason)
                    paths.append(points)
                    durations.append(duration)
                    selected=float(getattr(segment,'propulsion_effort',0.) or self.effort)
                    if not math.isfinite(selected) or selected<=0 or (self.model=='remus100' and
                            selected>self.backend.vehicle.nMax):
                        raise ValueError('invalid native segment propulsion effort')
                    efforts.append(selected)
                if not 1<=len(paths)<=16:
                    raise ValueError('fragment requires 1..16 segments')
                region_id=str(getattr(goal,'region_id',''))
                if region_id and not self.online_mapping:raise ValueError('REGION_REQUIRES_ONLINE_MAPPING')
                if self.online_mapping:
                    actual=tuple(self.backend.snapshot()['position'])
                    paths=[(actual,paths[-1][-1])]
                    efforts=efforts[:1];durations=[sum(durations)]
                reason=self._entry_reason(paths)
                if reason:raise ValueError(reason)
                timeout=goal.execution_timeout.to_sec()
                if not math.isfinite(timeout) or timeout<=0:
                    raise ValueError('finite positive timeout required')
                wait=getattr(goal,'terminal_wait',None)
                terminal_wait_s=wait.to_sec() if wait is not None else 0.
                if not math.isfinite(terminal_wait_s) or not 0<=terminal_wait_s<timeout:
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
            # The mission assigns a path and lets this endpoint execute it from
            # its current admissible state. A full future plant rollout belongs
            # to an explicit diagnostic, not every OFFSHORE_JOINT acceptance.
            task_level=(getattr(self.observation_request,'template_id','') in ('OFFSHORE_JOINT','WIND_INSPECTION','PLATFORM_PIPELINE_INSPECTION'))
            if self.native_preflight and not task_level and not self.online_mapping:
                query_deadline=time.monotonic()+self.planning_budget_s
                backend_snapshot=copy.deepcopy(self.backend)
                token=dict(handle=handle,id=ident,task=goal.task_id,paths=paths,efforts=efforts,
                    durations=durations,timeout=timeout,
                    generation=self.generation,backend=backend_snapshot,
                    state_signature=backend_snapshot.execution_state_bytes(),
                    deadline=query_deadline,prepare_only=bool(getattr(goal,'prepare_only',False)),observations=observations,
                    terminal_wait_s=terminal_wait_s)
                self.pending=token
                # ActionServer invokes callbacks while holding its lock. The
                # query must run outside BOTH locks so cancel and integration
                # remain live. A token prevents a late query committing a new goal.
                threading.Thread(target=self._preflight,args=(token,),daemon=True).start()
            else:
                self.last_prediction=dict(goal_id=ident,status='TASK_LEVEL_ADMITTED',
                    reason='LOCAL_STATE_AND_PATH_CHECKED; FUTURE_DYNAMICS_NOT_PREDICTED')
                self._accept(handle,ident,goal.task_id,paths,efforts,durations,timeout,
                    bool(getattr(goal,'prepare_only',False)),observations,terminal_wait_s,region_id)

    def _refresh_observation_request(self):
        path=str(rospy.get_param('/mission/request_file',''))
        if path==self.observation_request_file:return
        from qn_aav_simulator.task_line import load_request
        try:request=load_request(path)
        except (OSError,TypeError,ValueError) as error:
            raise ValueError('cannot load next monitoring request: '+str(error))
        if request.request_id in self.observation_request_ids:
            raise ValueError('next monitoring request must have a new request_id')
        if getattr(request,'execution_mode','') not in ('ONLINE_MAPPING','INSPECTION_CONTROL') or not self.online_mapping:
            raise ValueError('live request changes require the existing ONLINE_MAPPING mode')
        self.observation_epoch_pending=rospy.Time.now().to_sec()
        self.observation_request=request
        self.observation_request_file=path
        self.observation_request_ids.add(request.request_id)

    def _accept(self,handle,ident,task,paths,efforts,durations,timeout,prepare_only=False,observations=None,terminal_wait_s=0.,region_id=""):
        self.generation+=1
        self.work=dict(handle=handle,id=ident,task=task,paths=paths,efforts=efforts,durations=durations,
                       segment=0,point=1,segment_started=self.backend.time_s,
                       coast=False,settled=None,cause='',model_start=self.backend.time_s,waiting_commit=prepare_only,
                       deadline=time.monotonic()+timeout,observations=observations,ros_start=rospy.Time.now().to_sec(),
                       terminal_wait_s=terminal_wait_s,return_left=False,return_reentered=False,
                       admission='LOCAL_STATE_AND_PATH',region_id=region_id,local_wait=True,local_target=None)
        if (getattr(self.observation_request,'execution_mode','')=='INSPECTION_CONTROL' or
                getattr(self.observation_request,'execution_mode','')=='ONLINE_MAPPING' and
                rospy.get_param('/scene/communication/model','')=='FINITE_STAGE_SERVICE'):
            site=next((s for s in rospy.get_param('/scene/communication_sites',())
                       if math.dist(paths[-1][-1],s['position'])<1e-6),None)
            if site is not None:self.work['support_radius_m']=float(site['radius_m'])
        if self.last_prediction.get('goal_id')==ident:
            self.last_prediction['accepted_model_time_s']=self.backend.time_s
        handle.set_accepted('finite native path fragment accepted')

    def _entry_reason(self,paths):
        """Check present execution conditions without integrating a future copy."""
        state=self.backend.snapshot()
        if self.domain_failure:return 'PERSISTENT_NATIVE_DOMAIN_FAILURE'
        if self.scene_failure:return 'PERSISTENT_SCENE_SAFETY_FAILURE'
        if state['model']!=self.model:return 'NATIVE_MODEL_MISMATCH'
        if any(not all(math.isfinite(v) for v in state[key]) for key in
               ('position','world_velocity','body_angular_velocity','quaternion_wxyz','actuators')):
            return 'NONFINITE_ACTUAL_NATIVE_STATE'
        if state['actual_mode']!=self.mode:return 'ACTUAL_STATE_OUTSIDE_NATIVE_DOMAIN'
        if not self.online_mapping and math.dist(paths[0][0],state['position'])>NATIVE_START_TOLERANCE_M:
            return 'START_STATE_CHANGED'
        if math.sqrt(sum(v*v for v in state['world_velocity']))>self.speed_limit:
            return 'NATIVE_ENTRY_NOT_SETTLED'
        if self.scene:
            reason=self.scene.violation(state['position'],self.backend.collision_radius_m)
            if reason:return reason
            for path in (() if self.online_mapping else paths):
                reason=self.scene.path_violation(path,self.backend.collision_radius_m)
                if reason:return reason
        return ''

    def _preflight(self,token):
        try:
            prediction=bounded_travel_query(ExecutorTravelTimeProvider.query_native_fragment,
                (token['backend'],token['paths'],token['efforts'],self.scene,token['deadline'],
                 self.dt,self.speed_limit,self.hold_seconds+token['terminal_wait_s'],token['timeout'],
                 True,0.,None,token['durations']),token['deadline'])
        except Exception as exc:
            prediction=dict(status='UNKNOWN',reason=str(exc))
        # Same order as actionlib goal/cancel callbacks; never model->actionlib.
        with self.server.lock,self.lock:
            if self.pending is not token:return
            self.pending=None
            self.last_prediction={k:v for k,v in prediction.items() if k not in ('trajectory','terminal_backend','source_fingerprint','settled_model_time_s','terminal_wait_s')}
            self.last_prediction['goal_id']=token['id']
            self.last_prediction['qualified_entry_state_digest']=hashlib.sha256(token['state_signature']).hexdigest()
            if prediction.get('status')=='FEASIBLE' and prediction.get('terminal_backend') is not None:
                self.last_prediction['predicted_terminal_state_digest']=hashlib.sha256(
                    prediction['terminal_backend'].execution_state_bytes()).hexdigest()
            state=self.backend.snapshot()
            reason=''
            if self.locked or self.generation!=token['generation']:
                reason='STATE_OR_COMMITMENT_CHANGED'
            elif self.backend.execution_state_bytes()!=token['state_signature']:
                reason='NATIVE_STATE_CHANGED_DURING_QUERY'
            elif (state['actual_mode']!=self.mode or
                  math.dist(state['position'],token['paths'][0][0])>NATIVE_START_TOLERANCE_M):
                reason='STATE_CHANGED_DURING_QUERY'
            elif prediction['status']!='FEASIBLE':
                reason='NATIVE_PREDICTION_'+prediction['status']+': '+prediction['reason']
            elif self._needs_return_entry(token['paths'],token['observations']):
                outside=False;reentered=False
                for _,position in prediction['trajectory']:
                    if math.dist(position,self.return_site['position'])>self.return_site['radius_m']:
                        outside=True
                    elif outside:reentered=True
                if not reentered:reason='NATIVE_RETURN_SITE_NOT_REENTERED'
            elif (self._returns_to_declared_site(token['paths']) and
                  math.dist(prediction['terminal_position'],self.return_site['position'])>
                  self.return_site['radius_m']):
                reason='NATIVE_RETURN_SITE_NOT_REACHED'
            if reason:
                token['handle'].set_rejected(PlatformTaskResult(task_id=token['task'],goal_id=token['id'],
                    actual_mode=state['actual_mode'],reason=reason,resource_locked=self.locked,
                    model_time_s=self.backend.time_s))
            else:
                self._accept(token['handle'],token['id'],token['task'],token['paths'],token['efforts'],
                    token['durations'],token['timeout'],token['prepare_only'],token['observations'],token['terminal_wait_s'])
                self.work['qualified_state_signature']=token['state_signature']

    def _returns_to_declared_site(self,paths):
        return (self.return_site is not None and
                tuple(paths[-1][-1])==tuple(self.return_site['position']))

    def _needs_return_entry(self,paths,observations):
        request=getattr(self,'observation_request',None)
        return (self.backend.model=='remus100' and self.return_site is not None and
                (self._returns_to_declared_site(paths) or
                 (request is not None and request.return_required and observations is not None)))

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
            if work.get('admission')!='LOCAL_STATE_AND_PATH':
                return StartPreparedActionResponse(False,'LOCAL_PREPARATION_NOT_ADMITTED')
            # The live plant keeps integrating while reserved. Recheck actual
            # entry conditions, not byte identity of a previous simulation.
            reason=self._entry_reason(work['paths'])
            if reason:return StartPreparedActionResponse(False,reason)
            work['waiting_commit']=False
            work['model_start']=self.backend.time_s
            work['segment_started']=self.backend.time_s
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
                replacement=(self.online_mapping and self.work['id'] in
                    rospy.get_param('/mission/replacement_goal_ids',[]) and not self.locked and
                    not self.scene_failure and not self.domain_failure)
                if self.online_mapping:self._end_local_command(self.work,'CANCEL_REQUEST')
                self.work.update(cause='CANCEL_REQUEST',coast=True,settled=None,waiting_commit=False,
                    controlled_replacement=replacement)
                if not replacement:self.locked=True

    def finish(self,verified,reason):
        w=self.work
        normal=verified and not w['cause']
        replacement_verified=(verified and w['cause']=='CANCEL_REQUEST' and
            w.get('controlled_replacement',False) and not self.locked and
            not self.scene_failure and not self.domain_failure)
        if replacement_verified:reason='REPLACED_SAFE_HOLD'
        observations=w.get('observations')
        observation_missing=normal and observations is not None and observations.emitted!=set(observations.points)
        if observation_missing:normal=False;reason='OBSERVATION_NOT_SATISFIED'
        needs_return_entry=self._needs_return_entry(w['paths'],observations)
        if verified and not w['cause'] and needs_return_entry and not w.get('return_reentered',False):
            normal=False;reason='RETURN_SITE_NOT_REENTERED';self.locked=True
        elif (verified and not w['cause'] and not needs_return_entry and self._returns_to_declared_site(w['paths']) and
                math.dist(self.backend.snapshot()['position'],self.return_site['position'])>
                self.return_site['radius_m']):
            normal=False;reason='RETURN_SITE_NOT_REACHED';self.locked=True
        elif (verified and not w['cause'] and w.get('support_radius_m') and
                math.dist(self.backend.snapshot()['position'],w['paths'][-1][-1])>w['support_radius_m']):
            normal=False;reason='SUPPORT_SITE_NOT_REACHED';self.locked=True
        self.locked=self.locked or not verified or (bool(w['cause']) and not replacement_verified)
        if verified and observations is not None and not w['cause']:
            self.products.publish(String(data=json.dumps(observations.terminal_report(rospy.Time.now().to_sec()),allow_nan=False)))
        self.retired.add(w['id'])
        if self.last_prediction.get('goal_id')==w['id']:
            self.last_prediction['actual_duration_s']=self.backend.time_s-w['model_start']
            self.last_prediction['actual_terminal_position']=self.backend.snapshot()['position']
            try:self.last_prediction['actual_terminal_state_digest']=hashlib.sha256(
                self.backend.execution_state_bytes()).hexdigest()
            except (TypeError,ValueError,OverflowError):pass
        result=PlatformTaskResult(task_id=w['task'],goal_id=w['id'],task_completed=normal,
            terminal_verified=verified,actual_mode=self.backend.snapshot()['actual_mode'],reason=reason,
            resource_locked=self.locked,model_time_s=self.backend.time_s)
        self.work=None
        if normal:w['handle'].set_succeeded(result)
        elif w['cause']=='CANCEL_REQUEST' and verified:w['handle'].set_canceled(result)
        else:w['handle'].set_aborted(result)
        if self.observation_request is not None:
            from qn_aav_simulator.observation_coverage import action_terminal_event
            terminal='SUCCEEDED' if normal else 'CANCELED' if w['cause']=='CANCEL_REQUEST' and verified else 'ABORTED'
            notice=action_terminal_event(
                self.observation_request,w['id'],self.agent_id,rospy.Time.now().to_sec(),
                terminal,result.task_completed,result.terminal_verified,result.resource_locked,
                result.reason)
            try:notice['terminal_state_digest']=hashlib.sha256(self.backend.execution_state_bytes()).hexdigest()
            except (TypeError,ValueError,OverflowError):pass  # preserve Result; no reusable state claim
            self.products.publish(String(data=json.dumps(notice,allow_nan=False)))

    def step(self):
        with self.server.lock,self.lock:
            w=self.work
            target=None
            effort=0.
            waiting=False
            terminal_homing=False
            if self.online_mapping and w and not w['coast'] and not w['waiting_commit']:
                target=self._mapping_target(w,self.backend.snapshot()['position'])
                waiting=w.get('local_wait',False)
                effort=w.get('local_effort',0.) if w.get('local_has_command',False) and not w['coast'] else 0.
                # Exact forecast backup: zero added propulsion and no new
                # heading reference. A far business goal is never a fallback
                # motion command when the observed-space route is unavailable.
            elif w and not w['coast'] and not w['waiting_commit']:
                segment=w['segment'];path=w['paths'][segment]
                waiting=(path[0]==path[1] and w['durations'][segment]>0)
                if waiting and self.backend.time_s-w['segment_started']+1e-9>=w['durations'][segment]:
                    segment+=1;w['segment']=segment;w['point']=1;w['segment_started']=self.backend.time_s
                    w['coast']=segment==len(w['paths']);waiting=False
                if not waiting and not w['coast']:
                    position=self.backend.snapshot()['position']
                    terminal_return=(self.model=='otter' and
                        getattr(self.observation_request,'template_id','') in ('OFFSHORE_JOINT','WIND_INSPECTION','PLATFORM_PIPELINE_INSPECTION') and
                        self._returns_to_declared_site(w['paths']) and
                        w['segment']==len(w['paths'])-1 and
                        w['point']==len(w['paths'][-1])-1)
                    distance=(math.dist(position,self.return_site['position'])
                              if terminal_return else math.inf)
                    past_endpoint=False
                    if terminal_return:
                        start=w['paths'][-1][-2];end=w['paths'][-1][-1]
                        direction=tuple(end[i]-start[i] for i in range(3))
                        past_endpoint=sum((position[i]-start[i])*direction[i] for i in range(3))>=sum(
                            value*value for value in direction)
                    if terminal_return and (distance<=2*self.return_site['radius_m'] or past_endpoint):
                        if distance<=.6*self.return_site['radius_m']:
                            w['coast']=True
                        elif (self.scene and self.scene.path_violation(
                                (position,self.return_site['position']),self.backend.collision_radius_m)):
                            w.update(cause='SCENE_SAFETY_VIOLATION',coast=True)
                            self.locked=True
                        else:
                            target=self.return_site['position']
                            terminal_homing=True
                    else:
                        previous=w['segment']
                        w['segment'],w['point'],target,w['coast']=advance_path_target(
                            w['paths'],w['segment'],w['point'],position)
                        if w['segment']!=previous:w['segment_started']=self.backend.time_s
                    segment=w['segment'];path=w['paths'][segment]
                    waiting=(path[0]==path[1] and w['durations'][segment]>0)
                effort=0. if w['coast'] or waiting else w['efforts'][w['segment']]
                if w['coast'] or waiting:target=None
            leg_start=(None if self.online_mapping and waiting else
                       w.get('local_leg_start') if self.online_mapping and w and target is not None else
                       w['paths'][w['segment']][w['point']-1]
                       if w and target is not None and not terminal_homing else None)
            state=self.backend.step(self.dt,target,effort,leg_start,
                allow_reverse=self.online_mapping and self.model=='otter' and effort<0.)
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
                from qn_aav_simulator.time_alignment import DEFAULT_MAX_ABS_DRIFT_S
                now=rospy.Time.now().to_sec()
                self.action_model_clock_drift_s=abs((self.backend.time_s-w['model_start'])-(now-w['ros_start']))
                time_valid=self.action_model_clock_drift_s<=DEFAULT_MAX_ABS_DRIFT_S
                self.visual_timing_relaxed=self.visual_timing_relaxed or (
                    self.allow_visual_timing_relaxation and not time_valid)
                observations=w.get('observations')
                if observations is not None and not w.get("region_id"):
                    valid=(not w['cause'] and not w['waiting_commit'] and not self.locked and
                           (time_valid or self.allow_visual_timing_relaxation))
                    for event in observations.sample(self.backend.time_s,state['position'],state['actual_mode'],now,valid):
                        self.products.publish(String(data=json.dumps(event,allow_nan=False)))
                if (self._needs_return_entry(w['paths'],observations) and
                        not w['cause'] and not w['waiting_commit']):
                    if math.dist(state['position'],self.return_site['position'])>self.return_site['radius_m']:
                        w['return_left']=True
                    elif (w['return_left'] and
                          (observations is None or observations.emitted==set(observations.points))):
                        w['return_reentered']=True
                speed=math.sqrt(sum(v*v for v in state['world_velocity']))
                support_ok=(not w.get('support_radius_m') or
                    math.dist(state['position'],w['paths'][-1][-1])<=w['support_radius_m'])
                if w['coast'] and not w['cause'] and not support_ok:
                    # A service is a declared region. A coast leaving it cannot
                    # complete that service; resume the same observed guidance.
                    w['coast']=False
                if (w['coast'] and speed<=self.speed_limit and
                        (support_ok or bool(w['cause'])) and
                        (time_valid or self.allow_visual_timing_relaxation)):
                    if w['settled'] is None:w['settled']=self.backend.time_s
                else:w['settled']=None
                extra_wait=w.get('terminal_wait_s',0.) if not w['cause'] else 0.
                if w['settled'] is not None and self.backend.time_s-w['settled']>=self.hold_seconds+extra_wait:
                    self.finish(True,w['cause'] or self.terminal_behavior+'_VERIFIED_IN_QUALIFICATION')
                elif time.monotonic()>=w['deadline']:
                    self.finish(False,'OBSERVATION_TIMEOUT_UNVERIFIED')
                elif self.backend.steps%10==0:
                    w['handle'].publish_feedback(PlatformTaskFeedback(segment_index=w['segment'],
                    operation='PREPARED' if w['waiting_commit'] else
                              'WAIT_LOCAL_OBSERVATION' if self.online_mapping and waiting else
                              'PRECOMMITTED_WAIT' if waiting else
                              self.terminal_behavior if w['coast'] else self.mode+'_PATH',
                        reference_source='PVS_NATIVE',actual_mode=self.mode,
                        reference_generation=self.generation,model_time_s=self.backend.time_s))
            stamp=rospy.Time.now()
            if self.online_mapping:self.survey_poses.append((stamp.to_sec(),tuple(state['position'])))
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
                    'visual_timing_relaxed':self.visual_timing_relaxed,
                    'action_model_clock_drift_s':self.action_model_clock_drift_s,
                    'local_plan_seq':self.work.get('local_plan_seq',0) if self.work else 0,
                    'local_plan_stamp':self.work.get('local_plan_stamp',0.) if self.work else 0.,
                    'survey_stamp':self.survey_stamp,
                    'survey_age_ros_s':stamp.to_sec()-self.survey_stamp,
                    'local_target':json.dumps(self.work.get('local_target')) if self.work else 'null',
                    'local_wait':self.work.get('local_wait',False) if self.work else False,
                    'local_motion_status':self.work.get('local_motion_status','WAITING_FOR_SCAN') if self.work else '',
                    'local_prediction_status':self.work.get('local_prediction_status','') if self.work else '',
                    'local_prediction_reason':self.work.get('local_prediction_reason','') if self.work else '',
                    'local_candidate_cursor':self.work.get('local_candidate_cursor',0) if self.work else 0,
                    'local_attempted_candidates':self.work.get('local_attempted_candidates','') if self.work else '',
                    'local_query_budget_wall_s':self.work.get('local_query_budget_wall_s',0.) if self.work else 0.,
                    'local_query_wall_s':self.work.get('local_query_wall_s',0.) if self.work else 0.,
                    'local_query_coast_prefix_s':self.work.get('local_query_coast_prefix_s',0.) if self.work else 0.,
                    'local_query_model_elapsed_s':self.work.get('local_query_model_elapsed_s',0.) if self.work else 0.,
                    'local_map_snapshot_wall_s':self.work.get('local_map_snapshot_wall_s',0.) if self.work else 0.,
                    'local_coast_prefix_wall_s':self.work.get('local_coast_prefix_wall_s',0.) if self.work else 0.,
                    'local_coast_prefix_phase':self.work.get('local_coast_prefix_phase','') if self.work else '',
                    'local_query_phase':self.work.get('local_query_phase','') if self.work else '',
                    'local_peer_status':self.work.get('local_peer_status','WAITING_FOR_PEERS') if self.work else 'IDLE',
                    'local_query_execution':self.work.get('local_query_execution','') if self.work else '',
                    'local_candidate_wall_s':self.work.get('local_candidate_wall_s',0.) if self.work else 0.,
                    'local_candidate_model_elapsed_s':self.work.get('local_candidate_model_elapsed_s',0.) if self.work else 0.,
                    'local_rejected_commands':self.work.get('local_rejected_commands','') if self.work else '',
                    'local_geometric_target':json.dumps(self.work.get('local_geometric_target')) if self.work else '',
                    'local_effort':self.work.get('local_effort',0.) if self.work else 0.,
                    'local_has_command':self.work.get('local_has_command',False) if self.work else False,
                    'local_command_until':self.work.get('local_command_until',0.) if self.work else 0.,
                    'local_activation_step':self.work.get('local_activation_step',-1) if self.work else -1,
                    'local_activation_digest':self.work.get('local_activation_digest','') if self.work else '',
                    'local_actual_activation_step':self.work.get('local_actual_activation_step',-1) if self.work else -1,
                    'local_actual_activation_model_s':self.work.get('local_actual_activation_model_s',0.) if self.work else 0.,
                    'local_actual_activation_digest':self.work.get('local_actual_activation_digest','') if self.work else '',
                    'local_actual_activation_input_stamp_s':self.work.get('local_actual_activation_input_stamp_s',0.) if self.work else 0.,
                    'local_actual_end_step':self.work.get('local_actual_end_step',-1) if self.work else -1,
                    'local_actual_end_model_s':self.work.get('local_actual_end_model_s',0.) if self.work else 0.,
                    'local_actual_end_reason':self.work.get('local_actual_end_reason','') if self.work else '',
                    'local_command_end_step':self.work.get('local_command_end_step',-1) if self.work else -1,
                    'local_stop_s':self.work.get('local_stop_s',0.) if self.work else 0.,
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
        simulated=bool(rospy.get_param('/use_sim_time',False))
        speed=float(rospy.get_param('/mission/simulation_speed',1.))
        if simulated and (not math.isfinite(speed) or not 1.<=speed<=16.):
            raise ValueError('configured shared scene clock speed required')
        while simulated and not rospy.is_shutdown() and rospy.Time.now().to_sec()==0.:time.sleep(.005)
        next_tick=time.monotonic()
        next_sim_tick_ns=rospy.Time.now().to_nsec()
        model_step_ns=round(self.dt*1e9)
        while not rospy.is_shutdown():
            self.step()
            if simulated:
                next_sim_tick_ns+=model_step_ns
                while not rospy.is_shutdown() and rospy.Time.now().to_nsec()<next_sim_tick_ns:
                    time.sleep(min(.005,max(.0001,(next_sim_tick_ns-rospy.Time.now().to_nsec())/1e9/speed)))
                continue
            next_tick+=self.dt
            delay=next_tick-time.monotonic()
            if delay>0:time.sleep(delay)


if __name__=='__main__':
    rospy.init_node('pvs_platform')
    PvsNode().run()
