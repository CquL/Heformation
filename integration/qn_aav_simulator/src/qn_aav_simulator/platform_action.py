"""qn-local finite Action worker, advanced by completed plant steps.

Opt-in qualification endpoint. It shares the existing qn node lock and never
creates/resets a second plant. Production qualification is explicit configuration.
"""
import math
import time
import actionlib
import rospy
from qn_aav_simulator.msg import PlatformTaskAction, PlatformTaskFeedback, PlatformTaskResult
from qn_aav_simulator.srv import TakeReference, TakeReferenceResponse
from qn_aav_simulator.platform_execution import ReferenceOwnership, Segment, actual_mode, validate_fragment


class LocalPlatformAction:
    def __init__(self,node):
        self.node=node
        self.owner=ReferenceOwnership()
        self.work=None
        self.hold_point=node.state.position
        self.last_air_id=-1
        self.air_floor=-1
        self.qualification_only=bool(rospy.get_param('~qualification_only',True))
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
        self.server.start()

    def mode(self):
        return actual_mode(self.node.state.medium_flag)

    def claim(self,request):
        with self.node.lock:
            if self.work is not None and request.goal_id != self.work['id']:
                return TakeReferenceResponse(False,self.owner.generation,'MEMBER_BUSY')
            if request.source=='AIR_SWARM' and self.mode()!='AIR':
                return TakeReferenceResponse(False,self.owner.generation,'ACTUAL_MODE_NOT_AIR')
            if request.source=='AIR_SWARM' and self.owner.source=='PLATFORM':
                return TakeReferenceResponse(False,self.owner.generation,'SWARM_ROUND_TRIP_NOT_QUALIFIED')
            # External clients cannot install an unvalidated platform program.
            if request.source=='PLATFORM' and self.work is None:
                return TakeReferenceResponse(False,self.owner.generation,'NO_ACCEPTED_PLATFORM_ACTION')
            before=self.owner.generation
            accepted,reason=self.owner.claim(request.goal_id,request.source,request.expected_generation)
            if accepted and before!=self.owner.generation:
                self._flush_reference()
                if request.source=='AIR_SWARM':
                    self.air_floor=self.last_air_id
            return TakeReferenceResponse(accepted,self.owner.generation,reason)

    def _flush_reference(self):
        from qn_aav_simulator.qn_telemetry import CommandAdoptionBuffer
        self.node.commands=CommandAdoptionBuffer(self.node.command_buffer_size)
        self.node.latest_command=None
        self.hold_point=self.node.state.position

    def accepts_air(self,trajectory_id):
        self.last_air_id=max(self.last_air_id,trajectory_id)
        return self.owner.source=='AIR_SWARM' and trajectory_id>self.air_floor

    def goal(self,handle):
        goal=handle.get_goal()
        ident=handle.get_goal_id().id
        with self.node.lock:
            try:
                if self.work is not None or self.owner.locked or self.owner.active:
                    raise ValueError('MEMBER_BUSY_OR_LOCKED')
                segments=[]
                for raw in goal.segments:
                    if raw.path.header.frame_id!='map':
                        raise ValueError('path frame must be map')
                    points=tuple((p.pose.position.x,p.pose.position.y,p.pose.position.z) for p in raw.path.poses)
                    segments.append(Segment(raw.operation,points,raw.duration.to_sec()))
                permitted=frozenset(('ENTER_WATER','WATER_PATH','EXIT_WATER'))
                validate_fragment(segments,self.mode(),goal.terminal_behavior,permitted)
                if math.dist(segments[0].points[0],self.node.state.position)>self.position_tolerance:
                    raise ValueError('path start does not match actual position')
                timeout=goal.execution_timeout.to_sec()
                if not math.isfinite(timeout) or timeout<=0:
                    raise ValueError('positive finite execution timeout required')
                ok,reason=self.owner.claim(ident,'PLATFORM',self.owner.generation)
                if not ok:
                    raise ValueError(reason)
            except ValueError as exc:
                handle.set_rejected(PlatformTaskResult(task_id=goal.task_id,goal_id=ident,
                    reason=str(exc),actual_mode=self.mode(),resource_locked=self.owner.locked))
                return
            self._flush_reference()
            self.work=dict(handle=handle,id=ident,task=goal.task_id,segments=segments,index=0,
                start=self.node.clock.model_time_s,settled=None,
                deadline=time.monotonic()+min(timeout,self.wall_limit),cause='',last_feedback=-1.)
            handle.set_accepted('finite local fragment accepted')

    def cancel(self,handle):
        with self.node.lock:
            if self.work is None or handle.get_goal_id().id!=self.work['id']:
                return
            if not self.work['cause']:
                self.work['cause']='CANCEL_REQUEST'
                self.work['settled']=None
                self.work['start']=self.node.clock.model_time_s
                self.hold_point=self.node.state.position
                self.owner.locked=True
                # Do not renew the first observation deadline on repeated cancel.

    def _finish(self,terminal_verified,reason):
        work=self.work
        normal=not work['cause'] and terminal_verified
        self.owner.finish(work['id'],normal)
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
        if self.owner.source!='PLATFORM':
            return None
        work=self.work
        t=self.node.clock.model_time_s
        target=self.hold_point
        if work is not None:
            segment=work['segments'][work['index']]
            elapsed=t-work['start']
            target=self.hold_point if work['cause'] else segment.reference(elapsed)
            terminal_due=bool(work['cause']) or elapsed>=segment.duration
            consistent=(self.mode()!='UNKNOWN' if work['cause'] else self.mode()==segment.target_mode)
            settled=(terminal_due and consistent and
                math.dist(self.node.state.position,target)<=self.position_tolerance and
                math.sqrt(sum(v*v for v in self.node.state.velocity))<=self.speed_tolerance)
            if settled:
                if work['settled'] is None:
                    work['settled']=t
            else:
                work['settled']=None
            if work['settled'] is not None and t-work['settled']>=self.hold_duration:
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
                    segment_index=work['index'],operation=work['segments'][work['index']].operation,
                    reference_source=self.owner.source,actual_mode=self.mode(),
                    reference_generation=self.owner.generation,model_time_s=t))
        return dict(position=target,velocity=(0.,0.,0.),acceleration=(0.,0.,0.),yaw_rad=0.,
                    stamp_s=rospy.Time.now().to_sec(),received_ros_time_s=rospy.Time.now().to_sec(),
                    trajectory_id=self.owner.generation,trajectory_flag=1)

    def diagnostics(self):
        return [('reference_source',self.owner.source),('reference_generation',str(self.owner.generation)),
                ('reference_goal_id',self.owner.goal_id),('actual_mode',self.mode()),
                ('platform_resource_locked',str(self.owner.locked)),
                ('platform_qualification_only',str(self.qualification_only))]
