"""Thin Otter/REMUS100 boundary over the vendored Fossen implementation.

All integration and actuator/control dynamics remain in PVS. The ROS scene is ENU,
body is FLU; native PVS is NED/FRD. No position is assigned after initialization.
"""
import math
import copy
import time
import numpy as np


ENU_FROM_NED=np.array([[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]])
FLU_FROM_FRD=np.diag([1.,-1.,-1.])
NATIVE_START_TOLERANCE_M=.2


def advance_path_target(paths,segment,point,position):
    """The same finite path progression for native execution and prediction."""
    points=paths[segment]
    start,target=points[point-1:point+1]
    direction=tuple(b-a for a,b in zip(start,target))
    progress=sum((position[i]-start[i])*direction[i] for i in range(3))
    if progress>=sum(v*v for v in direction):
        if point+1<len(points):point+=1
        elif segment+1<len(paths):segment,point=segment+1,1
        else:return segment,point,None,True
    return segment,point,paths[segment][point],False


def quaternion_product(a,b):
    w,x,y,z=a
    v,i,j,k=b
    return (w*v-x*i-y*j-z*k,w*i+x*v+y*k-z*j,
            w*j-x*k+y*v+z*i,w*k+x*j-y*i+z*v)


def ned_attitude_to_ros(roll,pitch,yaw):
    cr,sr=math.cos(roll/2),math.sin(roll/2)
    cp,sp=math.cos(pitch/2),math.sin(pitch/2)
    cy,sy=math.cos(yaw/2),math.sin(yaw/2)
    native=(cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,
            cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy)
    q=quaternion_product(quaternion_product((0.,2**-.5,2**-.5,0.),native),(0.,1.,0.,0.))
    norm=math.sqrt(sum(v*v for v in q))
    return tuple(v/norm for v in q)


class PvsBackend:
    def __init__(self,model,position=(0.,0.,0.),heading_rad=0.,current_speed=0.,current_heading_rad=0.,initialization_mode='NATIVE_ZERO'):
        from python_vehicle_simulator.vehicles.otter import otter
        from python_vehicle_simulator.vehicles.remus100 import remus100
        if model not in ('otter','remus100'):
            raise ValueError('only the frozen Otter/REMUS100 models are supported')
        if not all(math.isfinite(v) for v in (*position,heading_rad,current_speed,current_heading_rad)):
            raise ValueError('initial values must be finite')
        if model=='otter' and position[2]!=0:
            raise ValueError('Otter initializes at the declared sea surface')
        if model=='remus100' and position[2]>=0:
            raise ValueError('REMUS initializes submerged; it is not an amphibious model')
        self.model=model
        heading_ned=math.pi/2-heading_rad
        current_ned=math.pi/2-current_heading_rad
        if model=='otter':
            self.vehicle=otter('headingAutopilot',math.degrees(heading_ned),current_speed,math.degrees(current_ned),0.)
        else:
            self.vehicle=remus100('depthHeadingAutopilot',-position[2],math.degrees(heading_ned),0.,current_speed,math.degrees(current_ned))
        self.eta=np.array([position[1],position[0],-position[2],0.,0.,heading_ned],dtype=float)
        self.nu=self.vehicle.nu.copy()
        self.actuators=self.vehicle.u_actual.copy()
        self.vehicle.psi_d=heading_ned
        if model=='remus100':
            self.vehicle.z_d=-position[2]
        self.time_s=0.
        self.steps=0
        if model=='otter':
            height=max(self.vehicle.T,abs(float(self.vehicle.rp[2])))
            self.collision_radius_m=math.sqrt((self.vehicle.L/2)**2+(self.vehicle.B/2)**2+height**2)
        else:
            self.collision_radius_m=math.hypot(self.vehicle.L/2,self.vehicle.diam/2)
        self.trim_command=0.
        if initialization_mode not in ('NATIVE_ZERO','STATIC_TRIM'):
            raise ValueError('unknown PVS initialization mode')
        self.terminal_behavior='TRIM_PROPULSION' if initialization_mode=='STATIC_TRIM' else 'COAST_STOP'
        if initialization_mode=='STATIC_TRIM':
            if model!='otter' or current_speed!=0:
                raise ValueError('static trim is qualified only for zero-current Otter')
            self._initialize_otter_trim()

    def _initialize_otter_trim(self):
        """Solve the native payload/hydrostatic equilibrium at initialization.

        Otter uses -G eta + g_0 in its native dynamics. Nonzero pitch trim
        projects payload weight into surge; zero shaft speed does not balance
        it. This is a static pilot-input trim, not a new speed controller.
        """
        from python_vehicle_simulator.lib.gnc import Rzyx
        vehicle=self.vehicle
        axes=[2,4]
        stiffness=vehicle.G[np.ix_(axes,axes)]
        for _ in range(50):
            force=Rzyx(*self.eta[3:]).T@np.array([0.,0.,vehicle.mp*vehicle.g])
            load=np.r_[force,vehicle.S_rp@force]
            equilibrium=np.linalg.solve(stiffness,load[axes])
            error=np.max(np.abs(self.eta[axes]-equilibrium))
            self.eta[axes]=equilibrium
            if error<1e-12:break
        else:raise ValueError('native Otter static trim did not converge')
        force=Rzyx(*self.eta[3:]).T@np.array([0.,0.,vehicle.mp*vehicle.g])
        required=-force[0]
        # Native allocation uses k_pos for both signs, whereas the native
        # propeller plant uses k_neg for reverse. Account for that static map.
        self.trim_command=required*(vehicle.k_pos/vehicle.k_neg if required<0 else 1.)
        self.actuators=np.asarray(vehicle.controlAllocation(self.trim_command,0.))
        residual,_=vehicle.dynamics(self.eta,self.nu,self.actuators,self.actuators,.001)
        if np.linalg.norm(residual)>.000001:
            raise ValueError('static trim fails native zero-acceleration check')

    def step(self,dt,target=None,effort=0.,leg_start=None):
        from python_vehicle_simulator.lib.gnc import attitudeEuler
        if not math.isfinite(dt) or not 0<dt<=.05 or not math.isfinite(effort) or effort<0:
            raise ValueError('invalid native model step/control effort')
        if target is not None:
            if len(target)!=3 or not all(math.isfinite(v) for v in target):
                raise ValueError('target must be a finite ENU point')
            delta=np.array([target[1],target[0],-target[2]])-self.eta[:3]
            if self.model=='otter' and leg_start is not None:
                if len(leg_start)!=3 or not all(math.isfinite(v) for v in leg_start):
                    raise ValueError('leg start must be a finite ENU point')
                east=target[0]-leg_start[0];north=target[1]-leg_start[1]
                length=math.hypot(east,north)
                if length<=1e-9:raise ValueError('LOS guidance needs a nonzero horizontal leg')
                position=(self.eta[1],self.eta[0])
                cross=(east*(position[1]-leg_start[1])-north*(position[0]-leg_start[0]))/length
                course=math.atan2(north,east)-math.atan2(cross,self.vehicle.L)
                heading=math.degrees(math.pi/2-course)
            else:
                heading=math.degrees(math.atan2(delta[1],delta[0])) if np.linalg.norm(delta[:2])>1e-9 else math.degrees(self.eta[5])
            # Native refModel3 uses the *linear* difference r - psi_d. atan2
            # alone jumps by 2*pi at the branch cut and commands the long
            # turn to an equivalent heading. Preserve the native controller;
            # give its reference model the nearest continuous NED angle.
            heading=math.degrees(self.vehicle.psi_d+math.remainder(
                math.radians(heading)-self.vehicle.psi_d,2*math.pi))
            if self.model=='otter':
                self.vehicle.ref=heading
            else:
                if not 0 < -target[2] <= 100:
                    raise ValueError('REMUS depth outside native input range')
                self.vehicle.ref_psi=heading
                self.vehicle.ref_z=-target[2]
        if self.model=='otter':
            self.vehicle.tauX=effort+self.trim_command
            command=self.vehicle.headingAutopilot(self.eta,self.nu,dt)
        else:
            if effort>self.vehicle.nMax:
                raise ValueError('propeller rpm exceeds native model limit')
            self.vehicle.ref_n=effort
            command=self.vehicle.depthHeadingAutopilot(self.eta,self.nu,dt)
        self.nu,self.actuators=self.vehicle.dynamics(self.eta,self.nu,self.actuators,command,dt)
        self.eta=attitudeEuler(self.eta,self.nu,dt)
        if not np.all(np.isfinite(np.r_[self.eta,self.nu,self.actuators])):
            raise ArithmeticError('PVS state is not finite')
        self.steps+=1
        self.time_s+=dt
        return self.snapshot()

    def predict_native_fragment(self,paths,effort,scene,deadline,dt=.01,
                                terminal_speed=.03,hold_duration=4.,max_model_time=180.,include_state=False,terminal_wait_s=0.,resume=None,
                                segment_durations=None):
        """Read-only rollout, including native actuator lag and terminal coast.

        Copy all controller/actuator state; never reset or step the live model.
        Results describe sampled feasibility of this snapshot, not a robust
        safety certificate. The caller owns the ONE monotonic query deadline.
        """
        if not math.isfinite(deadline) or any(not math.isfinite(v) or v<=0 for v in
                (dt,terminal_speed,hold_duration,max_model_time)):
            raise ValueError('finite query budget and terminal conditions required')
        if not math.isfinite(terminal_wait_s) or not 0<=terminal_wait_s<max_model_time:
            raise ValueError('terminal wait must fit the finite model horizon')
        paths=tuple(tuple(tuple(p) for p in path) for path in paths)
        if not paths or any(len(path)<2 for path in paths):raise ValueError('finite path fragment required')
        durations=(tuple(segment_durations) if segment_durations is not None else (0.,)*len(paths))
        if len(durations)!=len(paths) or any(not math.isfinite(value) or value<0 for value in durations):
            raise ValueError('invalid native segment durations')
        for index,path in enumerate(paths):
            if any(len(p)!=3 or not all(math.isfinite(v) for v in p) for p in path):
                raise ValueError('finite three-dimensional path required')
            stationary=len(path)==2 and path[0]==path[1]
            if stationary and durations[index] and (self.model!='otter' or
                    self.terminal_behavior!='TRIM_PROPULSION'):
                raise ValueError('bounded intermediate wait requires stationary Otter trim')
            if stationary and len(paths)>1 and durations[index]==0:
                raise ValueError('intermediate stationary path requires finite duration')
            if not stationary and any(math.dist(a,b)==0 for a,b in zip(path,path[1:])):raise ValueError('zero-length path leg')
        if any(math.dist(a[-1],b[0])>1e-9 for a,b in zip(paths,paths[1:])):
            raise ValueError('fragment paths are disconnected')
        efforts=(tuple(effort) if isinstance(effort,(tuple,list)) else (effort,)*len(paths))
        if (len(efforts)!=len(paths) or any(not math.isfinite(value) or value<0 for value in efforts) or
                (self.model=='remus100' and any(value>self.vehicle.nMax for value in efforts))):
            raise ValueError('invalid native segment propulsion efforts')
        if math.dist(paths[0][0],self.snapshot()['position'])>NATIVE_START_TOLERANCE_M:
            raise ValueError('prediction path start differs from supplied state')
        import hashlib,pickle
        fingerprint=hashlib.sha256(pickle.dumps((self,paths,efforts,durations,scene,dt,terminal_speed,hold_duration,max_model_time))).hexdigest()
        if resume is not None and (resume.get('status')!='FEASIBLE' or
                resume.get('source_fingerprint')!=fingerprint or 'terminal_backend' not in resume or
                resume.get('terminal_wait_s',0.)>terminal_wait_s):
            raise ValueError('terminal continuation does not match this exact motion snapshot')
        model=copy.deepcopy(resume['terminal_backend'] if resume is not None else self)
        start=self.time_s
        mode='SURFACE' if model.model=='otter' else 'WATER'
        segment,point=0,1
        segment_started=model.time_s
        fixed_signature=None;fixed_position=None;fixed_segment=None
        coasting=resume is not None
        coast_start=resume['coast_start_s'] if resume is not None else None
        settled=resume['settled_model_time_s'] if resume is not None else None
        state=model.snapshot()
        samples=list(resume['trajectory']) if resume is not None else [(0.,state['position'])]
        summary=dict(status='UNKNOWN',reason='MODEL_HORIZON_EXHAUSTED',snapshot_steps=self.steps,
                     snapshot_model_time_s=self.time_s,collision_radius_m=self.collision_radius_m,
                     geometry_checked=scene is not None)
        initial_reason=scene.violation(state['position'],model.collision_radius_m) if scene else ''
        if state['actual_mode']!=mode:initial_reason=initial_reason or 'NATIVE_DOMAIN_VIOLATION'
        if initial_reason:summary.update(status='INFEASIBLE',reason=initial_reason)
        while not initial_reason and model.time_s-start<max_model_time:
            if time.monotonic()>=deadline:
                summary['reason']='QUERY_BUDGET_EXHAUSTED';break
            if coasting and settled is not None and model.time_s-settled>=hold_duration+terminal_wait_s:
                summary.update(status='FEASIBLE' if scene is not None else 'UNKNOWN',
                    reason='NATIVE_TERMINAL_VERIFIED_IN_ROLLOUT' if scene is not None else 'SCENE_GEOMETRY_NOT_PROVIDED')
                break
            target=None
            waiting=False
            if not coasting:
                if paths[segment][0]==paths[segment][1] and durations[segment]:
                    if model.time_s-segment_started+1e-9<durations[segment]:
                        waiting=True
                    else:
                        segment+=1;point=1;segment_started=model.time_s
                        if segment==len(paths):coasting=True
                if not waiting and not coasting:
                    previous_segment=segment
                    segment,point,target,coasting=advance_path_target(paths,segment,point,state['position'])
                    if segment!=previous_segment:
                        segment_started=model.time_s
                        if not coasting and paths[segment][0]==paths[segment][1] and durations[segment]:
                            waiting=True;target=None
                if coasting:coast_start=model.time_s-start
            leg_start=paths[segment][point-1] if target is not None else None
            state=model.step(dt,target,0. if coasting or waiting else efforts[segment],leg_start)
            elapsed=model.time_s-start
            samples.append((elapsed,state['position']))
            reason=scene.violation(state['position'],model.collision_radius_m) if scene else ''
            if reason or state['actual_mode']!=mode:
                summary.update(status='INFEASIBLE',reason=reason or 'NATIVE_DOMAIN_VIOLATION');break
            if waiting and state['position']==fixed_position and segment==fixed_segment:
                signature=pickle.dumps({key:value for key,value in vars(model).items()
                    if key not in ('time_s','steps')},protocol=4)
                if signature==fixed_signature:
                    # Exact native Otter trim fixed point under zero effort:
                    # F(x,constant input)=x, so every later stationary state is
                    # x by induction. Only this read-only forecast advances
                    # predicted counters; pvs_node still integrates each tick.
                    while model.time_s-segment_started+1e-9<durations[segment]:
                        if time.monotonic()>=deadline:
                            summary['reason']='QUERY_BUDGET_EXHAUSTED';break
                        model.time_s+=dt;model.steps+=1
                        samples.append((model.time_s-start,state['position']))
                    if summary['reason']=='QUERY_BUDGET_EXHAUSTED':break
                fixed_signature=signature
            elif waiting:
                fixed_signature=pickle.dumps({key:value for key,value in vars(model).items()
                    if key not in ('time_s','steps')},protocol=4)
                fixed_position=state['position'];fixed_segment=segment
            else:
                fixed_signature=fixed_position=fixed_segment=None
            speed=math.sqrt(sum(v*v for v in state['world_velocity']))
            if coasting and speed<=terminal_speed:
                if settled is None:settled=model.time_s
            else:settled=None
            if settled is not None and model.time_s-settled>=hold_duration+terminal_wait_s:
                summary.update(status='FEASIBLE' if scene is not None else 'UNKNOWN',
                    reason='NATIVE_TERMINAL_VERIFIED_IN_ROLLOUT' if scene is not None else 'SCENE_GEOMETRY_NOT_PROVIDED')
                break
        duration=model.time_s-start
        summary.update(duration_s=duration,coast_start_s=coast_start,
            source_fingerprint=fingerprint,settled_model_time_s=settled,terminal_wait_s=terminal_wait_s,
            motion_s=duration if coast_start is None else coast_start,
            terminal_s=0. if coast_start is None else duration-coast_start,
            terminal_position=model.snapshot()['position'],terminal_mode=model.snapshot()['actual_mode'],
            trajectory=samples)
        if include_state:summary['terminal_backend']=model
        return summary

    def predict_idle(self,duration,scene,deadline,dt=.01):
        """Propagate the existing zero-propulsion terminal behavior while waiting.

        COAST_STOP is a measured low-speed condition, not a frozen pose. This
        forecast retains native controller and actuator state for a bounded wait.
        """
        if not math.isfinite(duration) or duration<0 or not math.isfinite(deadline):
            raise ValueError('finite idle duration and deadline required')
        if not math.isfinite(dt) or not 0<dt<=.05:raise ValueError('invalid native step')
        model=copy.deepcopy(self);start=model.time_s
        mode='SURFACE' if model.model=='otter' else 'WATER'
        samples=[(0.,model.snapshot()['position'])]
        status='FEASIBLE' if scene is not None else 'UNKNOWN'
        reason='BOUNDED_NATIVE_IDLE' if scene is not None else 'SCENE_GEOMETRY_NOT_PROVIDED'
        initial_reason=scene.violation(model.snapshot()['position'],model.collision_radius_m) if scene else ''
        if model.snapshot()['actual_mode']!=mode:initial_reason=initial_reason or 'NATIVE_DOMAIN_VIOLATION'
        if initial_reason:status,reason='INFEASIBLE',initial_reason
        # The real node uses fixed outer steps. Quantize the bounded wait up
        # to a whole step instead of inventing fractional integration samples.
        for _ in range(0 if initial_reason else math.ceil(duration/dt)):
            if time.monotonic()>=deadline:status,reason='UNKNOWN','QUERY_BUDGET_EXHAUSTED';break
            state=model.step(dt,None,0.)
            samples.append((model.time_s-start,state['position']))
            violation=scene.violation(state['position'],model.collision_radius_m) if scene else ''
            if violation or state['actual_mode']!=mode:
                status,reason='INFEASIBLE',violation or 'NATIVE_DOMAIN_VIOLATION';break
        return dict(status=status,reason=reason,duration_s=model.time_s-start,requested_wait_s=duration,
                    terminal_position=model.snapshot()['position'],trajectory=samples,terminal_backend=model)

    def execution_state_bytes(self):
        """Canonical native state affecting the next reference, not time counters.

        Raw pickle of a vehicle object depends on internal aliasing even when
        every physical/controller value is equal. Encode values and array bits
        in stable key order so a local prediction and the running node can
        compare the same state across processes.
        """
        from .contracts import canonical_model_state_bytes
        state={key:value for key,value in vars(self).items() if key not in ('time_s','steps')}
        return canonical_model_state_bytes(state)

    def numeric_state_claim(self):
        """Current plant, actuators and native autopilot memory for repair."""
        if self.model!='otter':raise ValueError('numeric state claim is qualified for Otter only')
        controls=('ref','tauX','e_int','psi_d','r_d','a_d')
        def scalar(value):
            kind=('numpy64' if isinstance(value,np.float64) else
                  'int' if type(value) is int else 'float')
            return [kind,float(value)]
        return dict(model=self.model,eta=self.eta.tolist(),nu=self.nu.tolist(),
                    actuators=self.actuators.tolist(),time_s=self.time_s,steps=self.steps,
                    autopilot={key:scalar(getattr(self.vehicle,key)) for key in controls})

    def apply_numeric_state_claim(self,claim):
        """Update a private query copy; the digest check is owned by the caller."""
        if self.model!='otter':raise ValueError('numeric state claim is qualified for Otter only')
        controls=('ref','tauX','e_int','psi_d','r_d','a_d')
        if (claim.get('model')!=self.model or set(claim)!=
                {'model','eta','nu','actuators','time_s','steps','autopilot'} or
                set(claim['autopilot'])!=set(controls) or
                any(len(claim['autopilot'][key])!=2 or
                    claim['autopilot'][key][0] not in ('numpy64','int','float') for key in controls) or
                len(claim['eta'])!=6 or len(claim['nu'])!=6 or
                len(claim['actuators'])!=len(self.actuators) or
                type(claim['steps']) is not int or claim['steps']<0):
            raise ValueError('incomplete native numeric state claim')
        values=(tuple(claim['eta'])+tuple(claim['nu'])+tuple(claim['actuators'])+
                (claim['time_s'],)+tuple(claim['autopilot'][key][1] for key in controls))
        if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in values):
            raise ValueError('nonfinite native numeric state claim')
        self.eta=np.asarray(claim['eta'],dtype=float)
        self.nu=np.asarray(claim['nu'],dtype=float)
        self.actuators=np.asarray(claim['actuators'],dtype=float)
        self.time_s=float(claim['time_s']);self.steps=claim['steps']
        for key in controls:
            kind,value=claim['autopilot'][key]
            setattr(self.vehicle,key,np.float64(value) if kind=='numpy64' else
                    int(value) if kind=='int' else float(value))
        return self

    def snapshot(self):
        from python_vehicle_simulator.lib.gnc import Rzyx
        rotation=Rzyx(*self.eta[3:])
        mode=('SURFACE' if abs(self.eta[2])<=self.vehicle.T else 'OUTSIDE_NATIVE_DOMAIN') if self.model=='otter' else ('WATER' if self.eta[2]>0 else 'OUTSIDE_NATIVE_DOMAIN')
        return dict(model=self.model,model_time_s=self.time_s,steps=self.steps,
                    collision_radius_m=self.collision_radius_m,
                    collision_geometry='PVS_DECLARED_HULL_PROXY_NOT_EQUIPMENT_ENVELOPE',
                    actual_mode=mode,
                    position=tuple(ENU_FROM_NED@self.eta[:3]),
                    quaternion_wxyz=ned_attitude_to_ros(*self.eta[3:]),
                    world_velocity=tuple(ENU_FROM_NED@rotation@self.nu[:3]),
                    body_velocity=tuple(FLU_FROM_FRD@self.nu[:3]),
                    body_angular_velocity=tuple(FLU_FROM_FRD@self.nu[3:]),
                    actuators=tuple(self.actuators),trim_command=self.trim_command,
                    energy_model='UNAVAILABLE')
