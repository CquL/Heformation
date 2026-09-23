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
    return segment,point,target,False


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

    def step(self,dt,target=None,effort=0.):
        from python_vehicle_simulator.lib.gnc import attitudeEuler
        if not math.isfinite(dt) or not 0<dt<=.05 or not math.isfinite(effort) or effort<0:
            raise ValueError('invalid native model step/control effort')
        if target is not None:
            if len(target)!=3 or not all(math.isfinite(v) for v in target):
                raise ValueError('target must be a finite ENU point')
            delta=np.array([target[1],target[0],-target[2]])-self.eta[:3]
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
                                terminal_speed=.03,hold_duration=4.,max_model_time=180.,include_state=False,terminal_wait_s=0.,resume=None):
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
        for path in paths:
            if any(len(p)!=3 or not all(math.isfinite(v) for v in p) for p in path):
                raise ValueError('finite three-dimensional path required')
            stationary=len(paths)==1 and len(path)==2 and path[0]==path[1]
            if not stationary and any(math.dist(a,b)==0 for a,b in zip(path,path[1:])):raise ValueError('zero-length path leg')
        if any(math.dist(a[-1],b[0])>1e-9 for a,b in zip(paths,paths[1:])):
            raise ValueError('fragment paths are disconnected')
        if math.dist(paths[0][0],self.snapshot()['position'])>NATIVE_START_TOLERANCE_M:
            raise ValueError('prediction path start differs from supplied state')
        import hashlib,pickle
        fingerprint=hashlib.sha256(pickle.dumps((self,paths,effort,scene,dt,terminal_speed,hold_duration,max_model_time))).hexdigest()
        if resume is not None and (resume.get('status')!='FEASIBLE' or
                resume.get('source_fingerprint')!=fingerprint or 'terminal_backend' not in resume or
                resume.get('terminal_wait_s',0.)>terminal_wait_s):
            raise ValueError('terminal continuation does not match this exact motion snapshot')
        model=copy.deepcopy(resume['terminal_backend'] if resume is not None else self)
        start=self.time_s
        mode='SURFACE' if model.model=='otter' else 'WATER'
        segment,point=0,1
        coasting=resume is not None
        coast_start=resume['coast_start_s'] if resume is not None else None
        settled=resume['settled_model_time_s'] if resume is not None else None
        samples=list(resume['trajectory']) if resume is not None else [(0.,model.snapshot()['position'])]
        summary=dict(status='UNKNOWN',reason='MODEL_HORIZON_EXHAUSTED',snapshot_steps=self.steps,
                     snapshot_model_time_s=self.time_s,collision_radius_m=self.collision_radius_m,
                     geometry_checked=scene is not None)
        initial_reason=scene.violation(model.snapshot()['position'],model.collision_radius_m) if scene else ''
        if model.snapshot()['actual_mode']!=mode:initial_reason=initial_reason or 'NATIVE_DOMAIN_VIOLATION'
        if initial_reason:summary.update(status='INFEASIBLE',reason=initial_reason)
        while not initial_reason and model.time_s-start<max_model_time:
            if time.monotonic()>=deadline:
                summary['reason']='QUERY_BUDGET_EXHAUSTED';break
            if coasting and settled is not None and model.time_s-settled>=hold_duration+terminal_wait_s:
                summary.update(status='FEASIBLE' if scene is not None else 'UNKNOWN',
                    reason='NATIVE_TERMINAL_VERIFIED_IN_ROLLOUT' if scene is not None else 'SCENE_GEOMETRY_NOT_PROVIDED')
                break
            target=None
            if not coasting:
                segment,point,target,coasting=advance_path_target(paths,segment,point,model.snapshot()['position'])
                if coasting:coast_start=model.time_s-start
            state=model.step(dt,target,0. if coasting else effort)
            elapsed=model.time_s-start
            samples.append((elapsed,state['position']))
            reason=scene.violation(state['position'],model.collision_radius_m) if scene else ''
            if reason or state['actual_mode']!=mode:
                summary.update(status='INFEASIBLE',reason=reason or 'NATIVE_DOMAIN_VIOLATION');break
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
