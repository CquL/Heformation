"""Thin Otter/REMUS100 boundary over the vendored Fossen implementation.

All integration and actuator/control dynamics remain in PVS. ROS map is ENU,
body is FLU; native PVS is NED/FRD. No position is assigned after initialization.
"""
import math
import numpy as np


ENU_FROM_NED=np.array([[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]])
FLU_FROM_FRD=np.diag([1.,-1.,-1.])


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
        self.trim_command=0.
        if initialization_mode not in ('NATIVE_ZERO','STATIC_TRIM'):
            raise ValueError('unknown PVS initialization mode')
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

    def snapshot(self):
        from python_vehicle_simulator.lib.gnc import Rzyx
        rotation=Rzyx(*self.eta[3:])
        mode=('SURFACE' if abs(self.eta[2])<=self.vehicle.T else 'OUTSIDE_NATIVE_DOMAIN') if self.model=='otter' else ('WATER' if self.eta[2]>0 else 'OUTSIDE_NATIVE_DOMAIN')
        return dict(model=self.model,model_time_s=self.time_s,steps=self.steps,
                    actual_mode=mode,
                    position=tuple(ENU_FROM_NED@self.eta[:3]),
                    quaternion_wxyz=ned_attitude_to_ros(*self.eta[3:]),
                    world_velocity=tuple(ENU_FROM_NED@rotation@self.nu[:3]),
                    body_velocity=tuple(FLU_FROM_FRD@self.nu[:3]),
                    body_angular_velocity=tuple(FLU_FROM_FRD@self.nu[3:]),
                    actuators=tuple(self.actuators),trim_command=self.trim_command,
                    energy_model='UNAVAILABLE')
