#!/usr/bin/env python3
"""Offline replay of adopted position references through the SAME qn backend.

Only for the recorded single-member handover experiment (-30,6,.8 initial trim).
Recorded actual positions are comparison evidence, never plant inputs. No reset
occurs after initialization. Retain original equations for failure diagnosis.
"""
import argparse
from dataclasses import replace
import json
import math
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'integration/qn_aav_simulator/src'))
from qn_aav_simulator.contracts import AgentState,ControlCmd,CommandMode,PlatformAdapterCmd,PlantStepInput
from qn_aav_simulator.qn_python_backend import QnPythonClosedLoopBackend
from qn_aav_simulator.qn_dynamics import actuator_wrench,buoyancy_wrench,mass_and_inertia


def replay(path,output):
    recorded=json.loads(path.read_text())['samples']
    backend=QnPythonClosedLoopBackend(dict(initialization_mode='STATIC_TRIM',model_step_s=.001,
        reference_mode='ROUTE_POSITION',water_guidance_mode='LOS_VELOCITY_REFERENCE',
        water_horizontal_controller_mode='LOS_SURGE_YAW'))
    state=AgentState('drone_0','AAV',0.,(-30.,6.,.8),(0.,0.,0.))
    backend.reset(state)
    samples=[]
    worst=0.
    missing_actual=[]
    for index,row in enumerate(recorded):
        if int(row['used_outer_step'])!=index:
            raise ValueError('missing/repeated model step: replay cannot invent a reference')
        desired=tuple(float(row['reference_position_'+axis]) for axis in 'xyz')
        cmd=ControlCmd('recorded-'+str(index),'drone_0',float(row['ros_stamp']),
            CommandMode.DESIRED_POSITION,(0.,0.,0.),desired_position=desired,
            desired_yaw_rad=float(row['used_reference_yaw_rad']),frame_id='world')
        result=backend.step(PlantStepInput(state,cmd,PlatformAdapterCmd('drone_0'),.01,2.,8.))
        actual=tuple(row['position_'+axis] for axis in 'xyz')
        residual=None
        if any(v is None for v in actual):
            missing_actual.append(index)
        else:
            residual=math.dist(result.position,tuple(float(v) for v in actual))
            worst=max(worst,residual)
        state=replace(state,position=result.position,velocity=result.velocity)
        plant=backend._state.plant
        flag=result.medium_flag
        mass,_=mass_and_inertia(flag,backend.constants)
        buoyancy,branch=buoyancy_wrench(plant.quaternion_wxyz,result.position[2],backend.constants)
        force=actuator_wrench(plant.actuator_outputs,flag,backend.constants)
        samples.append(dict(step=index,time_s=(index+1)*.01,z=result.position[2],
            vz=result.velocity[2],reference_z=desired[2],flag=flag,
            residual_m=residual,mass_kg=mass,buoyancy_z=buoyancy[2],buoyancy_branch=branch,
            actuator_force_z=force[2],actuator_outputs=plant.actuator_outputs))
        if index%5000==0:print('replayed',index,'max residual',worst,flush=True)
    report=dict(scope='offline original-backend replay, not new controller qualification',
        samples=samples,maximum_position_residual_m=worst,
        missing_actual_state_steps=missing_actual,
        matches_recorded_positions=not missing_actual and worst<=1e-6)
    output.write_text(json.dumps(report,indent=2)+'\n')
    print('maximum_position_residual_m',worst,flush=True)
    return report['matches_recorded_positions']


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('recording',type=Path)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    raise SystemExit(0 if replay(args.recording,args.output) else 1)
