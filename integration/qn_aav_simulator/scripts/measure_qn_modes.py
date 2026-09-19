#!/usr/bin/env python3
"""Bounded numerical qualification measurements; not an Action/ROS PASS.

Uses one original qn backend and one reset at initialization. Reference geometry
is an explicit small experiment, not a declared equipment operating envelope.
"""
import argparse
import csv
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from qn_aav_simulator.contracts import AgentState, ControlCmd, CommandMode, PlantStepInput, PlatformAdapterCmd
from qn_aav_simulator.qn_python_backend import QnPythonClosedLoopBackend


def reference(case, t):
    if case == 'water':
        return (min(.5, max(0., (t-5)*.1)), min(.5, max(0., (t-15)*.1)), -.5)
    # AIR settle, descent, WATER traverse, ascent, AIR settle.
    z = .5 if t < 5 else .5-min(1., (t-5)*.1)
    if t >= 25:
        z = -.5+min(1., (t-25)*.1)
    return (min(.5, max(0., (t-15)*.1)), 0., z)


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--case',choices=['water','transition'],required=True)
    p.add_argument('--controller',choices=['original','los'],required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--duration',type=float)
    args=p.parse_args()
    duration=args.duration or (25. if args.case == 'water' else 40.)
    if not math.isfinite(duration) or not 0 < duration <= 120:
        p.error('duration must be finite and in (0,120]')
    args.output.mkdir(parents=True,exist_ok=False)
    config=dict(initialization_mode='STATIC_TRIM',model_step_s=.001,
                reference_mode='ROUTE_POSITION',water_guidance_mode='LOS_VELOCITY_REFERENCE' if args.controller=='los' else 'QN_ORIGINAL_POSITION',
                water_horizontal_controller_mode='LOS_SURGE_YAW' if args.controller=='los' else 'QN_ORIGINAL_RBF_PD')
    backend=QnPythonClosedLoopBackend(config)
    state=AgentState('qualification-aav','AAV',0.,reference(args.case,0.),(0.,0.,0.))
    backend.reset(state)
    start=time.monotonic()
    seen=set()
    report={'scope':'offline qn equations; no ROS Action or task qualification verdict',
            'case':args.case,'config':config,'reset_count':1,'status':'MEASURED',
            'guidance_active_steps':0,'max_reference_error_m':0.,'max_speed_mps':0.}
    source=Path(__file__).resolve().parents[1]/'src/qn_aav_simulator/qn_python_backend.py'
    report['backend_sha256']=hashlib.sha256(source.read_bytes()).hexdigest()
    dt=.01
    with (args.output/'samples.csv').open('w') as out:
        writer=csv.writer(out)
        writer.writerow(['model_time_s','x','y','z','vx','vy','vz','medium_flag','ref_x','ref_y','ref_z','guidance_active'])
        try:
            for k in range(round(duration/dt)):
                t=k*dt
                target=reference(args.case,t+dt)
                cmd=ControlCmd(str(k),state.agent_id,t,CommandMode.DESIRED_POSITION,(0.,0.,0.),desired_position=target)
                result=backend.step(PlantStepInput(state,cmd,PlatformAdapterCmd(state.agent_id),dt,2.,8.))
                values=result.position+result.velocity
                if not all(math.isfinite(v) for v in values) or max(abs(v) for v in values)>100:
                    raise ValueError('numerical divergence or experiment extent exceeded')
                state=AgentState(state.agent_id,'AAV',t+dt,result.position,result.velocity,
                                 orientation_quat_wxyz=result.orientation_quat_wxyz,
                                 body_linear_velocity_mps=result.body_linear_velocity_mps,
                                 body_angular_velocity_radps=result.body_angular_velocity_radps,
                                 medium_flag=result.medium_flag)
                flag=result.medium_flag
                seen.add('AIR' if flag==0 else 'WATER' if flag==1 else 'TRANSITION')
                active=bool(backend._water_guidance_active)
                report['guidance_active_steps']+=int(active)
                report['max_reference_error_m']=max(report['max_reference_error_m'],math.dist(result.position,target))
                report['max_speed_mps']=max(report['max_speed_mps'],math.sqrt(sum(v*v for v in result.velocity)))
                writer.writerow([t+dt,*result.position,*result.velocity,flag,*target,int(active)])
                if k%500==0:
                    print(args.case,args.controller,'t',round(t,2),'p',tuple(round(v,4) for v in result.position),'medium',flag,flush=True)
        except (ArithmeticError,ValueError) as exc:
            report.update(status='NUMERICAL_FAILURE',reason=str(exc))
    report.update(model_time_s=state.timestamp_s,wall_elapsed_s=time.monotonic()-start,
                  modes_observed=sorted(seen),final_state=asdict(state))
    (args.output/'measurement.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False),flush=True)
    return 0 if report['status']=='MEASURED' else 1

if __name__=='__main__':
    raise SystemExit(main())
