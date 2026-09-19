from pathlib import Path
import math
import sys
import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'upstream/Fossen/src'))
from qn_aav_simulator.pvs_backend import ENU_FROM_NED, FLU_FROM_FRD, ned_attitude_to_ros, PvsBackend
from qn_aav_simulator.odometry import rotation_from_quaternion
from python_vehicle_simulator.lib.gnc import Rzyx


@pytest.mark.parametrize('angles',[(0,0,0),(.2,-.3,1.4),(.1,.2,-2.9)])
def test_complete_attitude_and_twist_transform(angles):
    expected=ENU_FROM_NED@Rzyx(*angles)@FLU_FROM_FRD
    actual=np.array(rotation_from_quaternion(ned_attitude_to_ros(*angles)))
    assert np.allclose(actual,expected,atol=1e-12)
    native=np.array([.6,-.2,.3])
    assert np.allclose(actual@(FLU_FROM_FRD@native),ENU_FROM_NED@Rzyx(*angles)@native)
    assert np.allclose(ENU_FROM_NED@ENU_FROM_NED,np.eye(3))


@pytest.mark.parametrize('model,z,effort',[('otter',0.,20.),('remus100',-2.,500.)])
def test_native_integrator_drives_position_and_coasting_keeps_state(model,z,effort):
    b=PvsBackend(model,(0.,0.,z))
    for _ in range(200):
        s=b.step(.01,(10.,0.,z),effort)
    assert s['position'][0]>0
    # Native REMUS propeller/attitude coupling can produce transverse motion;
    # the boundary test checks the commanded axis, not perfect straight tracking.
    assert s['position'][0]>abs(s['position'][1])
    before=s['position'][0]
    after=b.step(.01,None,0.)
    assert after['position'][0]>before
    assert after['steps']==201
    assert math.isclose(after['model_time_s'],2.01,abs_tol=1e-10)


def test_otter_static_payload_trim_has_zero_native_acceleration():
    b=PvsBackend('otter',initialization_mode='STATIC_TRIM')
    initial=b.snapshot()['position']
    assert b.trim_command<0
    for _ in range(300):
        s=b.step(.01,None,0.)
    assert np.allclose(s['position'],initial,atol=1e-9)
    assert np.linalg.norm(s['world_velocity'])<1e-9


def test_uuv_actual_domain_does_not_come_from_its_configured_type():
    b=PvsBackend('remus100',(0.,0.,-2.))
    assert b.snapshot()['actual_mode']=='WATER'
    # Explicit state fault injection for the domain classifier only.
    b.eta[2]=-.1
    assert b.snapshot()['actual_mode']=='OUTSIDE_NATIVE_DOMAIN'
