"""Water guidance must consume the adopted position stream, not a second input."""
import math

from qn_aav_simulator.contracts import AgentState, CommandMode, ControlCmd, PlantStepInput, PlatformAdapterCmd
from qn_aav_simulator.qn_python_backend import QnPythonClosedLoopBackend


def backend(los, height):
    b=QnPythonClosedLoopBackend(dict(initialization_mode='STATIC_TRIM',
        reference_mode='ROUTE_POSITION',
        water_guidance_mode='LOS_VELOCITY_REFERENCE' if los else 'QN_ORIGINAL_POSITION',
        water_horizontal_controller_mode='LOS_SURGE_YAW' if los else 'QN_ORIGINAL_RBF_PD'))
    s=AgentState('a','AAV',0.,(0.,0.,height),(0.,0.,0.))
    b.reset(s)
    return b,s


def step(b,s,p):
    c=ControlCmd('c','a',0.,CommandMode.DESIRED_POSITION,(99.,0.,0.),desired_position=p)
    return b.step(PlantStepInput(s,c,PlatformAdapterCmd('a'),.01,2.,8.))


def test_route_water_guidance_is_active_and_uses_position_derivative():
    b,s=backend(True,-.5)
    step(b,s,(0.,.001,-.5))
    assert b._water_guidance_active
    assert math.isclose(b._water_horizontal_speed_mps,.1)
    assert b._water_desired_heading_rad > 0
    # Constant position is a zero-speed request even if another field says 99.
    step(b,s,(0.,.001,-.5))
    assert b._water_horizontal_speed_mps == 0


def test_opt_in_water_guidance_preserves_air_dynamics():
    original,s=backend(False,.5)
    enabled,_=backend(True,.5)
    for k in range(20):
        target=(k*.001,0.,.5)
        first=step(original,s,target)
        second=step(enabled,s,target)
        assert first.position == second.position
        assert first.velocity == second.velocity
        assert not enabled._water_guidance_active
