"""Exercise the real worker's disposition lifetime without a ROS process.

Message/clock doubles provide transport only; ownership, segments and worker
methods are the production implementations. Dynamics remain real-run evidence.
"""
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest

from qn_aav_simulator.platform_execution import ReferenceOwnership, Segment


@pytest.fixture
def worker(monkeypatch):
    for name in ['actionlib','rospy','diagnostic_msgs','diagnostic_msgs.msg',
                 'std_msgs','std_msgs.msg',
                 'qn_aav_simulator.msg','qn_aav_simulator.srv']:
        monkeypatch.setitem(sys.modules,name,ModuleType(name))
    sys.modules['diagnostic_msgs.msg'].DiagnosticArray=object
    sys.modules['std_msgs.msg'].String=SimpleNamespace
    for name in ['PlatformTaskAction','PlatformTaskFeedback','FormationActionResult']:
        setattr(sys.modules['qn_aav_simulator.msg'],name,object)
    sys.modules['qn_aav_simulator.msg'].PlatformTaskResult=SimpleNamespace
    for name in ['TakeReference','TakeReferenceResponse']:
        setattr(sys.modules['qn_aav_simulator.srv'],name,object)
    sys.modules['rospy'].Time=SimpleNamespace(now=lambda:SimpleNamespace(to_sec=lambda:100.))
    path=Path(__file__).parents[1]/'src/qn_aav_simulator/platform_action.py'
    spec=importlib.util.spec_from_file_location('local_disposition_under_test',path)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    obj=module.LocalPlatformAction.__new__(module.LocalPlatformAction)
    obj.node=SimpleNamespace(state=SimpleNamespace(position=(0.,0.,-.08),medium_flag=.97,
                                                  orientation_quat_wxyz=(1.,0.,0.,0.)),
                             clock=SimpleNamespace(model_time_s=5.))
    obj.owner=ReferenceOwnership()
    assert obj.owner.claim('goal','PLATFORM',0)[0]
    obj.fault_hold_modes=None
    obj.fault_hold_operation=''
    obj.fault_transition=None
    obj.terminal_mode='WATER'
    obj.transition_fault_behavior='COMPLETE_ACCEPTED_VERTICAL_SEGMENT'
    obj.hold_point=(0.,0.,-.6)
    obj.hold_yaw=0.
    obj.air_floor=-1
    obj.events=[]
    handle=SimpleNamespace(set_aborted=lambda result:obj.events.append(result))
    segment=Segment('EXIT_WATER',((0.,0.,-.6),(0.,0.,.8)),14.)
    obj.work=dict(id='goal',task='inspection',handle=handle,segments=[segment],index=0,
                  start=0.,waiting_start=False,cause='',settled=None,deadline=150.,fault_allowed=None)
    return obj


def test_repeat_cancel_preserves_accepted_segment_epoch_and_deadline(worker):
    worker._begin_disposition('CANCEL_REQUEST')
    original=(worker.fault_transition,worker.owner.generation,worker.work['deadline'])
    worker.node.clock.model_time_s=8.
    worker._begin_disposition('CANCEL_REQUEST')
    assert original==(worker.fault_transition,worker.owner.generation,worker.work['deadline'])
    assert worker.fault_transition[0].operation=='EXIT_WATER'
    assert worker.owner.locked


def test_observation_timeout_does_not_discard_physical_conversion(worker):
    worker._begin_disposition('CANCEL_REQUEST')
    segment,start=worker.fault_transition
    worker._finish(False,'OBSERVATION_TIMEOUT_UNVERIFIED')
    assert worker.work is None and not worker.events[0].terminal_verified
    worker.node.clock.model_time_s=8.
    assert worker.tick()['position']==segment.reference(8.-start)
    worker.node.clock.model_time_s=20.
    assert worker.tick()['position']==segment.points[-1]
    assert not worker.owner.claim('new','AIR_SWARM',worker.owner.generation)[0]


def test_verified_observation_publishes_terminal_report_before_success(worker):
    published=[];results=[]
    worker.products=SimpleNamespace(publish=lambda message:published.append(message.data))
    worker.work['observations']=SimpleNamespace(points=('sample',),emitted={'sample'},
        terminal_report=lambda stamp:dict(event_type='OBSERVATION_TERMINAL',
                                          generated_at=stamp))
    worker.work['handle']=SimpleNamespace(set_succeeded=lambda result:results.append(result))
    worker._finish(True,'COMPLETED_LOCAL_FRAGMENT')
    assert len(published)==len(results)==1
    assert 'OBSERVATION_TERMINAL' in published[0]
    assert results[0].terminal_verified and not results[0].resource_locked


def test_air_to_air_handover_keeps_previous_adopted_yaw(worker):
    worker.work=None
    worker.owner.source='AIR_SWARM'
    worker.air_adopted=False
    worker.node.state.position=(-10.,0.,.8)
    worker.node.latest_command=dict(reference_source='AIR_SWARM',yaw_rad=-2.488)
    worker.node.command_buffer_size=16
    worker._flush_reference()
    assert worker.node.latest_command is None
    assert worker.tick()['position']==(-10.,0.,.8)
    assert worker.tick()['yaw_rad']==pytest.approx(-2.488)
    worker.owner.source='PLATFORM'
    assert worker.tick()['yaw_rad']==0.


def test_domain_failure_supersedes_cancel_without_using_unreliable_position(worker):
    worker._begin_disposition('CANCEL_REQUEST')
    segment,start=worker.fault_transition
    worker.node.clock.model_time_s=8.
    worker.node.state.position=(float('nan'),)*3
    worker._begin_disposition('DOMAIN_VIOLATION')
    assert worker.fault_transition is None
    assert worker.hold_point==segment.reference(8.-start)
    assert worker.work['cause']=='DOMAIN_VIOLATION'
    assert worker.owner.locked


@pytest.mark.parametrize('reason,waiting',[('DOMAIN_VIOLATION',False),('CANCEL_REQUEST',True)])
def test_unreliable_domain_or_unstarted_segment_does_not_continue(worker,reason,waiting):
    worker.work['waiting_start']=waiting
    worker._begin_disposition(reason)
    assert worker.fault_transition is None
    assert worker.hold_point==worker.node.state.position
