import math
import pytest
from qn_aav_simulator.task_line import assembly_member_order


def test_dispersed_members_approach_without_crossing_occupied_slots():
    actual={'drone_0':(-21.,3.4,.8),'drone_1':(-29.,3.2,.8),'drone_2':(-25.,-3.2,.8)}
    targets={'drone_0':(-30.,6.,.8),'drone_1':(-30.,4.,.8),'drone_2':(-30.,8.,.8)}
    order=assembly_member_order(actual,targets,1.)
    assert order.index('drone_2')<order.index('drone_0')


def test_no_safe_order_is_rejected_instead_of_relaxing_clearance():
    points={'a':(0.,0.,0.),'b':(.5,0.,0.),'c':(1.,0.,0.)}
    with pytest.raises(ValueError,match='no ordered'):
        assembly_member_order(points,points,1.)
