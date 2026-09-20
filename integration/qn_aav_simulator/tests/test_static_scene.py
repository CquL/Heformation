import pytest
from qn_aav_simulator.experiment_verdict import StaticSceneGeometry


def scene():
    return StaticSceneGeometry.from_mapping(dict(geometry_enabled=True,frame_id='world',
        seabed_z_m=-6.,required_clearance_m=.2,objects=[
            dict(id='solid',kind='SOLID',center=[0.,0.,-2.],size=[1.,1.,1.]),
            dict(id='restricted',kind='FORBIDDEN',center=[5.,0.,0.],size=[1.,1.,2.])]))


def test_swept_path_and_hull_radius_count_even_when_endpoints_clear():
    s=scene()
    assert not s.violation((-2.,0.,-2.),.8)
    assert not s.violation((2.,0.,-2.),.8)
    assert 'solid' in s.path_violation([(-2.,0.,-2.),(2.,0.,-2.)],.8)
    assert 'solid' in s.violation((1.4,0.,-2.),.8)
    assert not s.violation((1.6,0.,-2.),.8)


def test_seabed_uses_body_envelope_and_forbidden_is_not_missing():
    s=scene()
    assert 'seabed' in s.violation((-5.,0.,-5.4),.5)
    assert 'restricted' in s.violation((5.,0.,0.),.25)
    assert not s.violation((-5.,0.,0.),.5)  # surface is a medium boundary
    with pytest.raises(ValueError):s.path_violation([],.5)
    with pytest.raises(ValueError):s.violation((float('nan'),0.,0.),.5)


def test_legacy_scene_does_not_silently_enable_new_checks():
    assert StaticSceneGeometry.from_mapping({'obstacle_present':False}) is None
