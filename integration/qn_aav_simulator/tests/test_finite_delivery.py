import pytest
from qn_aav_simulator.observation_coverage import DeliveryProduct,FiniteDelivery


def test_delivery_prediction_cannot_extend_a_finished_physical_commitment():
    import time
    from pathlib import Path
    from qn_aav_simulator.task_line import load_request
    from qn_aav_simulator.observation_coverage import predict_received_products
    request=load_request(Path(__file__).parents[1]/'config/monitoring_request_joint.yaml')
    # Synthetic checked intervals isolate communication causality, not GNC.
    traces={'uuv':[(i/10.,(0.,8.,-2.),'WATER') for i in range(201)],
            'usv':[(i/10.,(0.,8.,0.),'SURFACE') for i in range(201)]}
    result=predict_received_products(request,['water_sample'],'uuv','goal',traces,(15.,-8.,0.),(),time.monotonic()+1.)
    assert result['status']=='FEASIBLE' and 17.<=result['receipt_finish_s']<=18.
    traces['uuv']=traces['uuv'][:101]
    result=predict_received_products(request,['water_sample'],'uuv','goal',traces,(15.,-8.,0.),(),time.monotonic()+1.)
    assert result['status']=='INFEASIBLE' and not result['received_at']


@pytest.mark.parametrize('order',[('acoustic','radio'),('radio','acoustic')])
def test_separate_channels_share_step_start_and_do_not_instantly_relay(order):
    ledger=FiniteDelivery()
    ledger.produce('p',DeliveryProduct('uuv','mother',100,0.,True))
    channels={'acoustic':(100.,[('p','uuv','usv',True)]),
              'radio':(100.,[('p','usv','mother',True)])}
    for channel in order:
        assert ledger.advance(channel,1.,*channels[channel])==()
    assert ledger.products['p'].received_prefix['usv']==100
    assert ledger.products['p'].received_prefix.get('mother',0)==0
    assert ledger.advance_all(2.,channels)==('p',)
    assert ledger.advance_all(2.,channels)==()


def test_two_hops_cannot_forward_data_before_receipt_or_while_disconnected():
    ledger=FiniteDelivery()
    ledger.produce('p',DeliveryProduct('uuv','mother',100,0.,True))
    both=[('p','uuv','usv',True),('p','usv','mother',True)]
    assert ledger.advance('radio',1.,100.,both)==()
    assert ledger.products['p'].received_prefix['usv']==100
    assert ledger.products['p'].received_prefix.get('mother',0)==0
    assert ledger.advance('radio',2.,100.,[('p','usv','mother',False)])==()
    assert ledger.advance('radio',3.,50.,both)==()
    assert ledger.products['p'].received_prefix['mother']==50
    assert ledger.advance('radio',4.,50.,both)==('p',)
    assert ledger.products['p'].delivered
    assert ledger.advance('radio',4.,50.,both)==()
    assert ledger.advance('radio',5.,50.,both)==()


def test_shared_capacity_is_not_granted_once_per_product():
    ledger=FiniteDelivery()
    for key in ('a','b'):
        ledger.produce(key,DeliveryProduct('uuv','mother',100,0.,True))
    ledger.advance('radio',1.,100.,[(key,'uuv','mother',True) for key in ('a','b')])
    assert sum(p.received_prefix.get('mother',0) for p in ledger.products.values())==100


def test_air_result_uses_shared_rf_to_usv_then_mother_without_instant_relay():
    from qn_aav_simulator.observation_coverage import declared_delivery_channels
    ledger=FiniteDelivery()
    ledger.produce('air',DeliveryProduct('drone_1','mother',100,0.,True))
    states={'drone_1':((-28.,4.,.8),'AIR'),
            'usv':((-10.,4.,0.),'SURFACE'),
            'mother':((15.,-8.,2.),'SURFACE')}
    channels=declared_delivery_channels(ledger.products,states,states)
    assert ledger.advance_all(.1,channels)==()
    assert ledger.products['air'].received_prefix['usv']==100
    assert ledger.products['air'].received_prefix.get('mother',0)==0
    assert ledger.advance_all(.2,channels)==('air',)


def test_mother_command_uses_rf_then_acoustic_without_instant_downlink():
    from qn_aav_simulator.observation_coverage import declared_delivery_channels
    ledger=FiniteDelivery()
    ledger.produce('command',DeliveryProduct('mother','uuv',100,0.,True))
    states={'mother':((15.,-8.,2.),'SURFACE'),
            'usv':((0.,8.,0.),'SURFACE'),
            'uuv':((0.,8.,-2.),'WATER')}
    channels=declared_delivery_channels(ledger.products,states,states)
    assert ledger.advance_all(.1,channels)==()
    assert ledger.products['command'].received_prefix['usv']==100
    assert ledger.products['command'].received_prefix.get('uuv',0.)==0.
    assert ledger.advance_all(.2,channels)==('command',)


def test_command_and_observation_share_the_same_acoustic_capacity():
    from qn_aav_simulator.observation_coverage import declared_delivery_channels
    ledger=FiniteDelivery()
    ledger.produce('observation',DeliveryProduct('uuv','mother',150,0.,True))
    ledger.produce('command',DeliveryProduct('mother','uuv',150,0.,True))
    states={'mother':((15.,-8.,2.),'SURFACE'),
            'usv':((0.,8.,0.),'SURFACE'),
            'uuv':((0.,8.,-2.),'WATER')}
    channels=declared_delivery_channels(ledger.products,states,states)
    assert ledger.advance_all(.1,channels)==()
    # Both directions use the single declared 2 KiB/s acoustic channel.
    first=ledger.products['observation'].received_prefix.get('usv',0.)
    second=ledger.products['command'].received_prefix.get('uuv',0.)
    assert first+second<=204.8+1e-9
    assert first==150 and second==0
    assert set(ledger.advance_all(.2,channels))=={'command','observation'}


def test_air_product_published_at_action_terminal_needs_committed_following_reference():
    import time
    from qn_aav_simulator.observation_coverage import predict_received_events
    product=dict(product_id='air-goal:air_sample',request_id='r',goal_id='air-goal',
        point_id='air_sample',producer='drone_1',generated_at=11.,observed=True,
        result=dict(model='GEOMETRIC_PROXY',dwell_s=1.),required_bytes=32768)
    air=[(i/10.,(-28.,4.,.8),'AIR') for i in range(111)]
    support=[(i/10.,(-5.,-8.,0.),'SURFACE') for i in range(151)]
    mother=(15.,-8.,2.)
    no_tail=predict_received_events([product],{'drone_1':air,'usv':support},
        {'drone_1':((0.,11.),),'usv':((0.,15.),)},mother,(),time.monotonic()+1.)
    assert no_tail['status']=='INFEASIBLE'
    held_air=air+[(i/10.,(-28.,4.,.8),'AIR') for i in range(111,151)]
    held=predict_received_events([product],{'drone_1':held_air,'usv':support},
        {'drone_1':((0.,15.),),'usv':((0.,15.),)},mother,(),time.monotonic()+1.)
    assert held['status']=='FEASIBLE' and held['received_at'][product['product_id']]>11.


def test_terminal_report_uses_capacity_without_inventing_a_32_kib_product():
    import time
    from qn_aav_simulator.observation_coverage import predict_received_events
    report=dict(event_type='OBSERVATION_TERMINAL',product_id='goal:terminal',
        request_id='r',goal_id='goal',producer='drone_1',
        point_ids=['air_sample'],observed_ids=['air_sample'],generated_at=1.)
    traces={'drone_1':[(i/10.,(-10.,4.,.8),'AIR') for i in range(31)]}
    result=predict_received_events([report],traces,{'drone_1':((0.,3.),)},
        (15.,-8.,2.),(),time.monotonic()+1.)
    assert result['status']=='FEASIBLE' and result['received_at']['goal:terminal']>=1.


def test_received_unobserved_product_is_not_delivered_and_future_generation_cannot_send():
    ledger=FiniteDelivery()
    ledger.produce('p',DeliveryProduct('uuv','mother',10,1.,False))
    assert ledger.advance('radio',1.,100.,[('p','uuv','mother',True)])==()
    ledger.advance('radio',2.,100.,[('p','uuv','mother',True)])
    assert not ledger.products['p'].delivered
    with pytest.raises(ValueError):
        ledger.advance('radio',1.,100.,[])
