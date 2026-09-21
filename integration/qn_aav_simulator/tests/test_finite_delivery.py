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


def test_received_unobserved_product_is_not_delivered_and_future_generation_cannot_send():
    ledger=FiniteDelivery()
    ledger.produce('p',DeliveryProduct('uuv','mother',10,1.,False))
    assert ledger.advance('radio',1.,100.,[('p','uuv','mother',True)])==()
    ledger.advance('radio',2.,100.,[('p','uuv','mother',True)])
    assert not ledger.products['p'].delivered
    with pytest.raises(ValueError):
        ledger.advance('radio',1.,100.,[])
