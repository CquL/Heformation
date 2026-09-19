import pytest
from qn_aav_simulator.observation_coverage import DeliveryProduct,FiniteDelivery


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
