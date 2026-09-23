"""Geometric visibility and continuous dwell on actual samples, then receipt.

This binary surrogate does not establish image quality or payload validity.
No exposure, blur, resolution score or arbitrary positive-score acceptance.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, Mapping, Optional, Sequence, Tuple

Vector3 = Tuple[float, float, float]


@dataclass
class DeliveryProduct:
    """A finite in-order byte stream; counters, not a file-transfer service."""
    producer: str
    receiver: str
    required_bytes: int
    generated_at: float
    observed: bool
    received_prefix: Dict[str, float] = field(default_factory=dict)
    received_at: Optional[float] = None

    def __post_init__(self):
        if not self.producer or not self.receiver or self.producer==self.receiver:
            raise ValueError('distinct producer and receiver required')
        if type(self.required_bytes) is not int or self.required_bytes<=0:
            raise ValueError('required_bytes must be a positive integer')
        if not math.isfinite(self.generated_at) or self.generated_at<0:
            raise ValueError('generation time must be finite and nonnegative')
        if type(self.observed) is not bool:
            raise ValueError('observation validity must be explicit boolean')
        self.received_prefix={self.producer:float(self.required_bytes)}

    @property
    def delivered(self):
        return self.observed and self.received_at is not None


class FiniteDelivery:
    """Apply a shared-channel byte budget exactly once per model interval.

    Flows are (product_id, sender, receiver, link_available). A hop may forward
    only the prefix already held at the START of the interval; this conservative
    sampled model cannot instantly forward a newly received hop in the same tick.
    The caller is the simulated transport, not the mother-ship truth reader.
    """
    def __init__(self, start_time=0.):
        if not math.isfinite(start_time) or start_time<0:
            raise ValueError('finite nonnegative start time required')
        self.products={}
        self.channel_time={}
        self.start_time=start_time
        self._epoch_time=None
        self._epoch_prefixes={}

    def produce(self,product_id,product):
        if product_id in self.products:
            raise ValueError('product already generated')
        self.products[product_id]=product

    def advance(self,channel,now_s,bytes_per_second,flows):
        """Legacy single-channel boundary; same-time calls share a snapshot."""
        return self.advance_all(now_s,{channel:(bytes_per_second,flows)})

    def advance_all(self,now_s,channels):
        """One shared step-start snapshot across all acoustic/RF channels.

        ``channels`` maps channel identity to (effective rate, ordered flows).
        A caller supplies all active channels once per communication tick.
        Rates apply to the elapsed interval; link changes split that interval.
        Duplicate same-time updates consume no additional capacity.
        """
        if not math.isfinite(now_s) or now_s<self.start_time:
            raise ValueError('invalid communication time')
        if self._epoch_time is not None and now_s<self._epoch_time:
            raise ValueError('communication time moved backwards')
        updates={name:(rate,tuple(flows)) for name,(rate,flows) in channels.items()}
        # Validate every channel before modifying any ledger or epoch.
        for name,(rate,flows) in updates.items():
            if not name or not math.isfinite(rate) or rate<0:
                raise ValueError('invalid channel time/rate')
            for key,source,destination,available in flows:
                if key not in self.products or not source or not destination or source==destination or type(available) is not bool:
                    raise ValueError('invalid delivery flow')
        if self._epoch_time!=now_s:
            self._epoch_prefixes={k:dict(p.received_prefix) for k,p in self.products.items()}
            self._epoch_time=now_s
        receipts=[]
        for channel,(rate,flows) in updates.items():
            before=self.channel_time.get(channel,self.start_time)
            budget=rate*(now_s-before)
            for key,source,destination,available in flows:
                product=self.products[key]
                if not available or product.generated_at>before:continue
                old=product.received_prefix.get(destination,0.)
                amount=min(budget,max(0.,self._epoch_prefixes.get(key,{}).get(source,0.)-old))
                if amount<=0:continue
                product.received_prefix[destination]=old+amount
                budget-=amount
                if destination==product.receiver and old+amount>=product.required_bytes and product.received_at is None:
                    product.received_at=now_s
                    receipts.append(key)
            self.channel_time[channel]=now_s
        return tuple(receipts)


def declared_delivery_channels(products,previous,states,obstacles=(),continuous=True):
    """The frozen sampled link model, shared by prediction and live transport.

    State values are (position, actual medium). The mother ship knows neither
    this input nor intermediate relay prefixes; these belong to the simulator.
    """
    def link(samples,source,destination,water):
        if source not in samples or destination not in samples:return False
        a,ma=samples[source];b,mb=samples[destination]
        if water:
            if {ma,mb}!={'WATER','SURFACE'}:return False
        elif ma not in ('AIR','SURFACE') or mb not in ('AIR','SURFACE'):return False
        return math.dist(a,b)<=(8. if water else 30.) and not any(o.blocks(a,b) for o in obstacles)
    acoustic=[];radio=[]
    for ident,product in products.items():
        if product.received_at is not None:continue
        source=product.producer
        if source=='mother':
            destination=product.receiver
            radio.append((ident,'mother',destination,continuous and
                link(previous,'mother',destination,False) and link(states,'mother',destination,False)))
            if destination!='usv':
                radio.append((ident,'mother','usv',continuous and
                    link(previous,'mother','usv',False) and link(states,'mother','usv',False)))
                radio.append((ident,'usv',destination,continuous and
                    link(previous,'usv',destination,False) and link(states,'usv',destination,False)))
                acoustic.append((ident,'usv',destination,continuous and
                    link(previous,'usv',destination,True) and link(states,'usv',destination,True)))
            continue
        if source!='usv':
            acoustic.append((ident,source,'usv',continuous and
                link(previous,source,'usv',True) and link(states,source,'usv',True)))
            # AIR/SURFACE observations use the same declared RF capacity to
            # reach a support USV. The old underwater-only hop made an AIR
            # product outside direct mother range permanently undeliverable.
            radio.append((ident,source,'usv',continuous and
                link(previous,source,'usv',False) and link(states,source,'usv',False)))
        for sender in dict.fromkeys((source,'usv')):
            radio.append((ident,sender,'mother',continuous and
                link(previous,sender,'mother',False) and link(states,sender,'mother',False)))
    return {'acoustic':(2*1024,acoustic),'radio':(32*1024,radio)}


@dataclass(frozen=True)
class ObservationSample:
    """One recorded state of one member during an observation window."""

    member_id: str
    time_s: float
    position: Vector3


@dataclass(frozen=True)
class ObstacleBox:
    centre: Vector3
    size: Vector3

    def blocks(self, start: Vector3, end: Vector3) -> bool:
        """Whether the segment start->end intersects the box (slab test)."""
        low = [self.centre[i] - self.size[i] / 2.0 for i in range(3)]
        high = [self.centre[i] + self.size[i] / 2.0 for i in range(3)]
        t0, t1 = 0.0, 1.0
        for i in range(3):
            direction = end[i] - start[i]
            if abs(direction) < 1e-12:
                if start[i] < low[i] or start[i] > high[i]:
                    return False
                continue
            a = (low[i] - start[i]) / direction
            b = (high[i] - start[i]) / direction
            if a > b:
                a, b = b, a
            t0 = max(t0, a)
            t1 = min(t1, b)
            if t0 > t1:
                return False
        return True


@dataclass(frozen=True)
class PointObservation:
    point_id: str
    observed: bool
    member_id: Optional[str]
    dwell_s: float
    reason: str


@dataclass
class CoverageResult:
    points: Dict[str, PointObservation] = field(default_factory=dict)
    delivered_point_ids: frozenset = frozenset()

    def observed_fraction(self, weights: Mapping[str, float]) -> float:
        total = sum(weights.values())
        if total <= 0:
            raise ValueError("weights must sum to a positive value")
        return sum(weight for point_id, weight in weights.items()
                   if self.points.get(point_id) and self.points[point_id].observed) / total

    def delivered_fraction(self, weights: Mapping[str, float]) -> float:
        """``C_delivered``: observed AND the result has been received.

        The invariant lives here, not only in the caller.  Recording a point as
        delivered cannot raise this fraction on its own: a receipt for something
        that was never observed is not a delivered observation.  Zero latency
        does not merge the two events either, it only makes the receipt immediate.
        """
        total = sum(weights.values())
        if total <= 0:
            raise ValueError("weights must sum to a positive value")
        return sum(weight for point_id, weight in weights.items()
                   if point_id in self.delivered_point_ids
                   and self.points.get(point_id) is not None
                   and self.points[point_id].observed) / total

    def delivered_point_observations(self, weights: Mapping[str, float]) -> Tuple[str, ...]:
        """Points that count towards ``C_delivered``, for reporting."""
        return tuple(sorted(point_id for point_id in weights
                            if point_id in self.delivered_point_ids
                            and self.points.get(point_id) is not None
                            and self.points[point_id].observed))

    def uncovered(self, weights: Mapping[str, float]) -> Tuple[str, ...]:
        return tuple(sorted(point_id for point_id in weights
                            if not (self.points.get(point_id)
                                    and self.points[point_id].observed)))

    def undelivered(self, weights: Mapping[str, float]) -> Tuple[str, ...]:
        return tuple(sorted(point_id for point_id in weights
                            if point_id not in self.delivered_point_ids))


def _visible(member: Vector3, point: Vector3, requirement,
            obstacles: Sequence[ObstacleBox]) -> bool:
    horizontal = math.dist((member[0], member[1], 0.0), (point[0], point[1], 0.0))
    if (horizontal > requirement.footprint_radius_m
            or abs(member[2] - point[2]) > requirement.max_distance_from_altitude_m):
        return False
    for box in obstacles:
        if box.blocks(member, point):
            return False
    return True


def evaluate_coverage(samples: Sequence[ObservationSample],
                      interest_points: Mapping[str, Vector3],
                      requirement,
                      obstacles: Sequence[ObstacleBox] = (),
                      *,
                      sample_timeout_s: float = 0.5) -> CoverageResult:
    """Binary geometric visibility, with the dwell enforced on one member.

    The dwell is not "how long any member was near": it is the longest continuous
    run during which a *single* member kept the point inside the footprint with an
    unblocked line of sight.  A break in the condition, or a gap longer than
    ``sample_timeout_s`` in that member's samples, ends the run and the count
    restarts, so three members contributing 0.4 s each cannot make 1.2 s.
    """
    if not math.isfinite(sample_timeout_s) or sample_timeout_s <= 0:
        raise ValueError("sample_timeout_s must be finite and positive")
    by_member: Dict[str, list] = {}
    for sample in samples:
        if (not math.isfinite(sample.time_s) or len(sample.position) != 3
                or not all(math.isfinite(x) for x in sample.position)):
            raise ValueError("observation samples must be finite")
        rows = by_member.setdefault(sample.member_id, [])
        if rows and sample.time_s <= rows[-1].time_s:
            raise ValueError("member sample times must strictly increase")
        rows.append(sample)

    result = CoverageResult()
    for point_id, point in interest_points.items():
        observed = False
        best_member = None
        best_dwell = 0.0
        reason = "no sample put the point in the footprint"
        for member_id, rows in by_member.items():
            run_start = None
            previous = None
            for sample in rows:
                seen = _visible(sample.position, point, requirement,
                               obstacles)
                if previous is not None and sample.time_s - previous.time_s > sample_timeout_s:
                    run_start = None          # stale: the run does not continue
                previous = sample
                if not seen:
                    run_start = None
                    continue
                if run_start is None:
                    run_start = sample.time_s
                dwell = sample.time_s - run_start
                if dwell + 1e-9 < requirement.min_dwell_s:
                    continue
                observed = True
                best_member, best_dwell = member_id, dwell
                reason = "geometric visibility and continuous dwell; image quality unverified"
                break
            if observed:
                break
        result.points[point_id] = PointObservation(
            point_id=point_id, observed=observed,
            member_id=best_member, dwell_s=best_dwell, reason=reason)
    return result


def record_delivery(result: CoverageResult, point_ids: Iterable[str]) -> None:
    """Record that the results for these points have been received.

    Separate from observation on purpose.  With an ideal link this is called
    immediately, but it is still a distinct event and is recorded as one.
    """
    result.delivered_point_ids = frozenset(result.delivered_point_ids) | set(point_ids)


class LocalObservationWindow:
    """Streaming local geometric sensor surrogate for one accepted action.

    Only the executing node calls this with completed model samples. It emits
    products, never receipt or task-success events. Conditions come from the
    loaded request, not from an arbitrary payload-quality score.
    """
    def __init__(self,request,point_ids,producer,goal_id,obstacles=(),sample_timeout_s=.25):
        if request is None:raise ValueError('observation requires a loaded request')
        if not producer or not goal_id:raise ValueError('observation needs execution identity')
        if not math.isfinite(sample_timeout_s) or sample_timeout_s<=0:raise ValueError('invalid sample timeout')
        points={p.point_id:(p.position,'WATER' if region.kind=='UNDERWATER' else 'AIR')
                for region in request.regions for p in region.interest_points}
        if len(set(point_ids))!=len(point_ids) or any(k not in points for k in point_ids):
            raise ValueError('unknown or repeated observation ID')
        self.request=request;self.points={k:points[k] for k in point_ids}
        self.producer=producer;self.goal_id=goal_id;self.obstacles=obstacles;self.timeout=sample_timeout_s
        self.previous=None;self.started={};self.emitted=set()

    def sample(self,model_time,position,mode,stamp,valid=True):
        if not math.isfinite(model_time) or not math.isfinite(stamp):
            raise ValueError('nonfinite observation time')
        if self.previous is not None and model_time<=self.previous:
            raise ValueError('observation model time must advance')
        if self.previous is not None and model_time-self.previous>self.timeout:self.started.clear()
        self.previous=model_time
        events=[]
        finite=len(position)==3 and all(math.isfinite(v) for v in position)
        for key,(point,required_mode) in self.points.items():
            if key in self.emitted:continue
            if not valid or not finite or mode!=required_mode or not _visible(position,point,self.request.requirement,self.obstacles):
                self.started.pop(key,None);continue
            begin=self.started.setdefault(key,model_time)
            dwell=model_time-begin
            if dwell+1e-9<self.request.requirement.min_dwell_s:continue
            self.emitted.add(key)
            events.append(dict(product_id=self.goal_id+':'+key,request_id=self.request.request_id,
                goal_id=self.goal_id,point_id=key,producer=self.producer,generated_at=stamp,observed=True,
                result=dict(model='GEOMETRIC_PROXY',dwell_s=dwell),required_bytes=32*1024))
        return tuple(events)

    def terminal_report(self,stamp):
        """Report the observed and missing IDs after a verified local terminal.

        This small notification is a result of the local observation window,
        not a synthetic business product. The mother must actually receive it
        before deciding whether a missing point needs a retest.
        """
        if not math.isfinite(stamp):raise ValueError('nonfinite observation terminal time')
        return dict(event_type='OBSERVATION_TERMINAL',product_id=self.goal_id+':terminal',
            request_id=self.request.request_id,goal_id=self.goal_id,producer=self.producer,
            point_ids=sorted(self.points),observed_ids=sorted(self.emitted),generated_at=stamp)


def predict_received_products(request,point_ids,producer,goal_id,traces,mother_position,obstacles,deadline):
    """Evaluate observations and finite receipt on supplied complete rollouts.

    Traces contain (relative model time, position, actual medium). Never freeze
    a platform beyond its supplied, checked terminal/wait path. The event's
    supplied identity is part of its exact encoded notification size; this is
    a nominal prediction, not a guarantee about a later different wire event.
    """
    import time
    if producer not in traces or not point_ids or not traces or not math.isfinite(deadline):
        raise ValueError('observation source, finite traces and shared deadline required')
    for rows in traces.values():
        if not rows or any(not math.isfinite(r[0]) or r[0]<0 or len(r[1])!=3 or
                          not all(math.isfinite(v) for v in r[1]) or r[2] not in ('AIR','SURFACE','WATER','TRANSITION') for r in rows):
            raise ValueError('invalid predicted physical trace')
        if any(a[0]>=b[0] for a,b in zip(rows,rows[1:])):raise ValueError('unordered predicted trace')
        if any(b[0]-a[0]>.25 for a,b in zip(rows,rows[1:])):
            return dict(status='UNKNOWN',reason='PREDICTED_STATE_SAMPLES_MISSING')
    window=LocalObservationWindow(request,point_ids,producer,goal_id,obstacles)
    generated=[]
    for stamp,pos,mode in traces[producer]:
        if time.monotonic()>=deadline:return dict(status='UNKNOWN',reason='PLANNING_BUDGET_EXHAUSTED')
        generated.extend(window.sample(stamp,pos,mode,stamp))
    if window.emitted!=set(point_ids):return dict(status='INFEASIBLE',reason='REQUIRED_OBSERVATION_NOT_COVERED')
    horizon=min(rows[-1][0] for rows in traces.values())
    result=predict_received_events(generated,traces,
        {member:((0.,horizon),) for member in traces},mother_position,obstacles,deadline)
    receipts={event['point_id']:result['received_at'][event['product_id']]
              for event in generated if event['product_id'] in result['received_at']}
    result.update(received_at=receipts,generated_events=generated)
    return result


def predict_received_events(generated,traces,communication_intervals,mother_position,obstacles,deadline):
    """Replay all nominal products once on shared channels in plan time.

    Geometry may cover idle/terminal motion for safety, but only accepted
    activity intervals enable communication. This does not grant a finished
    participant extra support time, nor clear products at a method boundary.
    Notification identity/encoding is nominal until actual GoalIDs exist.
    """
    import time
    import json
    from bisect import bisect_right
    ledger=FiniteDelivery(0.);events={};queue=sorted(generated,key=lambda e:(e['generated_at'],e['product_id']))
    if len({e['product_id'] for e in queue})!=len(queue):raise ValueError('duplicate predicted product')
    previous={};receipts={};notice_received=set();data_received=set()
    clocks={k:[r[0] for r in rows] for k,rows in traces.items()}
    # Adjacent accepted activities are continuous support. A gap between two
    # communication ticks is still a gap; endpoint tests must not hide it.
    active={}
    for member,intervals in communication_intervals.items():
        merged=[]
        for begin,end in sorted(intervals):
            if merged and begin<=merged[-1][1]+1e-9:merged[-1]=(merged[-1][0],max(end,merged[-1][1]))
            else:merged.append((begin,end))
        active[member]=merged
    horizon=max((end for intervals in communication_intervals.values() for _,end in intervals),default=0.)
    for tick in range(int(math.floor(horizon*10))+1):
        if time.monotonic()>=deadline:return dict(status='UNKNOWN',reason='PLANNING_BUDGET_EXHAUSTED',received_at=receipts)
        now=tick/10.
        while queue and queue[0]['generated_at']<=now:
            event=queue.pop(0);ident=event['product_id'];events[ident]=event
            encoded=json.dumps(event,allow_nan=False).encode('utf-8')
            ledger.produce('notice:'+ident,DeliveryProduct(event['producer'],'mother',4+len(encoded),event['generated_at'],True))
            if event.get('event_type')!='OBSERVATION_TERMINAL':
                ledger.produce('data:'+ident,DeliveryProduct(event['producer'],'mother',32768,event['generated_at'],True))
        states={'mother':(mother_position,'SURFACE')}
        for member,rows in traces.items():
            if not any(begin<=now<=end for begin,end in active.get(member,())):continue
            index=bisect_right(clocks[member],now)-1
            if index>=0 and now<=rows[-1][0]+1e-9 and now-rows[index][0]<=.25:
                states[member]=(rows[index][1],rows[index][2])
        continuous_previous={member:state for member,state in previous.items() if member=='mother' or
            any(begin<=now-.1+1e-9 and now<=end for begin,end in active.get(member,()))}
        for ident in ledger.advance_all(now,declared_delivery_channels(ledger.products,continuous_previous,states,obstacles)):
            kind,key=ident.split(':',1)
            (data_received if kind=='data' else notice_received).add(key)
            if key in notice_received and (events[key].get('event_type')=='OBSERVATION_TERMINAL' or key in data_received):
                receipts[key]=now
        previous=states
        if len(receipts)==len(generated):
            return dict(status='FEASIBLE',reason='NOMINAL_OBSERVATION_AND_FINITE_RECEIPT',
                        received_at=receipts,receipt_finish_s=max(receipts.values(),default=0.))
    return dict(status='INFEASIBLE',reason='RECEIPT_NOT_COMPLETED_WITHIN_CHECKED_COMMITMENTS',
                received_at=receipts)
