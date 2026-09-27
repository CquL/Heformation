"""Declared geometric monitoring surrogate; no calibrated camera/payload model.

Greedy footprint set cover is a project integration choice, not CARIC's planner.
Lengths and dwell times are scenario inputs, not literature-derived thresholds.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
import copy
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

SURFACE = "SURFACE"
SHORELINE = "SHORELINE"
UNDERWATER = "UNDERWATER"
KINDS = (SURFACE, SHORELINE, UNDERWATER)

Vector3 = Tuple[float, float, float]


@dataclass(frozen=True)
class InterestPoint:
    point_id: str
    position: Vector3
    weight: float = 1.0


@dataclass(frozen=True)
class SurveyRegion:
    region_id: str
    kind: str
    corner_a: Vector3
    corner_b: Vector3
    interest_points: Tuple[InterestPoint, ...]
    shape: str = "BOX"
    center: Optional[Vector3] = None
    radius_m: Optional[float] = None
    coverage_resolution_m: Optional[float] = None


@dataclass(frozen=True)
class ObservationRequirement:
    """What counts as a valid observation.  Declared, not inferred."""

    footprint_radius_m: float
    min_dwell_s: float
    cruise_altitude_m: float
    max_distance_from_altitude_m: float = 3.0

    def __post_init__(self):
        for name in ("footprint_radius_m", "min_dwell_s", "max_distance_from_altitude_m"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name + " must be finite and positive")
        if not math.isfinite(self.cruise_altitude_m):
            raise ValueError("cruise_altitude_m must be finite")



@dataclass(frozen=True)
class MonitoringRequest:
    request_id: str
    regions: Tuple[SurveyRegion, ...]
    requirement: ObservationRequirement
    required_capabilities: frozenset
    service_time_s: float
    deadline_s: Optional[float] = None
    #: result must be received, not merely captured
    delivery_required: bool = True
    #: the request explicitly demands underwater work or a USV relay
    requires_underwater: bool = False
    requires_relay_delivery: bool = False
    #: declared mission policy: participating members reach scenario-declared
    #: return/exit sites before the request can finish
    return_required: bool = False
    # A single fixed joint-operation template; ordinary requests leave it unset.
    template_id: str = ''


def circle_sweep_points(center, radius_m, z, spacing_m, prefix):
    """Finite boustrophedon witnesses for a selected circular area.

    The structure follows line-sweep coverage planning (Huang, ICRA 2001) and
    the swath/route separation used by Fields2Cover (RA-L 2023).  These points
    remain a declared geometric coverage proxy; they are not sensor pixels.
    """
    center=tuple(float(v) for v in center)
    radius_m=float(radius_m);spacing_m=float(spacing_m);z=float(z)
    if (len(center)!=2 or not all(math.isfinite(v) for v in center) or
            not math.isfinite(radius_m) or radius_m<=0 or
            not math.isfinite(spacing_m) or spacing_m<=0 or not math.isfinite(z)):
        raise ValueError('circle sweep needs finite center, radius, spacing and depth')
    lane_count=max(2,int(math.ceil(2*radius_m/spacing_m))+1)
    points=[]
    for lane in range(lane_count):
        offset=-radius_m+2*radius_m*lane/(lane_count-1)
        half=math.sqrt(max(0.,radius_m*radius_m-offset*offset))
        count=max(1,int(math.ceil(2*half/spacing_m))+1)
        xs=([center[0]] if count==1 else
            [center[0]-half+2*half*i/(count-1) for i in range(count)])
        if lane%2:xs.reverse()
        points.extend((x,center[1]+offset,z) for x in xs)
    if len(points)>64:raise ValueError('selected circle produces more than 64 coverage witnesses')
    return tuple(InterestPoint('{}_{:02d}'.format(prefix,index+1),point,1.)
                 for index,point in enumerate(points))


def circle_joint_request_mapping(base, center, radius_m):
    """Instantiate the existing joint business template from one UI circle."""
    raw=copy.deepcopy(base)
    if raw.get('template_id')!='OFFSHORE_JOINT':
        raise ValueError('circle selection requires the OFFSHORE_JOINT template')
    requirement=ObservationRequirement(**raw['requirement'])
    center=tuple(float(v) for v in center);radius_m=float(radius_m)
    if len(center)!=2 or not all(math.isfinite(v) for v in center) or not 2.<=radius_m<=10.:
        raise ValueError('selected circle radius must be within 2..10 model metres')
    spacing=min(requirement.footprint_radius_m*1.25,radius_m)
    air=circle_sweep_points(center,radius_m,0.,spacing,'air_swath')
    deep=circle_sweep_points(center,radius_m,-2.,spacing,'deep_swath')
    shallow=InterestPoint('aav_water_sample',(center[0],center[1],-.6),1.)
    by_id={entry['region_id']:entry for entry in raw['regions']}
    if set(by_id)!={'offshore_air','offshore_aav_water','offshore_uuv'}:
        raise ValueError('joint template region identities changed')
    for region,points,z in ((by_id['offshore_air'],air,0.),
                            (by_id['offshore_aav_water'],(shallow,),-.6),
                            (by_id['offshore_uuv'],deep,-2.)):
        region.update(shape='CIRCLE',center=[center[0],center[1],z],radius_m=radius_m,
                      coverage_resolution_m=spacing,
                      corner_a=[center[0]-radius_m,center[1]-radius_m,z],
                      corner_b=[center[0]+radius_m,center[1]+radius_m,z],
                      interest_points=[dict(point_id=p.point_id,position=list(p.position),weight=p.weight)
                                       for p in points])
    raw['request_id']='circle-{:.2f}-{:.2f}-r{:.2f}'.format(center[0],center[1],radius_m)
    return raw


def circle_joint_mission_mappings(base_request, base_scene, center, radius_m):
    """Create one request/scene pair consumed by every existing endpoint."""
    from .experiment_verdict import StaticSceneGeometry
    request=circle_joint_request_mapping(base_request,center,radius_m)
    root=copy.deepcopy(base_scene);scene=root['scene'] if 'scene' in root else root
    geometry=StaticSceneGeometry.from_mapping(scene)
    if geometry is None:raise ValueError('circle selection needs declared static geometry')
    regions={entry['region_id']:entry for entry in request['regions']}
    air=regions['offshore_air']['interest_points']
    deep=regions['offshore_uuv']['interest_points']
    for entry in air:
        position=(entry['position'][0],entry['position'][1],request['requirement']['cruise_altitude_m'])
        reason=geometry.violation(position,.25)
        if reason:raise ValueError('selected AIR coverage intersects declared obstacle: '+reason)
    for entry in deep:
        reason=geometry.violation(tuple(entry['position']),.25)
        if reason:raise ValueError('selected deep coverage intersects declared obstacle: '+reason)
    cx,cy=(float(v) for v in center);radius_m=float(radius_m)
    air_xy=tuple((entry['position'][0],entry['position'][1]) for entry in air)
    # The airborne survey and the amphibious approach run concurrently.  An
    # entry on an AIR coverage witness gives the Swarm hard-clearance layer no
    # feasible terminal state.  Put the first entry ring outside the business
    # area by the declared two-body envelope plus one platform radius; after
    # entering WATER, qn plans from that entry to the selected sample.
    required_aav_center_separation=2*.25+.5
    outer_radius=radius_m+required_aav_center_separation+.25
    candidates=[(cx+distance*math.cos(angle),cy+distance*math.sin(angle))
        for distance in (outer_radius,.7*radius_m,.4*radius_m)
        for angle in tuple(2*math.pi*i/12 for i in range(12))]
    transitions=[]
    for x,y in candidates:
        if (not geometry.violation((x,y,0.),.25) and
                not geometry.violation((x,y,-.6),.25) and
                min(math.hypot(x-a,y-b) for a,b in air_xy)>=required_aav_center_separation):
            if all(math.hypot(x-a,y-b)>.5 for a,b in transitions):transitions.append((x,y))
    if not transitions:raise ValueError('selected circle has no qualified entry/exit point')
    shallow=regions['offshore_aav_water']['interest_points'][0]
    shallow_position=(cx,cy) if (not geometry.violation((cx,cy,-.6),.25)) else transitions[0]
    shallow['position']=[shallow_position[0],shallow_position[1],-.6]
    regions['offshore_aav_water'].update(center=[shallow_position[0],shallow_position[1],-.6],
        corner_a=[shallow_position[0],shallow_position[1],-.6],
        corner_b=[shallow_position[0],shallow_position[1],-.6],
        shape='BOX',radius_m=None,coverage_resolution_m=None)
    stage=tuple(deep[-1]['position'])
    mother=tuple(scene.get('mother_ship_receiver_position',scene['mother_ship_position']))
    acoustic=8.;radio=30.;depth=abs(stage[2]);receiver_height=abs(mother[2])
    # Keep finite geometric margin; a boundary-equality contact is brittle to
    # integration and state timestamp differences.
    acoustic_xy=max(0.,math.sqrt(max(0.,acoustic*acoustic-depth*depth))-.5)
    radio_xy=max(0.,math.sqrt(max(0.,radio*radio-receiver_height*receiver_height))-1.)
    dx,dy=mother[0]-stage[0],mother[1]-stage[1];distance=math.hypot(dx,dy)
    lower=max(0.,distance-radio_xy);upper=min(distance,acoustic_xy)
    if lower>upper+1e-9:raise ValueError('selected circle is outside one-USV contact geometry')
    start=tuple(scene['return_sites']['usv']['position'])
    support_candidates=[]
    for index in range(17):
        along=lower+(upper-lower)*index/16 if upper>lower else lower
        x=stage[0]+(dx/distance*along if distance else 0.)
        y=stage[1]+(dy/distance*along if distance else 0.)
        position=(x,y,0.)
        if not geometry.violation(position,1.1891593669479295):
            support_candidates.append((math.dist(start,position),position))
    if not support_candidates:raise ValueError('selected circle has no free USV contact point')
    support=min(support_candidates)[1]
    scene['selected_monitoring_area']=dict(shape='CIRCLE',center=[cx,cy,0.],radius_m=radius_m,
                                           coverage_resolution_m=regions['offshore_air']['coverage_resolution_m'])
    scene['observation_targets']=[dict(id=point['point_id'],position=[point['position'][0],point['position'][1],
        request['requirement']['cruise_altitude_m']],domain='AIR') for point in air]+[
        dict(id=shallow['point_id'],position=shallow['position'],domain='WATER')]+[
        dict(id=point['point_id'],position=point['position'],domain='WATER') for point in deep]
    scene['transition_sites']=[dict(id='selected_entry_'+str(index+1),position=[point[0],point[1],0.])
                               for index,point in enumerate(transitions[:8])]
    scene['communication_sites']=[dict(id='selected_support',position=list(support),radius_m=2.,
        acoustic_contact_m=acoustic,mother_contact_m=radio,
        departure_wait_candidates_s=[0.,30.,60.])]
    scene['return_sites']['uuv']['staging_position']=list(stage)
    scene.pop('water_route_candidates',None);scene.pop('air_route_via',None)
    return request,root


@dataclass(frozen=True)
class ObservationTask:
    """One dispatchable observation: a position that covers a set of points."""

    task_id: str
    target: Vector3
    covers: Tuple[str, ...]
    region_id: str
    service_time_s: float
    deadline_s: Optional[float]
    required_capabilities: frozenset
    #: observation tasks never want a larger unit unless they say so
    allow_larger_unit: bool = False


class UnsupportedRequirement(ValueError):
    """The request asks for something no configured platform can do online."""


def _distance(a: Vector3, b: Vector3) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def unsupported_reasons(request: MonitoringRequest,
                        online_capabilities: Iterable[str]) -> Tuple[str, ...]:
    """Reasons this request cannot be fully executed online, if any."""
    available = {str(value).upper() for value in online_capabilities}
    reasons: List[str] = []
    if request.requires_underwater or any(r.kind == UNDERWATER for r in request.regions):
        reasons.append("UNDERWATER observation has no supported online expansion/execution adapter")
    if request.requires_relay_delivery:
        reasons.append("delivery via a SURFACE relay has no supported online delivery adapter")
    for region in request.regions:
        if region.kind not in KINDS:
            reasons.append("unknown region kind {} for {}".format(region.kind, region.region_id))
            continue
        if region.kind == UNDERWATER and UNDERWATER not in available:
            continue  # already reported once
        if region.kind in (SURFACE, SHORELINE) and "AIR" not in available:
            reasons.append(
                "region {} needs AIR observation but no AIR endpoint is configured".format(
                    region.region_id))
    return tuple(reasons)


def _observable_from(point: InterestPoint, position: Vector3,
                     requirement: ObservationRequirement) -> bool:
    """Whether a position is within the declared footprint of a point.

    The footprint is a horizontal disc at the fixed cruise altitude: the plan
    states this explicitly, so vertical range is checked separately rather than
    being folded into a 3-D distance.
    """
    horizontal = _distance((position[0], position[1], 0.0),
                           (point.position[0], point.position[1], 0.0))
    vertical = abs(point.position[2] - position[2])
    return (horizontal <= requirement.footprint_radius_m
            and vertical <= requirement.max_distance_from_altitude_m)


def observation_position(anchor: Vector3, requirement: ObservationRequirement) -> Vector3:
    """Where the vehicle actually goes to look at ``anchor``.

    The vehicle flies at the declared cruise altitude and looks down, so an
    observation position is vertically above the interest point rather than at
    the point itself.  This matters: an interest point below the observation band
    then has no flyable position above it, which is exactly the case the
    expansion must refuse instead of pretending to cover.
    """
    return (anchor[0], anchor[1], requirement.cruise_altitude_m)


def observation_candidates(region: SurveyRegion,
                           requirement: ObservationRequirement) -> Dict[str, Tuple[str, ...]]:
    """Observation positions worth flying, and the points each would cover.

    Each interest point contributes one observation position - directly above it
    at the cruise altitude - and that position may cover neighbouring points when
    the footprint reaches.  Nothing is invented between the declared points, so
    every waypoint the system flies is one the request asked about.
    """
    candidates: Dict[str, Tuple[str, ...]] = {}
    for anchor in region.interest_points:
        position = observation_position(anchor.position, requirement)
        covered = tuple(point.point_id for point in region.interest_points
                        if _observable_from(point, position, requirement))
        candidates[anchor.point_id] = covered
    return candidates


def expand(request: MonitoringRequest,
           online_capabilities: Iterable[str] = ("AIR",)) -> Tuple[ObservationTask, ...]:
    """Turn a request into observation tasks, or refuse it.

    Raises UnsupportedRequirement when part of the request cannot be executed by
    the configured platforms: it must be reported, not quietly omitted, because a
    request with an unexecutable part cannot be reported as complete.
    """
    validate_request(request)
    reasons = unsupported_reasons(request, online_capabilities)
    if reasons:
        raise UnsupportedRequirement("; ".join(reasons))
    tasks: List[ObservationTask] = []
    for region in request.regions:
        if region.kind == UNDERWATER:
            # Reachable only when an UNDERWATER endpoint exists, which the
            # reasons check above has already established for this version.
            continue
        remaining = {point.point_id: point for point in region.interest_points}
        candidates = observation_candidates(region, request.requirement)
        while remaining:
            # Greedy set cover: take the candidate that covers the most points
            # still unobserved, ties broken by id so the expansion is stable.
            best = None
            for point_id in sorted(candidates):
                covered = set(candidates[point_id]) & set(remaining)
                if not covered:
                    continue
                if best is None or len(covered) > len(best[1]):
                    best = (point_id, covered)
            if best is None:
                raise UnsupportedRequirement(
                    "region {} has interest points that no declared footprint can "
                    "cover: {}".format(region.region_id, sorted(remaining)))
            anchor_id, covered = best
            anchor = next(p for p in region.interest_points if p.point_id == anchor_id)
            tasks.append(ObservationTask(
                task_id="{}-obs{}".format(region.region_id, len(tasks)),
                target=observation_position(anchor.position, request.requirement),
                covers=tuple(sorted(covered)),
                region_id=region.region_id,
                service_time_s=request.service_time_s,
                deadline_s=request.deadline_s,
                required_capabilities=frozenset(request.required_capabilities or ('AIR',)),
            ))
            for point_id in covered:
                remaining.pop(point_id, None)
    if not tasks:
        raise UnsupportedRequirement("the request expands to no observable task")
    return tuple(tasks)


def validate_request(request: MonitoringRequest) -> None:
    """Reject unusable geometry/timing before any dispatch or division."""
    if not request.request_id or not request.regions:
        raise ValueError("request id and regions must be nonempty")
    if (not math.isfinite(request.service_time_s)
            or request.service_time_s < request.requirement.min_dwell_s):
        raise ValueError("service_time_s must be finite and at least min_dwell_s")
    if request.deadline_s is not None and (not math.isfinite(request.deadline_s) or request.deadline_s <= 0):
        raise ValueError("deadline_s must be absent or finite and positive")
    if type(request.return_required) is not bool:
        raise ValueError('return_required must be boolean')
    region_ids, point_ids = set(), set()
    for region in request.regions:
        if not region.region_id or region.region_id in region_ids or not region.interest_points:
            raise ValueError("region ids must be unique and regions nonempty")
        region_ids.add(region.region_id)
        if region.shape not in ('BOX','CIRCLE'):
            raise ValueError('region shape must be BOX or CIRCLE')
        if region.shape=='CIRCLE':
            if (region.center is None or len(region.center)!=3 or
                    not all(math.isfinite(v) for v in region.center) or
                    region.radius_m is None or not math.isfinite(region.radius_m) or region.radius_m<=0 or
                    region.coverage_resolution_m is None or
                    not math.isfinite(region.coverage_resolution_m) or region.coverage_resolution_m<=0):
                raise ValueError('circle region needs finite center, radius and coverage resolution')
        for vector in (region.corner_a, region.corner_b) + tuple(p.position for p in region.interest_points):
            if len(vector) != 3 or not all(math.isfinite(x) for x in vector):
                raise ValueError("coordinates must be finite 3-vectors")
        for point in region.interest_points:
            if not point.point_id or point.point_id in point_ids:
                raise ValueError("point ids must be globally unique")
            point_ids.add(point.point_id)
            if not math.isfinite(point.weight) or point.weight <= 0:
                raise ValueError("point weights must be finite and positive")
            if (region.shape=='CIRCLE' and
                    math.hypot(point.position[0]-region.center[0],
                               point.position[1]-region.center[1])>region.radius_m+1e-8):
                raise ValueError('circle coverage witness lies outside its region')


def regional_requirements(request):
    """Business requirements without selecting a viewpoint or physical robot.

    The legacy expand() entry retains its old AIR demonstration. This entry
    keeps every region and every point mandatory for complete-method planning.
    Actual method duration replaces the old scalar service estimate.
    """
    from mrta_python.models import Task
    validate_request(request)
    extra=set(request.required_capabilities)-{'AIR','WATER','SURFACE'}
    if request.template_id=='OFFSHORE_JOINT':
        roles={'offshore_air':('SURFACE',frozenset({'AIR','AAV'})),
               'offshore_aav_water':('UNDERWATER',frozenset({'WATER','AAV'})),
               'offshore_uuv':('UNDERWATER',frozenset({'WATER','UUV'}))}
        if (len(request.regions)!=len(roles) or
                any(r.region_id not in roles or r.kind!=roles[r.region_id][0]
                    for r in request.regions)):
            raise ValueError('OFFSHORE_JOINT requires AIR, AAV WATER and UUV WATER regions')
        return tuple(Task(request.request_id+'::'+r.region_id,roles[r.region_id][1],1,
            0.,request.deadline_s,r.region_id) for r in request.regions)
    if request.template_id:
        raise ValueError('unknown monitoring template: '+request.template_id)
    tasks=[]
    for region in request.regions:
        if region.kind not in KINDS:raise ValueError('unknown region kind: '+region.kind)
        domain='WATER' if region.kind==UNDERWATER else 'AIR'
        tasks.append(Task(request.request_id+'::'+region.region_id,frozenset(extra|{domain}),1,
            0.,request.deadline_s,region.region_id,allow_larger_unit=domain=='AIR'))
    return tuple(tasks)
