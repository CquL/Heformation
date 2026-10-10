"""Declared geometric monitoring surrogate; no calibrated camera/payload model.

Greedy footprint set cover is a project integration choice, not CARIC's planner.
Lengths and dwell times are scenario inputs, not literature-derived thresholds.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
import copy
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple
from .observation_coverage import communication_settings

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
    # ONLINE_MAPPING consumes local range measurements; coordinates below are
    # map accounting cells, never an operator-supplied flight path.
    execution_mode: str = 'POINT_OBSERVATION'
    work_items: Tuple[dict, ...] = ()


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


def mapping_cell_samples(center_xy,radius_m,cell_center,cell_size=1.,resolution=.25):
    """Declared global voxel centres, clipped to one report cell and the ROI.

    Request area weights and actual-map acceptance use this same discretisation.
    It is sampled area at this resolution, not exact continuous surface area.
    """
    x,y,z=cell_center;low=(x-cell_size/2,y-cell_size/2)
    high=(x+cell_size/2,y+cell_size/2);points=[]
    for ix in range(math.floor(low[0]/resolution),math.ceil(high[0]/resolution)):
        for iy in range(math.floor(low[1]/resolution),math.ceil(high[1]/resolution)):
            px=(ix+.5)*resolution;py=(iy+.5)*resolution
            if (low[0]<=px<high[0] and low[1]<=py<high[1] and
                    math.hypot(px-center_xy[0],py-center_xy[1])<=radius_m):
                points.append((px,py,z))
    return tuple(points)


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
    if raw.get('execution_mode')=='ONLINE_MAPPING':
        # Fixed accounting samples of the declared depth slices. This is not
        # a scan order or an instruction to fly through every cell centre.
        spacing=1.
        def cells(z,prefix):
            points=[];n=int(math.ceil(radius_m))
            for ix in range(-n,n):
                for iy in range(-n,n):
                    p=(center[0]+ix+.5,center[1]+iy+.5,z)
                    weight=len(mapping_cell_samples(center,radius_m,p))*.25**2
                    if not weight:continue
                    points.append(InterestPoint('{}_{:+d}_{:+d}'.format(prefix,ix,iy),p,weight))
            return tuple(points)
        air=cells(0.,'air_map');deep=cells(-2.,'deep_map')
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
    if raw.get('execution_mode')=='ONLINE_MAPPING':raw['request_id']+='-mapping'
    return raw


def circle_joint_mission_mappings(base_request, base_scene, center, radius_m):
    """Create one request/scene pair consumed by every existing endpoint."""
    from .experiment_verdict import StaticSceneGeometry
    request=circle_joint_request_mapping(base_request,center,radius_m)
    root=copy.deepcopy(base_scene);scene=root['scene'] if 'scene' in root else root
    scene.setdefault('communication',communication_settings(scene))
    geometry=StaticSceneGeometry.from_mapping(scene)
    if geometry is None:raise ValueError('circle selection needs declared static geometry')
    regions={entry['region_id']:entry for entry in request['regions']}
    if request.get('execution_mode')=='ONLINE_MAPPING':
        # Region/work intentions use only the user's geometry and deployment
        # policy. Hidden objects are not inspected to remove tasks or pick paths.
        cx,cy=(float(v) for v in center);r=float(radius_m)
        shallow=regions['offshore_aav_water'];sample=shallow['interest_points'][0]['position']
        shallow.update(shape='BOX',corner_a=list(sample),corner_b=list(sample),
                       center=list(sample),radius_m=None,coverage_resolution_m=None)
        mother=scene['mother_ship_receiver_position'];dx,dy=mother[0]-cx,mother[1]-cy
        norm=math.hypot(dx,dy) or 1.;ux,uy=dx/norm,dy/norm
        stage=(cx+(r+1.)*ux,cy+(r+1.)*uy,-2.)
        # The amphibious vehicle crosses the UUV's depth while entering water.
        # Reserve the existing two-body centre clearance in the horizontal
        # plane, plus one platform radius so numerical/tracking error is not
        # balanced exactly on the 0.5 m surface-clearance boundary.  Vertical
        # separation at the nominal endpoints cannot protect the conversion.
        platform_radius=.25;fleet_surface_clearance=.5
        conversion_offset=2.*platform_radius+fleet_surface_clearance+platform_radius
        usv_radius=1.1891593669479295
        scene['amphibious_return_altitude_m']=max(
            float(scene.get('amphibious_return_altitude_m',0.)),
            usv_radius+platform_radius+fleet_surface_clearance+platform_radius)
        scene.pop('air_return_lanes_m',None)
        scene['aav_return_policy']='SWARM_FORMATION'
        entry=(stage[0]+conversion_offset*ux,stage[1]+conversion_offset*uy)
        scene['transition_sites']=[dict(id='mapping_entry',position=[entry[0],entry[1],0.])]
        # Put the surface support abeam of the conversion column.  The previous
        # collinear site lay on the shore-to-entry approach and made a support
        # vessel block the amphibious AAV it was meant to serve.  The 3 m
        # lateral offset preserves the declared acoustic envelope while the
        # runner releases the low-altitude approach only after this site is
        # actually occupied.
        support=(entry[0]-3.*uy,entry[1]+3.*ux,0.)
        scene['communication_sites']=[dict(id='mapping_support',position=list(support),radius_m=2.,
            acoustic_contact_m=communication_settings(scene)['acoustic_range_m'],
            mother_contact_m=communication_settings(scene)['radio_range_m'],departure_wait_candidates_s=[0.])]
        scene['return_sites']['uuv']['staging_position']=list(stage)
        scene['online_mapping']=True
        scene['selected_monitoring_area']=dict(shape='CIRCLE',center=[cx,cy,0.],radius_m=r,
            coverage_resolution_m=1.,coverage_scope='DECLARED_DEPTH_SLICE_SAMPLES')
        scene['known_free_deployment']=[]
        for site in scene['return_sites'].values():
            position=site['position'];half=1.75
            low=[v-half for v in position];high=[v+half for v in position]
            # The launch/deployment region is a declared known prior, unlike the
            # remote task map. Refuse an inconsistent prior instead of carving
            # a hidden obstacle away. This is not mission coverage evidence.
            if any(all(low[i]<c[i]+s[i]/2 and high[i]>c[i]-s[i]/2 for i in range(3))
                   for _,_,c,s in geometry.objects):
                raise ValueError('deployment free-space prior overlaps a scene obstacle')
            scene['known_free_deployment'].append(dict(low=low,high=high))
        scene['observation_targets']=[]
        scene.pop('water_route_candidates',None);scene.pop('air_route_via',None)
        return request,root
    # Obstacles inside a selected region are holes in its free workspace, as
    # in obstacle-aware boustrophedon coverage planning.  Keep the business
    # circle intact and remove only witnesses a physical centre cannot occupy.
    raw_air=regions['offshore_air']['interest_points']
    raw_deep=regions['offshore_uuv']['interest_points']
    air=[entry for entry in raw_air if not geometry.violation(
        (entry['position'][0],entry['position'][1],request['requirement']['cruise_altitude_m']),.25)]
    deep=[entry for entry in raw_deep if not geometry.violation(tuple(entry['position']),.25)]
    if not air:raise ValueError('selected circle has no free AIR coverage witness')
    if not deep:raise ValueError('selected circle has no free deep-water coverage witness')
    regions['offshore_air']['interest_points']=air
    regions['offshore_uuv']['interest_points']=deep
    excluded_air=len(raw_air)-len(air);excluded_deep=len(raw_deep)-len(deep)
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
    acoustic=8.;radio=30.;depth=abs(stage[2])
    # Keep finite geometric margin; a boundary-equality contact is brittle to
    # integration and state timestamp differences.
    acoustic_xy=max(0.,math.sqrt(max(0.,acoustic*acoustic-depth*depth))-.5)
    dx,dy=mother[0]-stage[0],mother[1]-stage[1];distance=math.hypot(dx,dy)
    # The first-version task service requires the USV to meet the UUV; its
    # backhaul is deliberately abstracted once support is active.  Do not
    # reject a user region merely because one static point cannot also be in
    # direct RF range of the mother ship.
    lower=0.;upper=min(distance,acoustic_xy)
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
        coverage_resolution_m=regions['offshore_air']['coverage_resolution_m'],
        excluded_air_witnesses=excluded_air,excluded_deep_witnesses=excluded_deep)
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
    if request.execution_mode=='INSPECTION_CONTROL':
        from .inspection_work import INSPECTION_TEMPLATES,validate_work
        if not request.request_id or request.template_id not in INSPECTION_TEMPLATES or not request.work_items:
            raise ValueError('inspection request needs a supported template and mandatory work')
        ids=[w['work_id'] for w in request.work_items]
        if len(ids)!=len(set(ids)):raise ValueError('inspection work identities must be unique')
        for work in request.work_items:validate_work(work)
        return
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
            if region.shape=='CIRCLE' and request.execution_mode=='ONLINE_MAPPING':
                samples=mapping_cell_samples(region.center[:2],region.radius_m,point.position,
                    cell_size=region.coverage_resolution_m)
                if not samples or abs(point.weight-len(samples)*.25**2)>1e-8:
                    raise ValueError('mapping report cell must use its clipped measured-grid area')
            elif (region.shape=='CIRCLE' and
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
    if request.execution_mode=='INSPECTION_CONTROL':
        return tuple(Task(request.request_id+'::'+w['work_id'],frozenset({w['domain']}),1,
            0.,request.deadline_s,w['work_id']) for w in request.work_items)
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


def joint_mission_mappings(base_request,base_scene,selection):
    """One operator selection factory for first and in-session A/B/C batches."""
    import uuid
    from .inspection_work import layered_contour,offset_line,local_work,definition_version,effective_contour_offset,inspection_work_budget,inspection_motion_profile
    business=selection.get('business','A')
    if business=='A':
        base=copy.deepcopy(base_request);base['template_id']='OFFSHORE_JOINT'
        base['execution_mode']='ONLINE_MAPPING';base.pop('work_items',None)
        # The A input remains the original declared template, even after B/C.
        if not base.get('regions'):
            from pathlib import Path
            import yaml
            base=yaml.safe_load((Path(__file__).parents[2]/'config/monitoring_request_offshore.yaml').read_text())
        return circle_joint_mission_mappings(base,base_scene,selection['center'],selection['radius_m'])
    if business not in ('B','C'):raise ValueError('unknown mission business')
    root=copy.deepcopy(base_scene);scene=root.get('scene',root)
    scene.setdefault('communication',communication_settings(scene))
    models={m['id']:m for m in scene.get('world_models',())}
    chosen=selection.get('object_ids')
    if chosen is None:chosen=([m['id'] for m in models.values() if m['model']=='wind_turbine']
        if business=='B' else ['production_platform','seabed_pipeline'])
    parts=selection.get('parts')
    if parts is None:parts=['air','foundation'] if business=='B' else ['air','structure','pipeline']
    if not chosen or not parts:raise ValueError('请选择至少一个设施及必做部位')
    profile=scene.get('inspection_profiles',{}).get(business,{})
    if business=='B' and profile.get('rotor_state','STATIC_KNOWN')!='STATIC_KNOWN':
        raise ValueError('first-version wind inspection requires stationary known rotor pose')
    offset=float(profile.get('offset_m',1.2))
    layer_count=selection.get('layers',profile.get('layers',3))
    if (isinstance(layer_count,bool) or not isinstance(layer_count,(int,float)) or
            not math.isfinite(layer_count) or layer_count<1 or int(layer_count)!=layer_count or
            not math.isfinite(offset) or offset<=0):
        raise ValueError('inspection layers must be a positive integer and offset finite and positive')
    layers=int(layer_count)
    air_contour_offset=effective_contour_offset(offset,'AIR')
    water_contour_offset=effective_contour_offset(offset,'WATER')
    motion_profile=inspection_motion_profile(scene)
    works=[]
    def add(obj,part,domain,sections):
        row=dict(work_id=obj+':'+part,object_id=obj,part=part,domain=domain,
                 label=obj+' / '+part,sections=sections,geometry_version=definition_version(models[obj]),
                 position_tolerance_m=.5 if domain=='AIR' else .2,
                 speed_limit_mps=2.,nominal_speed_mps=motion_profile['air_nominal_speed_mps' if domain=='AIR' else 'uuv_water_speed_mps'],
                 motion_profile=copy.deepcopy(motion_profile),
                 heading_tolerance_rad=math.pi/4)
        row.update(execution_work_traversals=2,execution_search_margin_model_s=600.,
                   execution_wall_reserve_factor=2.,execution_wall_model_rate_floor=.5,
                   execution_wall_model_rate_fallback=1.)
        if domain=='WATER':
            # qn's original mixed-medium buoyancy moments use the source DCM
            # without a yaw rotation into body coordinates. Use its qualified
            # aligned vertical conversion method; sensing work keeps its own
            # tangent headings. The exit runout is navigation, never credit.
            row.update(transition_heading_rad=0.,transition_heading_tolerance_rad=.1,
                       transition_runout_m=2.,transition_roll_pitch_limit_rad=.05,
                       transition_angular_rate_limit_radps=.03)
        if any(section['kind']=='CONTOUR' for section in sections):
            row.update(requested_contour_offset_m=offset,
                       effective_contour_offset_m=effective_contour_offset(offset,domain))
        row['execution_geometry_budget']=inspection_work_budget(row,sections[0]['points'][0])
        row['version']=definition_version(row);works.append(row)
    def levels(low,high):
        return [low+(high-low)*n/max(1,layers-1) for n in range(layers)]
    for ident in chosen:
        if ident not in models:raise ValueError('unknown selected facility '+ident)
        model=models[ident];x,y,z=model['position']
        if business=='B' and model['model']=='wind_turbine':
            if 'air' in parts:
                # Exterior of declared component proxies, not an inferred free map.
                sections=[]
                def envelope(cx,cy,height,minimum):
                    components=[o for o in scene.get('objects',()) if o['id'].startswith(ident+'_')
                        and abs(o['center'][2]-height)<=o['size'][2]/2+.5]
                    return tuple(max([minimum[a]]+[abs(o['center'][a]-c)+o['size'][a]/2
                        for o in components]) for a,c in enumerate((cx,cy)))
                for height in levels(float(profile.get('min_air_altitude_m',2.2)),6.2):
                    hx,hy=envelope(x,y,height,(.3,.3))
                    sections+=layered_contour((x,y),(hx,hy),[height],air_contour_offset)
                # Static rotor proxies also occupy the hub plane. Do not
                # declare a nacelle contour through a selected rotor blade.
                sections+=layered_contour((x,y+.12),envelope(x,y+.12,7.25,(.31,.6)),[7.25],air_contour_offset)
                for angle in (0.,2*math.pi/3,4*math.pi/3):
                    axis=(math.sin(angle),0.,math.cos(angle))
                    for side in (-1.,1.):
                        points=[(x+axis[0]*v,y+side*(offset+1.),7.25+axis[2]*v)
                                for v in (.25,2.3)]
                        section=offset_line(points);section['focus']=(x,y-.73,7.25)
                        sections.append(section)
                add(ident,'air','AIR',sections)
            if 'foundation' in parts:
                add(ident,'foundation','WATER',layered_contour((x,y),(.54,.54),levels(-1.2,-4.8),water_contour_offset))
        elif business=='C' and model['model']=='production_platform':
            if 'air' in parts:
                add(ident,'air','AIR',layered_contour((x,y),(3.6,3.3),levels(2.2,5.6),air_contour_offset))
            if 'structure' in parts:
                add(ident,'structure','WATER',layered_contour((x,y),(3.3,2.7),levels(-1.2,-4.8),water_contour_offset))
        elif business=='C' and model['model']=='seabed_pipeline' and 'pipeline' in parts:
            points=model['points'];n=len(points)-1
            span=selection.get('pipeline_span',[0,n]);a,b=(int(v) for v in span)
            if not 0<=a<b<=n:raise ValueError('pipeline range must be a nonempty declared contiguous span')
            # Working side outside the platform footprint; configurable control
            # distance, not a claimed sonar footprint or a route through pipe.
            side=float(profile.get('pipeline_side_offset_m',3.))
            section=offset_line(points[a:b+1],side,offset)
            sections=[]
            local_index=selection.get('recheck_index')
            if local_index is not None:
                local_index=int(local_index)
                if not 1<=local_index<len(points):raise ValueError('unknown selected valve')
                local_position=offset_line(points,side,offset)['points'][local_index]
                add(ident,'valve_'+str(local_index)+'_recheck','WATER',[local_work(local_position,4.)])
            else:
                for first,last in zip(section['points'],section['points'][1:]):
                    sections.extend([dict(kind='LINE',points=[first,last]),local_work(last,4.)])
                add(ident,'pipeline','WATER',sections)
    if not works:raise ValueError('selection contains no declared required facility work')
    raw=copy.deepcopy(base_request)
    raw.update(request_id=business.lower()+'-'+uuid.uuid4().hex[:12],
        template_id='WIND_INSPECTION' if business=='B' else 'PLATFORM_PIPELINE_INSPECTION',
        execution_mode='INSPECTION_CONTROL',regions=[],work_items=works,return_required=True,
        requires_underwater=any(w['domain']=='WATER' for w in works),
        requires_relay_delivery=any(w['domain']=='WATER' or
            math.dist(w['sections'][-1]['points'][-1],scene.get('mother_ship_receiver_position',scene['mother_ship_position']))>
            communication_settings(scene)['radio_range_m'] for w in works))
    scene['online_mapping']=True;scene['aav_return_policy']='SWARM_FORMATION'
    scene['amphibious_return_altitude_m']=max(2.1891593669479295,float(scene.get('amphibious_return_altitude_m',0.)))
    # Services are intentions next to the work, not obstacle-free approach paths.
    sites=[];transitions=[]
    for work in works:
        first=work['sections'][0]['points'][0];last=work['sections'][-1]['points'][-1]
        if work['domain']=='WATER':
            transitions.append(dict(id=work['work_id'],position=[first[0],first[1],0.]))
            from .inspection_work import work_legs,inspection_exit_runout
            service_radius=1.5
            vessel_radius=1.1891593669479295
            separation=vessel_radius+.25+.5
            points=[point for item in works for section in item['sections'] for point in section['points']]
            for item in works:
                if item['domain']=='WATER':
                    entry=item['sections'][0]['points'][0];runout=inspection_exit_runout(item)
                    points.extend((tuple(entry[:2])+(0.,),tuple(runout[:2])+(0.,)))
            from .experiment_verdict import StaticSceneGeometry
            facility_prefixes=tuple(model['id']+'_' for model in models.values()
                if model['model'] in ('wind_turbine','production_platform','seabed_pipeline'))
            declared_objects=[obj for obj in scene.get('objects',())
                if obj['id'].startswith(facility_prefixes) or obj['kind']=='FORBIDDEN']
            facility_geometry=(StaticSceneGeometry.from_mapping(dict(scene,objects=declared_objects,
                obstacle_present=False)) if declared_objects else None)
            lower=[min(point[axis] for section in work['sections'] for point in section['points']) for axis in (0,1)]
            upper=[max(point[axis] for section in work['sections'] for point in section['points']) for axis in (0,1)]
            distance=separation+service_radius
            candidates=[(lower[0]-distance,last[1],0.),(upper[0]+distance,last[1],0.),
                        (last[0],lower[1]-distance,0.),(last[0],upper[1]+distance,0.)]
            candidates.extend((east,north,0.) for east in (lower[0]-distance,upper[0]+distance)
                              for north in (lower[1]-distance,upper[1]+distance))
            safe=[]
            for candidate in candidates:
                if math.dist(candidate,last)>8.:continue
                if facility_geometry and facility_geometry.violation(candidate,vessel_radius+service_radius):continue
                blocked=False
                for item in works:
                    for leg in work_legs(item):
                        origin,target=leg['start'],leg['end']
                        delta=tuple(b-a for a,b in zip(origin,target));square=sum(value*value for value in delta)
                        along=max(0.,min(1.,sum((value-start)*direction for value,start,direction in
                            zip(candidate,origin,delta))/square)) if square else 0.
                        closest=tuple(start+along*direction for start,direction in zip(origin,delta))
                        if math.dist(candidate,closest)<separation+service_radius:
                            blocked=True;break
                    if blocked:break
                if not blocked and all(math.dist(candidate,point)>=separation+service_radius for point in points):
                    safe.append(candidate)
            if not safe:raise ValueError('no safe shared support position for '+work['work_id'])
            home=scene['return_sites']['usv']['position']
            safe.sort(key=lambda point:math.dist(point,home))
            sites.append(dict(id=work['work_id'],position=list(safe[0]),position_candidates=[list(point) for point in safe],
                              radius_m=service_radius,acoustic_contact_m=communication_settings(scene)['acoustic_range_m'],
                              mother_contact_m=communication_settings(scene)['radio_range_m']))
    if not sites:
        p=works[0]['sections'][0]['points'][0]
        sites=[dict(id='air_support',position=[p[0],p[1]-4.,0.],radius_m=1.5,
            acoustic_contact_m=communication_settings(scene)['acoustic_range_m'],
            mother_contact_m=communication_settings(scene)['radio_range_m'])] if raw['requires_relay_delivery'] else []
    scene.update(communication_sites=sites,transition_sites=transitions,inspection_selection=copy.deepcopy({k:v for k,v in selection.items() if v is not None}),
                 observation_targets=[dict(id=w['work_id'],position=list(w['sections'][0]['points'][0]),domain=w['domain']) for w in works],
                 selected_monitoring_area=dict(center=list(selection.get('center',models[chosen[0]]['position'][:2])),
                                               radius_m=float(selection.get('radius_m',7.))))
    return raw,root
