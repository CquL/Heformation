"""Existing request/planner join and one staged retest.

Formation geometry is diagnostic using Swarm's own normalized graph metric.
Motion success comes from the native Action result; no invented task corridor,
heading or similarity threshold is a business acceptance gate.
"""

from __future__ import annotations

import math
import itertools
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from .monitoring_request import (
    MonitoringRequest, ObservationRequirement, ObservationTask, SurveyRegion,
    InterestPoint, UNDERWATER, expand,
)
from .observation_coverage import CoverageResult

Vector3 = Tuple[float, float, float]


def assembly_member_order(positions,targets,minimum_center_distance):
    """Finite three-member approach alternatives, before group optimization.

    Reject straight approaches intersecting another member's occupied envelope.
    This is a geometric candidate filter, not a tracking-error safety guarantee;
    native Swarm planning and actual fleet safety still decide execution.
    """
    if set(positions)!=set(targets) or len(positions)!=3:
        raise ValueError('declared three-member assembly requires all members')
    if not math.isfinite(minimum_center_distance) or minimum_center_distance<=0:
        raise ValueError('minimum center distance must be finite and positive')
    if any(len(p)!=3 or not all(math.isfinite(v) for v in p) for p in list(positions.values())+list(targets.values())):
        raise ValueError('assembly geometry must use finite three-vectors')
    for order in itertools.permutations(sorted(positions)):
        predicted=dict(positions)
        valid=True
        for member in order:
            start,end=predicted[member],targets[member]
            delta=tuple(end[i]-start[i] for i in range(3))
            length2=sum(v*v for v in delta)
            for other,point in predicted.items():
                if other==member:continue
                fraction=(max(0.,min(1.,sum((point[i]-start[i])*delta[i] for i in range(3))/length2))
                          if length2 else 0.)
                closest=tuple(start[i]+fraction*delta[i] for i in range(3))
                if math.dist(closest,point)<minimum_center_distance:
                    valid=False
                    break
            if not valid:break
            predicted[member]=end
        if valid:return order
    raise ValueError('no ordered member approach found in the finite candidate set')


@dataclass(frozen=True)
class FormationPhase:
    """A declared group transfer; motion completion comes from native Action."""

    phase_id: str
    path_start: Vector3
    path_end: Vector3
    interest_point_ids: Tuple[str, ...]
    required_agent_count: int = 3


def formation_similarity(positions: Mapping[str, Vector3],
                         slots: Mapping[str, Vector3]) -> float:
    """SwarmGraph::calcMatrices/calcFNorm2, squared Frobenius difference.

    Source: upstream/Swarm-Formation/src/planner/swarm_graph/src/swarm_graph.cpp.
    Translation, rotation and uniform scale invariant. Dimensionless diagnostic;
    no calibrated business threshold has been established.
    """
    members = sorted(slots)
    if len(members) < 2 or set(positions) != set(slots):
        raise ValueError("all declared formation members must be present")

    def laplacian(points):
        values = [points[m] for m in members]
        if any(len(v) != 3 or not all(math.isfinite(x) for x in v) for v in values):
            raise ValueError("formation positions must be finite 3-vectors")
        adjacency = [[sum((x-y)**2 for x, y in zip(a, b)) for b in values] for a in values]
        degrees = [sum(row) for row in adjacency]
        if any(d <= 0 for d in degrees):
            raise ValueError("degenerate formation graph")
        return [[1.0 if i == j else -adjacency[i][j] / math.sqrt(degrees[i]*degrees[j])
                 for j in range(len(members))] for i in range(len(members))]

    actual, desired = laplacian(positions), laplacian(slots)
    return sum((a-b)**2 for row, wanted in zip(actual, desired) for a, b in zip(row, wanted))


def formation_interval_metrics(samples, slots, *, sample_timeout_s):
    """Aggregate every aligned sample; missing/invalid data are not zero error.

    Caller uses the Action's existing alignment/ledger and freshness limits.
    This reports measurements, never a shape/task PASS.
    """
    if len(samples) < 2:
        raise ValueError("at least two aligned formation samples are required")
    previous, errors = None, []
    for stamp, positions in samples:
        if not math.isfinite(stamp):
            raise ValueError("formation sample time must be finite")
        if previous is not None and not 0 < stamp - previous <= sample_timeout_s:
            raise ValueError("formation sample times are unordered or missing")
        errors.append(formation_similarity(positions, slots))
        previous = stamp
    return {"metric": "Swarm normalized Laplacian squared Frobenius difference",
            "sample_count": len(errors), "max_similarity_error": max(errors),
            "mean_similarity_error": sum(errors) / len(errors),
            "business_shape_verdict": "NOT_DEFINED"}


def request_centres(tasks: Sequence[ObservationTask]) -> Dict[str, Vector3]:
    """Named travel references, one per observation task.

    The executor planner travels between named references, so every observation
    position the request produced has to be nameable.  Naming them by task id
    keeps the plan and the request aligned without inventing extra references.
    """
    return {task.task_id: task.target for task in tasks}


def to_plan_tasks(tasks: Sequence[ObservationTask]):
    """Convert observation tasks into planner tasks.

    Imported lazily so this module stays usable without the planner package.
    """
    from mrta_python import Task

    return tuple(Task(task.task_id, task.required_capabilities, 1,
                      task.service_time_s, task.deadline_s, task.task_id,
                      allow_larger_unit=task.allow_larger_unit)
                 for task in tasks)


def request_native_methods(request,scene,executors,member_states,native_models,deadline,
                           return_sites=None,tasks_override=None):
    """Generate finite regional work/support methods, not a preselected tour.

    Each region remains a mandatory business Task. Single-anchor passes and
    forward/reverse declared-point tours compete; the real model/coverage
    query, not a greedy footprint pass, decides which satisfies every point.
    Unsupported domains/models remain without a method and therefore UNKNOWN
    in the existing planner. No required region is removed.
    """
    import time
    from mrta_python.models import NativeActionSpec,NativeSegmentSpec
    from mrta_python.executors import eligible_executors,PlanningBudgetExceeded
    from .monitoring_request import regional_requirements,UNDERWATER
    tasks=tuple(tasks_override) if tasks_override is not None else regional_requirements(request)
    if not tasks:raise ValueError('joint request needs at least one current business task')
    if not math.isfinite(deadline) or time.monotonic()>=deadline:
        raise PlanningBudgetExceeded('request method generation has no remaining budget')
    if request.return_required and not return_sites:
        raise ValueError('return destinations must be declared before method generation')
    if request.template_id=='OFFSHORE_JOINT':
        # The existing provider builds finite task methods directly from public
        # state and declared routes. It does not require plant/controller copies.
        return tasks,{}
    return_sites=return_sites or {}
    sites=[]
    for site in scene.get('communication_sites',()):
        position=tuple(site['position'])
        waits=site.get('departure_wait_candidates_s',(site.get('wait_before_s',0.),))
        for raw_wait in waits:
            wait=float(raw_wait)
            if (len(position)!=3 or not all(math.isfinite(v) for v in position) or
                    position[2]!=scene['surface_z_m'] or not math.isfinite(wait) or wait<0):
                raise ValueError('communication site must be a finite surface position and wait')
            entry=(position,wait)
            if entry not in sites:sites.append(entry)
    regions={r.region_id:r for r in request.regions};methods={}
    for task in tasks:
        region=regions[task.target_ref]
        if region.kind!=UNDERWATER or not request.delivery_required:continue
        for work in eligible_executors(executors,task):
            if len(work.physical_agent_ids)!=1:continue
            member=work.physical_agent_ids[0]
            if request.return_required and member not in return_sites:continue
            backend=member_states[member].get('native_backend',native_models.get(member))
            qn_uuv=(member=='uuv' and getattr(backend,'backend_id',None)=='PYTHON_QN_CLOSED_LOOP'
                    and member_states[member]['mode']=='WATER')
            if getattr(backend,'model',None)!='remus100' and not qn_uuv:continue
            start=tuple(member_states[member]['position'])
            home=tuple(return_sites[member]['position']) if request.return_required else start
            work_specs=[]
            point_positions=tuple(p.position for p in region.interest_points)
            routes=[(p,) for p in point_positions]
            if len(point_positions)>1:routes.extend((point_positions,tuple(reversed(point_positions))))
            for candidate in scene.get('water_route_candidates',{}).get(region.region_id,()):
                segments=tuple(NativeSegmentSpec('WATER_PATH',tuple(tuple(p) for p in raw['points']),
                    duration_s=float(raw.get('duration_s',0.)),
                    propulsion_effort=float(raw.get('propulsion_effort',0.)))
                    for raw in candidate['segments'])
                if (segments[0].points[0]!=start or
                        (request.return_required and segments[-1].points[-1]!=home)):
                    continue  # This declared route is not qualified from this member state.
                work_specs.append(NativeActionSpec(segments,'FIXED_REFERENCE' if qn_uuv else backend.terminal_behavior,
                    execution_timeout_s=float(candidate['execution_timeout_s']),
                    observation_ids=tuple(p.point_id for p in region.interest_points)))
            for route in (() if qn_uuv else routes):
                points=[start]
                for p in route:
                    if p!=points[-1]:points.append(p)
                if len(points)==1:points.append(points[0])  # qualified native coast/idle at an existing sample
                if request.return_required and points[-1]!=home:points.append(home)
                path=tuple(points)
                spec=NativeActionSpec((NativeSegmentSpec('WATER_PATH',path),),backend.terminal_behavior,
                    observation_ids=tuple(p.point_id for p in region.interest_points))
                if spec not in work_specs:work_specs.append(spec)
            choices=[]
            for support in executors:
                if len(support.physical_agent_ids)!=1 or member in support.physical_agent_ids:continue
                other=support.physical_agent_ids[0]
                if request.return_required and other not in return_sites:continue
                boat=member_states[other].get('native_backend',native_models.get(other))
                if getattr(boat,'model',None)!='otter' or 'SURFACE' not in support.capabilities:continue
                p=member_states[other]['position'];boat_start=(p[0],p[1],scene['surface_z_m'])
                for work_spec in work_specs:
                    for site,wait in sites:
                        if time.monotonic()>=deadline:raise PlanningBudgetExceeded('request method generation exceeded shared budget')
                        support_home=tuple(return_sites[other]['position']) if request.return_required else boat_start
                        support_path=(boat_start,site,support_home) if request.return_required else (boat_start,site)
                        if request.template_id=='OFFSHORE_JOINT':
                            support_segments=((NativeSegmentSpec('SURFACE_PATH',(boat_start,boat_start),duration_s=wait),)
                                              if wait else ())+(NativeSegmentSpec('SURFACE_PATH',(boat_start,site)),)
                            if request.return_required:
                                support_segments+=(NativeSegmentSpec('SURFACE_PATH',(site,support_home)),)
                        else:
                            support_segments=((NativeSegmentSpec('SURFACE_PATH',(boat_start,boat_start),duration_s=wait),)
                                              if wait else ())+(NativeSegmentSpec('SURFACE_PATH',support_path),)
                        support_spec=NativeActionSpec(support_segments,boat.terminal_behavior,
                            execution_timeout_s=(300. if request.template_id=='OFFSHORE_JOINT' else 180.)+wait)
                        choices.append({work:work_spec,support:support_spec})
            if choices:methods[(work.executor_id,task.task_id)]=tuple(choices)
    return tasks,methods


def build_request_executor_plan(request,scene,executors,provider,member_states,*,budget_s=10.,return_sites=None,
                                tasks_override=None,first_feasible=False):
    """Existing request/planner boundary with one budget including generation.

    The caller supplies the model snapshot. Unsupported regions stay mandatory;
    this function cannot return a plan for a silently truncated request.
    """
    import time
    from dataclasses import replace
    from mrta_python import build_executor_plan
    from mrta_python.executors import PlanningBudgetExceeded
    from .experiment_verdict import StaticSceneGeometry
    if not math.isfinite(budget_s) or budget_s<=0:raise ValueError('finite positive planning budget required')
    if request.return_required:
        if return_sites is None:
            return_sites=scene.get('return_sites')
        if not return_sites:
            raise ValueError('return destinations must be declared in the scenario or invocation')
        for site in return_sites.values():
            if (not isinstance(site,Mapping) or set(site)!={'position','radius_m'} or
                    len(site['position'])!=3 or not all(math.isfinite(v) for v in site['position']) or
                    not math.isfinite(site['radius_m']) or site['radius_m']<=0):
                raise ValueError('return site requires finite position and positive declared radius')
    deadline=time.monotonic()+budget_s
    tasks,methods=request_native_methods(request,scene,executors,member_states,provider.native_models,deadline,
                                         return_sites=return_sites,tasks_override=tasks_override)
    transition_sites=[];seen_sites=set()
    for site in scene.get('transition_sites',()):
        ident=str(site['id']);position=tuple(site['position'])
        air_stages=tuple(tuple(point) for point in site.get('air_stages',()))
        if (not ident or ident in seen_sites or len(position)!=3 or
                not all(math.isfinite(v) for v in position) or position[2]!=scene['surface_z_m'] or
                any(len(point)!=3 or not all(math.isfinite(v) for v in point) or
                    point[2]!=request.requirement.cruise_altitude_m for point in air_stages)):
            raise ValueError('transition site needs unique ID, surface position and AIR stages')
        seen_sites.add(ident);transition_sites.append((ident,position,air_stages))
    task_level=request.template_id=='OFFSHORE_JOINT'
    support_task=next((task.task_id for task in tasks if task.target_ref=='offshore_uuv'),
                      tasks[-1].task_id)
    provider=replace(provider,cooperative_routes=methods,observation_request=request,
        task_water_routes=dict(scene.get('water_route_candidates',{})),
        native_routes=dict(scene.get('air_route_via',{})) if task_level else provider.native_routes,
        task_support_task_id=support_task if task_level else '',
        scene_geometry=StaticSceneGeometry.from_mapping(scene),
        scene_resolution_m=float(scene['resolution']),
        air_support_units=tuple(unit for unit in executors if len(unit.physical_agent_ids)==1 and
            'SURFACE' in unit.capabilities and (task_level or getattr(member_states[unit.physical_agent_ids[0]].get(
                'native_backend',provider.native_models.get(unit.physical_agent_ids[0])),'model',None)=='otter')),
        air_support_sites=tuple(tuple(site['position']) for site in scene.get('communication_sites',())),
        task_support_sites=tuple(dict(site) for site in scene.get('communication_sites',())),
        planning_time_origin=time.time(),
        mother_position=tuple(scene.get('mother_ship_receiver_position',scene['mother_ship_position'])),
        return_sites=dict(return_sites) if request.return_required else {},
        air_units=tuple(unit for unit in executors if len(unit.physical_agent_ids)==1 and
                        'AIR' in unit.capabilities),
        transition_sites=tuple(transition_sites))
    remaining=deadline-time.monotonic()
    if remaining<=0:raise PlanningBudgetExceeded('request method generation exhausted planning budget')
    plan=build_executor_plan(executors,tasks,provider,initial_target_ref='start',budget_s=remaining,
        member_states=member_states,execution_candidates=provider.execution_candidates,
        hard_deadlines=True,first_feasible=first_feasible)
    return plan,tasks


def retest_tasks(request: MonitoringRequest,
                 tasks: Sequence[ObservationTask],
                 coverage: CoverageResult,
                 weights: Mapping[str, float],
                 *,
                 delivery_recorded: bool,
                 already_retested: bool) -> Tuple[ObservationTask, ...]:
    """Observation tasks for the points that were not covered, at most once.

    Release is staged on purpose: nothing is produced until the observation
    result has been *received*, because a retest scheduled before the data
    arrived would be a task waiting for an event a later task has to create.
    A second round is never generated: ``already_retested`` ends it.
    """
    if already_retested:
        return ()
    if not delivery_recorded:
        # The result of the first pass has not arrived, so there is nothing to
        # decide a retest from yet.  This is a wait, not a failure.
        return ()
    missing = set(coverage.uncovered(weights))
    if not missing:
        return ()
    by_region: Dict[str, List[InterestPoint]] = {}
    for region in request.regions:
        for point in region.interest_points:
            if point.point_id in missing:
                by_region.setdefault(region.region_id, []).append(point)
    retests: List[ObservationTask] = []
    for region_id in sorted(by_region):
        region = next(r for r in request.regions if r.region_id == region_id)
        narrowed = SurveyRegion(region.region_id, region.kind, region.corner_a,
                                region.corner_b, tuple(by_region[region_id]))
        if region.kind==UNDERWATER:
            # The legacy AIR set-cover expander skips underwater regions.
            # Keep one regional requirement so the joint method search can
            # select its viewpoint/pass and executor after the report arrives.
            extra=set(request.required_capabilities)-{'AIR','WATER','SURFACE'}
            retests.append(ObservationTask(
                task_id='retest-'+region_id+'-0',
                target=narrowed.interest_points[0].position,
                covers=tuple(point.point_id for point in narrowed.interest_points),
                region_id=region_id,service_time_s=request.service_time_s,
                deadline_s=request.deadline_s,
                required_capabilities=frozenset(extra|{'WATER'})))
            continue
        if request.template_id=='OFFSHORE_JOINT':
            retests.append(ObservationTask(
                task_id='retest-'+region_id+'-0',
                target=narrowed.interest_points[0].position,
                covers=tuple(point.point_id for point in narrowed.interest_points),
                region_id=region_id,service_time_s=request.service_time_s,
                deadline_s=request.deadline_s,
                required_capabilities=frozenset(extra|{'AIR'})))
            continue
        for index, task in enumerate(expand(MonitoringRequest(
                request_id="{}-retest".format(request.request_id),
                regions=(narrowed,), requirement=request.requirement,
                required_capabilities=request.required_capabilities,
                service_time_s=request.service_time_s,
                deadline_s=request.deadline_s,
                delivery_required=request.delivery_required))):
            retests.append(ObservationTask(
                task_id="retest-{}-{}".format(region_id, index),
                target=task.target, covers=task.covers, region_id=region_id,
                service_time_s=task.service_time_s, deadline_s=task.deadline_s,
                required_capabilities=task.required_capabilities))
    return tuple(retests)


def load_formation_phase(path: Path) -> Optional[FormationPhase]:
    """Read the optional group stage from the same request file."""
    import yaml

    raw = yaml.safe_load(Path(path).read_text())
    entry = (raw or {}).get("formation_phase")
    if entry is None:
        return None
    allowed = {"phase_id", "path_start", "path_end", "interest_point_ids", "required_agent_count"}
    if set(entry) - allowed:
        raise ValueError("unsupported formation fields: " + str(sorted(set(entry) - allowed)))
    request = load_request(path)
    point_ids = {p.point_id for r in request.regions for p in r.interest_points}
    if not set(entry.get("interest_point_ids", ())) <= point_ids:
        raise ValueError("formation phase references undefined interest points")
    if entry.get("interest_point_ids"):
        raise ValueError("formation observation is not configured; this phase supports transfer only")
    for key in ("path_start", "path_end"):
        if len(entry[key]) != 3 or not all(math.isfinite(x) for x in entry[key]):
            raise ValueError("formation endpoints must be finite 3-vectors")
    if entry.get("required_agent_count", 3) != 3:
        raise ValueError("this online formation phase requires three members")
    return FormationPhase(
        phase_id=str(entry["phase_id"]),
        path_start=tuple(entry["path_start"]),
        path_end=tuple(entry["path_end"]),
        interest_point_ids=tuple(entry["interest_point_ids"]),
        required_agent_count=int(entry.get("required_agent_count", 3)))


def load_request(path: Path) -> MonitoringRequest:
    """Read an operator request from YAML and validate it.

    Required geometry/service fields must be explicit. An absent or null
    deadline means no business deadline; never substitute a large number.
    """
    import yaml

    raw = yaml.safe_load(Path(path).read_text())
    if not isinstance(raw, dict):
        raise ValueError("a monitoring request must be a mapping")
    allowed={'request_id','template_id','requirement','regions','required_capabilities','service_time_s','deadline_s',
             'delivery_required','requires_underwater','requires_relay_delivery','return_required','formation_phase'}
    if set(raw)-allowed:
        raise ValueError('unsupported request fields: '+str(sorted(set(raw)-allowed)))
    for field in ("request_id", "requirement", "regions",
                  "service_time_s"):
        if field not in raw:
            raise ValueError("monitoring request is missing '{}'".format(field))
    requirement = ObservationRequirement(**raw["requirement"])
    regions = []
    for entry in raw["regions"]:
        points = tuple(InterestPoint(str(p["point_id"]), tuple(p["position"]),
                                     float(p.get("weight", 1.0)))
                       for p in entry.get("interest_points", ()))
        regions.append(SurveyRegion(str(entry["region_id"]), str(entry["kind"]).upper(),
                                    tuple(entry["corner_a"]), tuple(entry["corner_b"]),
                                    points))
    request = MonitoringRequest(
        request_id=str(raw["request_id"]), regions=tuple(regions),
        requirement=requirement,
        required_capabilities=frozenset(raw.get("required_capabilities", ())),
        service_time_s=float(raw["service_time_s"]),
        deadline_s=None if raw.get("deadline_s") is None else float(raw["deadline_s"]),
        delivery_required=bool(raw.get("delivery_required", True)),
        requires_underwater=bool(raw.get("requires_underwater", False)),
        requires_relay_delivery=bool(raw.get("requires_relay_delivery", False)),
        return_required=raw.get('return_required',False),
        template_id=str(raw.get('template_id','')))

    from .monitoring_request import validate_request
    validate_request(request)
    return request
