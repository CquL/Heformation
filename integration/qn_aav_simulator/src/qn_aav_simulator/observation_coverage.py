"""Geometric visibility and continuous dwell on actual samples, then receipt.

This binary surrogate does not establish image quality or payload validity.
No exposure, blur, resolution score or arbitrary positive-score acceptance.
"""

from __future__ import annotations

import math
import heapq
import time
import json
from dataclasses import dataclass, field
from typing import Dict, Iterable, Mapping, Optional, Sequence, Tuple

Vector3 = Tuple[float, float, float]


def local_peer_snapshot(histories, radii, own_member, stamp, position, own_radius,
                        horizon_s, own_speed_bound, speed_bounds, clearance_m=.5,
                        freshness_s=.25):
    """Measured peers at one query time; stale exclusion needs a motion bound."""
    states=[]
    for member,history in histories.items():
        if member==own_member:continue
        radius=radii.get(member)
        if radius is None or not math.isfinite(radius) or radius<=0:
            raise ValueError('PEER_GEOMETRY_UNKNOWN:'+member)
        rows=tuple(row for row in history if row.stamp<=stamp)
        if not rows:raise ValueError('PEER_STATE_UNKNOWN:'+member)
        sample=rows[-1];age=stamp-sample.stamp
        if age>freshness_s:
            bound=speed_bounds.get(member)
            if (bound is not None and math.isfinite(bound) and bound>0 and
                    math.dist(position,sample.position)>own_radius+radius+clearance_m+
                    own_speed_bound*horizon_s+bound*(age+horizon_s)):
                continue
            raise ValueError('PEER_STATE_STALE:'+member)
        states.append(dict(member=member,position=tuple(
            value+age*speed for value,speed in zip(sample.position,sample.velocity)),
            velocity=tuple(sample.velocity),radius_m=radius,stamp_s=sample.stamp,
            query_stamp_s=stamp))
    return tuple(states)


def peer_segment_clear(start,end,radius,peers,start_s=0.,end_s=0.,clearance_m=.5):
    """Check synchronous short linear motions, using measured world velocities."""
    for peer in peers:
        first=tuple(value+speed*start_s for value,speed in zip(peer['position'],peer['velocity']))
        last=tuple(value+speed*end_s for value,speed in zip(peer['position'],peer['velocity']))
        relative=tuple(value-other for value,other in zip(start,first))
        change=tuple((value-origin)-(other-before) for origin,value,before,other in zip(start,end,first,last))
        square=sum(value*value for value in change)
        closest=max(0.,min(1.,-sum(value*delta for value,delta in zip(relative,change))/square)) if square else 0.
        distance=sum((value+closest*delta)**2 for value,delta in zip(relative,change))
        if distance<(radius+peer['radius_m']+clearance_m)**2:return False
    return True


def segment_box_distance_sq(start,end,low,high):
    """Euclidean segment-to-voxel distance for the declared spherical hull.

    Squared box distance is quadratic between its finite face crossings.
    Minimize those pieces instead of enlarging the hull sphere to a cube.
    """
    delta=tuple(b-a for a,b in zip(start,end));cuts=[0.,1.]
    for a,d,lo,hi in zip(start,delta,low,high):
        if abs(d)>1e-12:
            cuts.extend(t for t in ((lo-a)/d,(hi-a)/d) if 0.<t<1.)
    cuts=sorted(set(cuts));best=math.inf
    for left,right in zip(cuts,cuts[1:]):
        middle=(left+right)/2.;quadratic=linear=0.
        for a,d,lo,hi in zip(start,delta,low,high):
            value=a+middle*d
            p,q=(-d,lo-a) if value<lo else (d,a-hi) if value>hi else (0.,0.)
            quadratic+=p*p;linear+=p*q
        t=max(left,min(right,-linear/quadratic)) if quadratic else middle
        distance=sum(max(lo-(a+t*d),0.,a+t*d-hi)**2 for a,d,lo,hi in zip(start,delta,low,high))
        best=min(best,distance)
        if best==0.:break
    return best


class LocalSurveyMap:
    """Sparse, measured local map used by the existing region action.

    Only first-return sensor rays in world coordinates enter this map: the
    scene geometry and desired trajectory are not inputs. Ray interiors are
    known free, measured end hits are occupied, and everything else is unknown
    (OctoMap occupancy semantics). Occupancy is persistent for this static
    scene; a later miss cannot erase a previous obstacle. This deliberately is
    a geometric, voxel-resolution proxy, not SLAM or complete 3-D reconstruction.

    The small observed-space search follows the frontier guidance used in
    FUEL/FALCON, without importing their allocation or trajectory frameworks.
    It proposes a safe short target; native motion/control still executes it.
    """

    def __init__(self, resolution_m=.25, *, spherical_hull=True):
        if not math.isfinite(resolution_m) or resolution_m <= 0:
            raise ValueError('survey resolution must be finite and positive')
        self.resolution_m = float(resolution_m)
        self.spherical_hull=bool(spherical_hull)
        self.free = set()
        self.occupied = set()
        self._hits = {}
        self._visits = {}
        self._last_origin = None
        self.stamp = None
        self._measured = set()
        self.measured_modes = {}
        self.observation_epoch_s = 0.
        self._forbidden_boxes = []
        self._free_bounding_boxes = set()
        self._nonfree_bounding_boxes = {}
        self.peer_states=()
        self.peer_speed_mps=1.
        self.peer_clearance_m=.5
        self.peer_body_radius_m=None

    def navigation_snapshot(self):
        """Freeze the measured map without recursively copying immutable keys.

        Voxel keys, hit points and frozen ObstacleBox values are immutable.
        Navigation copies only its occupancy, visit costs and geometry cache.
        Observation products stay in the live map and are never consumed by
        next_target/segment_clear or the private native motion query.
        """
        snapshot=LocalSurveyMap(self.resolution_m,spherical_hull=self.spherical_hull)
        snapshot.free=self.free.copy()
        snapshot.occupied=self.occupied.copy()
        snapshot._visits=self._visits.copy()
        snapshot._last_origin=self._last_origin
        snapshot.stamp=self.stamp
        snapshot.observation_epoch_s=self.observation_epoch_s
        snapshot._forbidden_boxes=self._forbidden_boxes.copy()
        snapshot._free_bounding_boxes=self._free_bounding_boxes.copy()
        snapshot._nonfree_bounding_boxes=self._nonfree_bounding_boxes.copy()
        snapshot.peer_states=self.peer_states
        snapshot.peer_speed_mps=self.peer_speed_mps
        snapshot.peer_clearance_m=self.peer_clearance_m
        snapshot.peer_body_radius_m=self.peer_body_radius_m
        return snapshot

    def motion_clear(self,start,end,radius,start_s=0.,end_s=None):
        if end_s is None:end_s=start_s+math.dist(start,end)/max(self.peer_speed_mps,1e-6)
        body_radius=radius if self.peer_body_radius_m is None else self.peer_body_radius_m
        return (self.segment_clear(start,end,radius) and
            peer_segment_clear(start,end,body_radius,
                self.peer_states,start_s,end_s,self.peer_clearance_m))

    def begin_observation_epoch(self, stamp):
        """Retain navigation knowledge, but require fresh sensing for a new job.

        A repeated region is a new monitoring request, not permission to emit
        the preceding request's products again. Delayed older scans may still
        refine the navigation map but cannot fulfil this observation epoch.
        """
        if not math.isfinite(stamp) or stamp < self.observation_epoch_s:
            raise ValueError('observation epoch must be finite and monotone')
        self.observation_epoch_s = float(stamp)
        self._measured.clear()
        self.measured_modes.clear()

    def declare_forbidden_box(self, low, high):
        """Known navigation policy, not a sensed solid or an occluding object."""
        low, high = self._point(low), self._point(high)
        if any(a >= b for a, b in zip(low, high)):
            raise ValueError('forbidden policy needs ordered box corners')
        self._free_bounding_boxes.clear()
        self._nonfree_bounding_boxes.clear()
        self._forbidden_boxes.append(ObstacleBox(
            tuple((a + b) / 2. for a, b in zip(low, high)),
            tuple(b - a for a, b in zip(low, high))))

    def declare_free_box(self, low, high):
        """Explicit deployment prior, for navigation only; not a new scan.

        Only complete voxels inside the declared already-surveyed launch box
        are known free. A declaration never erases measured occupied cells and
        cannot manufacture this request's mapping products.
        """
        low, high = self._point(low), self._point(high)
        if any(a >= b for a, b in zip(low, high)):
            raise ValueError('deployment prior needs ordered box corners')
        self._free_bounding_boxes.clear()
        self._nonfree_bounding_boxes.clear()
        lo = [math.ceil(v / self.resolution_m) for v in low]
        hi = [math.floor(v / self.resolution_m) for v in high]
        for x in range(lo[0], hi[0]):
            for y in range(lo[1], hi[1]):
                for z in range(lo[2], hi[2]):
                    key = (x, y, z)
                    if key not in self.occupied:
                        self.free.add(key)

    @staticmethod
    def _point(point):
        if len(point) != 3 or not all(math.isfinite(v) for v in point):
            raise ValueError('survey point must be a finite XYZ vector')
        return tuple(float(v) for v in point)

    def _key(self, point):
        return tuple(math.floor(v / self.resolution_m) for v in point)

    def _centre(self, key):
        return tuple((v + .5) * self.resolution_m for v in key)

    def _ray_keys(self, start, end):
        """Voxel traversal along a measured finite ray; no ray past its end."""
        x, y, z = self._key(start)
        final_x, final_y, final_z = self._key(end)
        yield (x, y, z)
        if x == final_x and y == final_y and z == final_z:
            return
        dx, dy, dz = end[0] - start[0], end[1] - start[1], end[2] - start[2]
        step_x = 1 if dx > 0 else -1 if dx < 0 else 0
        step_y = 1 if dy > 0 else -1 if dy < 0 else 0
        step_z = 1 if dz > 0 else -1 if dz < 0 else 0
        resolution = self.resolution_m
        delta_x = resolution / abs(dx) if dx else math.inf
        delta_y = resolution / abs(dy) if dy else math.inf
        delta_z = resolution / abs(dz) if dz else math.inf
        edge_x = ((x + (step_x > 0)) * resolution - start[0]) / dx if dx else math.inf
        edge_y = ((y + (step_y > 0)) * resolution - start[1]) / dy if dy else math.inf
        edge_z = ((z + (step_z > 0)) * resolution - start[2]) / dz if dz else math.inf
        # Each iteration advances at least one grid coordinate toward the end.
        # Scalar axes preserve the original simultaneous epsilon crossings,
        # without two three-element Python loops and list indexing per voxel.
        for _ in range(abs(x - final_x) + abs(y - final_y) + abs(z - final_z) + 1):
            if x == final_x:
                edge_x = math.inf
            if y == final_y:
                edge_y = math.inf
            if z == final_z:
                edge_z = math.inf
            crossing = min(edge_x, edge_y, edge_z) + 1e-12
            if x != final_x and edge_x <= crossing:
                x += step_x
                edge_x += delta_x
            if y != final_y and edge_y <= crossing:
                y += step_y
                edge_y += delta_y
            if z != final_z and edge_z <= crossing:
                z += step_z
                edge_z += delta_z
            yield (x, y, z)
            if x == final_x and y == final_y and z == final_z:
                return

    def integrate(self, origin, endpoints, hits, stamp, mode=None):
        """Integrate one actual scan; ``False`` is an explicit valid no-hit ray.

        Missing sensor returns must be omitted, not invented as no-hit rays.
        Input is validated before mutation. Persistent occupied voxels truncate
        conflicting rays, so a later miss cannot clear or observe through them.
        ``mode`` describes the platform when this scan was captured, never its
        mode when a delayed callback happens to run. Untagged scans can inform
        navigation but cannot fulfil AIR/WATER-specific monitoring work.
        """
        origin = self._point(origin)
        endpoints = tuple(self._point(p) for p in endpoints)
        hits = tuple(hits)
        if (len(hits) != len(endpoints) or any(type(hit) is not bool for hit in hits)
                or not math.isfinite(stamp) or stamp < 0
                or mode not in (None, 'AIR', 'WATER', 'SURFACE', 'TRANSITION', 'UNKNOWN')
                or self.stamp is not None and stamp < self.stamp):
            raise ValueError('invalid or stale measured scan')
        self._free_bounding_boxes.clear()
        self._nonfree_bounding_boxes.clear()
        rows = sorted(zip(endpoints, hits), key=lambda row: math.dist(origin, row[0]))
        current_observation = stamp >= self.observation_epoch_s
        measured = set()
        for endpoint, hit in rows:
            terminal = self._key(endpoint)
            for key in self._ray_keys(origin, endpoint):
                if key in self.occupied:
                    # A real repeat hit may provide the first measurement in
                    # another medium. A contradictory miss cannot do so.
                    if key == terminal and hit and current_observation:
                        measured.add(key)
                    break
                if key == terminal and hit:
                    self.occupied.add(key)
                    self.free.discard(key)
                    self._hits[key] = endpoint
                else:
                    self.free.add(key)
                if current_observation:
                    measured.add(key)
        # Ray sorting, occupied truncation and hit/free mutations above retain
        # their order. Measurement membership is idempotent within one scan;
        # update each visited cell once instead of allocating an unused default
        # set and adding its mode again for every overlapping sensor ray.
        self._measured.update(measured)
        if mode is not None:
            for key in measured:
                modes = self.measured_modes.get(key)
                if modes is None:
                    self.measured_modes[key] = {mode}
                elif mode not in modes:
                    modes.add(mode)
        # Empty scans do not create a known-free seed around the vehicle.
        self.stamp = float(stamp)
        origin_key = self._key(origin)
        if rows and origin_key != self._last_origin:
            self._visits[origin_key] = self._visits.get(origin_key, 0) + 1
            self._last_origin = origin_key

    def state(self, point):
        key = self._key(self._point(point))
        return 'OCCUPIED' if key in self.occupied else 'FREE' if key in self.free else 'UNKNOWN'

    def observed(self, point, mode=None):
        """Measured cell at the declared slice; not whole-surface coverage."""
        key = self._key(self._point(point))
        return key in self._measured and (mode is None or mode in self.measured_modes.get(key, ()))

    def covered_ids(self, points, mode=None):
        """Existing fixed, weighted observation cells that have ray evidence.

        A hit is a mapping result as well as an obstacle. Ray-free cells count
        as explored at that declared depth slice, not as an image of a seabed
        at another depth. This method never changes the task's denominator.
        """
        return tuple(key for key, point in points.items() if self.observed(point, mode))

    def hit_points(self):
        """One actual first-return point per occupied voxel, for map display."""
        return tuple(self._hits.values())

    def segment_clear(self, start, end, radius):
        """Require observed FREE throughout a conservative swept-body envelope.

        Use the Euclidean swept sphere, matching the actual hull safety proxy.
        This never accepts an UNKNOWN neighbour
        merely because the centreline ray was free. This is voxel geometry,
        not a proof of the native controller's tracking error bound.
        """
        start, end = self._point(start), self._point(end)
        if not math.isfinite(radius) or radius < 0:
            raise ValueError('finite nonnegative body radius required')
        for box in self._forbidden_boxes:
            expanded = ObstacleBox(box.centre, tuple(size + 2. * radius for size in box.size))
            if expanded.blocks(start, end):
                return False
        low_keys = [math.floor((min(a, b) - radius - 1e-10) / self.resolution_m)
                    for a, b in zip(start, end)]
        high_keys = [math.floor((max(a, b) + radius + 1e-10) / self.resolution_m)
                     for a, b in zip(start, end)]
        bounds = tuple(low_keys + high_keys)
        if bounds in self._free_bounding_boxes:
            return True
        def blocks(key):
            t0,t1=0.,1.
            for j in range(3):
                low=key[j]*self.resolution_m-radius
                high=(key[j]+1)*self.resolution_m+radius
                direction=end[j]-start[j]
                if abs(direction)<1e-12:
                    if start[j]<low or start[j]>high:return False
                else:
                    a,b=(low-start[j])/direction,(high-start[j])/direction
                    t0,t1=max(t0,min(a,b)),min(t1,max(a,b))
                    if t0>t1:return False
            low=tuple(k*self.resolution_m for k in key)
            high=tuple((k+1)*self.resolution_m for k in key)
            if not self.spherical_hull:return True
            return segment_box_distance_sq(start,end,low,high)<=radius*radius+1e-12
        nonfree=self._nonfree_bounding_boxes.get(bounds)
        if nonfree is not None:
            # A successful prior segment supplies only this box's exact
            # nonfree membership. Every new segment repeats the unchanged
            # swept-sphere predicate for those cells.
            return not any(blocks(key) for key in nonfree)
        nonfree=[]
        for ix in range(low_keys[0],high_keys[0]+1):
            for iy in range(low_keys[1],high_keys[1]+1):
                for iz in range(low_keys[2],high_keys[2]+1):
                    key=(ix,iy,iz)
                    if key in self.free and key not in self.occupied:continue
                    nonfree.append(key)
                    if blocks(key):return False
        # Reuse only a fully observed-free box. A successful slab intersection
        # test alone cannot certify other segments sharing these same bounds.
        if not nonfree:
            self._free_bounding_boxes.add(bounds)
        else:
            self._nonfree_bounding_boxes[bounds]=tuple(nonfree)
        return True

    def next_target(self, current, goal, radius, max_step_m=1., z_bounds=None, peer_positions=(), peer_clearance_m=0., prefer_direct_step=False, allow_vertical=False, motion_filter=None):
        """Find a bounded short target through the currently observed free map.

        A known goal uses A*; an unknown/occupied goal selects a reachable near
        frontier biased toward the goal and away from repeatedly visited cells.
        Equal-depth requests stay at that depth. No safe move returns None, so
        callers can obtain another scan/change heading rather than enter unknown.
        """
        current, goal = self._point(current), self._point(goal)
        if z_bounds is not None and (len(z_bounds)!=2 or not all(math.isfinite(v) for v in z_bounds) or z_bounds[0]>z_bounds[1]):
            raise ValueError('finite ordered local working-height bounds required')
        if not math.isfinite(max_step_m) or max_step_m <= 0:
            raise ValueError('positive local step required')
        if not self.segment_clear(current, current, radius):
            return None
        peers=tuple(self._point(p) for p in peer_positions)
        if not math.isfinite(peer_clearance_m) or peer_clearance_m<0:
            raise ValueError('finite nonnegative peer clearance required')
        def clear(a,b):
            if not self.motion_clear(a,b,radius):return False
            if motion_filter is not None and not motion_filter(a,b):return False
            delta=tuple(y-x for x,y in zip(a,b));length2=sum(v*v for v in delta)
            for peer in peers:
                t=0. if length2<1e-12 else max(0.,min(1.,sum(
                    (p-x)*v for p,x,v in zip(peer,a,delta))/length2))
                closest=tuple(x+t*v for x,v in zip(a,delta))
                if math.dist(peer,closest)<peer_clearance_m:return False
            return True
        distance = math.dist(current, goal)
        if distance <= max_step_m and clear(current, goal):
            return goal
        if prefer_direct_step and distance>max_step_m:
            short=tuple(a+(b-a)*max_step_m/distance for a,b in zip(current,goal))
            if (z_bounds is None or z_bounds[0]<=short[2]<=z_bounds[1]) and clear(current,short):
                return short
        start = self._key(current)
        # Small tracking errors must not move a declared cruise/return layer
        # onto an arbitrary voxel-centre height (e.g. 2.0 -> 1.875 m). Once
        # within one map cell of that layer, connect to its exact height.
        planar = not allow_vertical and abs(current[2] - goal[2]) <= self.resolution_m

        def position(key):
            p = self._centre(key)
            return (p[0], p[1], goal[2]) if planar else p

        neighbours = ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0))
        if not planar:
            neighbours += ((0, 0, 1), (0, 0, -1))
        queue = [(distance, 0., start)]
        costs, parent, closed = {start: 0.}, {}, set()
        best, score, found_goal = start, math.inf, False
        # A planning call must not consume the model's integration interval.
        # Exhausting this local budget keeps a safe incumbent, not "no path".
        deadline = time.monotonic() + .025
        while queue and len(closed) < 768 and time.monotonic() < deadline:
            _, cost, key = heapq.heappop(queue)
            if key in closed:
                continue
            closed.add(key)
            p = current if key == start else position(key)
            remaining = math.dist(p, goal)
            if remaining <= self.resolution_m * 1.8 and clear(p, goal):
                best, found_goal = key, True
                break
            candidate_score = remaining + self.resolution_m * 2 * self._visits.get(key, 0)
            if key != start and candidate_score < score:
                best, score = key, candidate_score
            for offset in neighbours:
                adjacent = tuple(key[j] + offset[j] for j in range(3))
                if adjacent in closed or adjacent not in self.free or adjacent in self.occupied:
                    continue
                q = position(adjacent)
                if z_bounds is not None and not z_bounds[0]-1e-9<=q[2]<=z_bounds[1]+1e-9:
                    continue
                new_cost = cost + math.dist(p, q)
                if new_cost >= costs.get(adjacent, math.inf):
                    continue
                if not clear(p, q):
                    continue
                costs[adjacent], parent[adjacent] = new_cost, key
                heapq.heappush(queue, (new_cost + math.dist(q, goal), new_cost, adjacent))
        if best == start and not found_goal:
            return None
        path = [goal] if found_goal else []
        key = best
        while key != start:
            path.append(position(key))
            key = parent[key]
        path.reverse()
        target = None
        for point in path:
            length = math.dist(current, point)
            if length > max_step_m:
                point = tuple(a + (b - a) * max_step_m / length for a, b in zip(current, point))
            if not clear(current, point):
                break
            target = point
            if length >= max_step_m:
                break
        return target


def action_terminal_event(request,goal_id,producer,generated_at,terminal_state,
                          task_completed,terminal_verified,resource_locked,reason):
    """Minimal local Action result notice for the existing finite transport."""
    if (request is None or not goal_id or not producer or
            terminal_state not in ('SUCCEEDED','CANCELED','ABORTED') or
            not math.isfinite(generated_at) or generated_at<0 or
            any(type(value) is not bool for value in
                (task_completed,terminal_verified,resource_locked))):
        raise ValueError('invalid local Action terminal event')
    return dict(event_type='ACTION_TERMINAL',
        product_id=goal_id+':action_terminal:'+producer,
        request_id=request.request_id,goal_id=goal_id,producer=producer,
        generated_at=generated_at,terminal_state=terminal_state,
        task_completed=task_completed,terminal_verified=terminal_verified,
        resource_locked=resource_locked,reason=str(reason))


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


def communication_settings(scene):
    values=dict(model='FINITE_STAGE_SERVICE',acoustic_range_m=8.,radio_range_m=30.,
                acoustic_bytes_per_s=2048.,radio_bytes_per_s=32768.,
                acoustic_propagation_mps=1500.,radio_propagation_mps=299792458.)
    values.update(scene.get('communication',{}))
    for key,value in values.items():
        if key=='model':continue
        if not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
            raise ValueError('positive finite communication parameter required: '+key)
    return values


def message_wire_size(event):
    return 4+len(json.dumps(event,allow_nan=False).encode('utf-8'))


def service_upload_candidates(scene,position,radius_m=1.5):
    from .experiment_verdict import StaticSceneGeometry
    settings=communication_settings(scene)
    mother=tuple(scene.get('mother_ship_receiver_position',scene['mother_ship_position']))
    reach=settings['radio_range_m']-radius_m
    if reach<=abs(mother[2]):
        raise ValueError('radio range cannot provide a surface upload window')
    horizontal=math.sqrt(reach*reach-mother[2]*mother[2])
    angle=math.atan2(position[1]-mother[1],position[0]-mother[0])
    candidates=[tuple(position[:2])+(0.,)]
    candidates.extend((mother[0]+horizontal*math.cos(angle+index*math.pi/8),
        mother[1]+horizontal*math.sin(angle+index*math.pi/8),0.) for index in range(16))
    candidates.append(tuple(scene['return_sites']['usv']['position']))
    geometry=StaticSceneGeometry.from_mapping(scene)
    candidates=[point for point in dict.fromkeys(candidates) if math.dist(point,mother)<=reach+1e-8 and
        (geometry is None or not geometry.violation(point,1.1891593669479295))]
    if not candidates:raise ValueError('no declared safe surface upload position')
    return tuple(sorted(candidates,key=lambda point:math.dist(point,position)))


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
        self.in_flight=[]
        self.sent_prefixes={}
        self.committed_prefixes={}
        self.transfers=[]

    def produce(self,product_id,product):
        if product_id in self.products:
            raise ValueError('product already generated')
        self.products[product_id]=product

    def advance(self,channel,now_s,bytes_per_second,flows):
        """Legacy single-channel boundary; same-time calls share a snapshot."""
        return self.advance_all(now_s,{channel:(bytes_per_second,flows)})

    def advance_all(self,now_s,channels,propagation_delays=None):
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
        propagation_delays=propagation_delays or {}
        # Validate every channel before modifying any ledger or epoch.
        for name,(rate,flows) in updates.items():
            if not name or not math.isfinite(rate) or rate<0:
                raise ValueError('invalid channel time/rate')
            for key,source,destination,available in flows:
                if key not in self.products or not source or not destination or source==destination or type(available) is not bool:
                    raise ValueError('invalid delivery flow')
                delay=propagation_delays.get((name,key,source,destination),0.)
                if not math.isfinite(delay) or delay<0:
                    raise ValueError('finite nonnegative propagation delay required')
        if self._epoch_time!=now_s:
            self._epoch_prefixes={k:dict(p.received_prefix) for k,p in self.products.items()}
            self._epoch_time=now_s
        receipts=[]
        pending=[]
        for arrival,key,destination,prefix in self.in_flight:
            if arrival>now_s:
                pending.append((arrival,key,destination,prefix));continue
            product=self.products[key]
            product.received_prefix[destination]=max(product.received_prefix.get(destination,0.),prefix)
            if destination==product.receiver and prefix>=product.required_bytes and product.received_at is None:
                product.received_at=now_s;receipts.append(key)
        self.in_flight=pending
        for channel,(rate,flows) in updates.items():
            before=self.channel_time.get(channel,self.start_time)
            budget=rate*(now_s-before)
            for key,source,destination,available in flows:
                product=self.products[key]
                if not available or product.generated_at>before:continue
                flow=(channel,key,source,destination)
                old=max(product.received_prefix.get(destination,0.),self.committed_prefixes.get((key,destination),0.))
                amount=min(budget,max(0.,self._epoch_prefixes.get(key,{}).get(source,0.)-old))
                if amount<=0:continue
                prefix=old+amount
                self.sent_prefixes[flow]=prefix
                self.committed_prefixes[(key,destination)]=prefix
                budget-=amount
                delay=propagation_delays.get(flow,0.)
                self.transfers.append(dict(channel=channel,product_id=key,sender=source,
                    receiver=destination,bytes=amount,sent_at=now_s,arrival_at=now_s+delay))
                if delay>0:
                    self.in_flight.append((now_s+delay,key,destination,prefix));continue
                product.received_prefix[destination]=prefix
                if destination==product.receiver and prefix>=product.required_bytes and product.received_at is None:
                    product.received_at=now_s
                    receipts.append(key)
            self.channel_time[channel]=now_s
        return tuple(receipts)


def radio_link_available(samples, source, destination, obstacles=()):
    """Use the same declared RF geometry for command feasibility and delivery."""
    if source not in samples or destination not in samples:
        return False
    a,ma=samples[source];b,mb=samples[destination]
    return (ma in ('AIR','SURFACE') and mb in ('AIR','SURFACE') and
            math.dist(a,b)<=30. and not any(o.blocks(a,b) for o in obstacles))


def declared_delivery_channels(products,previous,states,obstacles=(),continuous=True,settings=None,propagation_delays=None):
    """The frozen sampled link model, shared by prediction and live transport.

    State values are (position, actual medium). The mother ship knows neither
    this input nor intermediate relay prefixes; these belong to the simulator.
    """
    def link(samples,source,destination,water):
        if source not in samples or destination not in samples:return False
        a,ma=samples[source];b,mb=samples[destination]
        if water:
            if {ma,mb}!={'WATER','SURFACE'}:return False
        else:
            if settings is not None:
                return (ma in ('AIR','SURFACE') and mb in ('AIR','SURFACE') and
                    math.dist(a,b)<=settings['radio_range_m'])
            return radio_link_available(samples,source,destination,obstacles)
        if settings is not None:
            return math.dist(a,b)<=settings['acoustic_range_m']
        return math.dist(a,b)<=8. and not any(o.blocks(a,b) for o in obstacles)
    acoustic=[];radio=[]
    for ident,product in products.items():
        if product.received_at is not None:continue
        source=product.producer
        if source!='mother' and product.receiver!='mother':
            destination=product.receiver
            radio.append((ident,source,destination,continuous and
                link(previous,source,destination,False) and link(states,source,destination,False)))
            continue
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
    if settings is not None:
        if propagation_delays is not None:
            for channel,flows in (('acoustic',acoustic),('radio',radio)):
                speed=settings[channel+'_propagation_mps']
                for key,source,destination,available in flows:
                    if source in states and destination in states:
                        propagation_delays[(channel,key,source,destination)]=math.dist(states[source][0],states[destination][0])/speed
        return {'acoustic':(settings['acoustic_bytes_per_s'],acoustic),
                'radio':(settings['radio_bytes_per_s'],radio)}
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
        self.mapping_cells = {key: (point,) for key, (point, _) in self.points.items()}
        self.mapping_resolution_m = .25
        if getattr(request, 'execution_mode', '') == 'ONLINE_MAPPING':
            from qn_aav_simulator.monitoring_request import mapping_cell_samples
            for region in request.regions:
                if region.shape != 'CIRCLE':
                    continue  # A declared BOX point sample is still that point.
                for point in region.interest_points:
                    if point.point_id not in self.points:
                        continue
                    cells = mapping_cell_samples(region.center[:2], region.radius_m,
                        point.position, cell_size=region.coverage_resolution_m,
                        resolution=self.mapping_resolution_m)
                    if not cells:
                        raise ValueError('mapping report cell contains no required sampled voxel')
                    self.mapping_cells[point.point_id] = cells

    def unobserved_mapping_points(self, local_map, mode):
        """Actual residual fine cells, not unreceived coarse-cell centres."""
        return tuple(point for key, (_, required_mode) in self.points.items()
                     if key not in self.emitted and mode == required_mode
                     for point in self.mapping_cells[key]
                     if not local_map.observed(point, required_mode))

    def sample_mapping(self, local_map, mode, stamp):
        """Emit fixed slice-cell products only from actual measured map cells.

        This new region execution has no manufactured dwell or virtual sensor
        positions. It reports sampled occupancy coverage, not optical imagery,
        seabed quality or completeness of every unknown three-dimensional face.
        Existing point/dwell actions continue to use sample() below.
        """
        if not math.isfinite(stamp) or stamp < 0:
            raise ValueError('invalid mapping product time')
        if local_map.stamp is None or local_map.stamp > stamp:
            return ()
        if not math.isclose(local_map.resolution_m, self.mapping_resolution_m, abs_tol=1e-9):
            raise ValueError('mapping window and measured map resolutions differ')
        events = []
        for key, (point, required_mode) in self.points.items():
            cells = self.mapping_cells[key]
            if key in self.emitted or mode != required_mode or not all(
                    local_map.observed(cell, required_mode) for cell in cells):
                continue
            self.emitted.add(key)
            states = {local_map.state(cell) for cell in cells}
            event = dict(product_id=self.goal_id + ':' + key, request_id=self.request.request_id,
                         goal_id=self.goal_id, point_id=key, producer=self.producer,
                         generated_at=stamp, observed=True,
                         result=dict(model='MAPPING_PROXY', dwell_s=0.,
                                     resolution_m=local_map.resolution_m,
                                     observed_state=next(iter(states)) if len(states) == 1 else 'MIXED',
                                     source_mode=required_mode,
                                     sampled_voxel_count=len(cells),
                                     scope='DECLARED_DEPTH_SLICE_OCCUPANCY'))
            if self.request.template_id != 'OFFSHORE_JOINT':
                event['required_bytes'] = 32 * 1024
            events.append(event)
        return tuple(events)

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
            event=dict(product_id=self.goal_id+':'+key,request_id=self.request.request_id,
                goal_id=self.goal_id,point_id=key,producer=self.producer,generated_at=stamp,observed=True,
                result=dict(model='GEOMETRIC_PROXY',dwell_s=dwell))
            if self.request.template_id!='OFFSHORE_JOINT':event['required_bytes']=32*1024
            events.append(event)
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


def predict_local_products(request,point_ids,producer,goal_id,traces,obstacles,deadline):
    """Check the declared observation on the actual method rollout."""
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
    return dict(status='FEASIBLE',reason='LOCAL_GEOMETRIC_OBSERVATION',
                generated_events=generated)


def predict_received_products(request,point_ids,producer,goal_id,traces,mother_position,obstacles,deadline):
    """Evaluate observations and finite receipt on supplied complete rollouts."""
    local=predict_local_products(request,point_ids,producer,goal_id,traces,obstacles,deadline)
    if local['status']!='FEASIBLE':return local
    generated=local['generated_events']
    horizon=min(rows[-1][0] for rows in traces.values())
    result=predict_received_events(generated,traces,
        {member:((0.,horizon),) for member in traces},mother_position,obstacles,deadline)
    receipts={event['point_id']:result['received_at'][event['product_id']]
              for event in generated if event['product_id'] in result['received_at']}
    result.update(received_at=receipts,generated_events=generated)
    return result


def task_service_ready(states,sites):
    """Whether the shared USV is at a declared task support position."""
    sample=states.get('usv')
    return bool(sample and sample[1]=='SURFACE' and any(
        math.dist(sample[0],site['position'])<=site['radius_m'] for site in sites))


def predict_task_service_receipts(events,traces,intervals,sites,deadline):
    """Nominal task events: notifications are direct; products need support."""
    import time
    from bisect import bisect_right
    rows=traces.get('usv',())
    clocks=[row[0] for row in rows]
    horizon=max((end for ranges in intervals.values() for _,end in ranges),default=0.)
    received={event['product_id']:event['generated_at'] for event in events
              if event.get('event_type')=='OBSERVATION_TERMINAL'}
    for tick in range(int(math.floor(horizon*10.))+1):
        if time.monotonic()>=deadline:
            return dict(status='UNKNOWN',reason='PLANNING_BUDGET_EXHAUSTED',received_at=received)
        stamp=tick/10.
        index=bisect_right(clocks,stamp+1e-8)-1
        usv_active=any(start<=stamp<=end for start,end in intervals.get('usv',()))
        state=({'usv':(rows[index][1],rows[index][2])}
            if usv_active and index>=0 and stamp-clocks[index]<=.010001 else {})
        supported=task_service_ready(state,sites)
        for event in events:
            ident=event['product_id']
            if ident not in received and event['generated_at']<=stamp and supported:
                received[ident]=stamp
    if len(received)!=len(events):
        return dict(status='INFEASIBLE',reason='TASK_SUPPORT_WINDOW_MISSING',received_at=received)
    return dict(status='FEASIBLE',received_at=received,
                receipt_finish_s=max(received.values(),default=0.))


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
