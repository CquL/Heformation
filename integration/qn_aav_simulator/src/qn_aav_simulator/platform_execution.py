"""Local finite reference execution; no allocator and no dynamics reset.

The owning node supplies a lock around claim/adoption/step. Actual medium is an
input from the plant; neither a reference claim nor a nominal end time sets it.
"""
from dataclasses import dataclass
import math
from typing import Tuple


def actual_mode(flag):
    if flag is None or not math.isfinite(flag) or not 0 <= flag <= 1:
        return 'UNKNOWN'
    return 'AIR' if flag == 0 else 'WATER' if flag == 1 else 'TRANSITION'


class ReferenceOwnership:
    SOURCES = frozenset(('AIR_SWARM','PLATFORM'))

    def __init__(self):
        self.source='AIR_SWARM'
        self.generation=0
        self.goal_id=''
        self.active=False
        self.locked=False
        self.retired=set()

    def claim(self, goal_id, source, expected_generation):
        if not goal_id or source not in self.SOURCES:
            return False,'INVALID_CONTEXT'
        if self.locked:
            return False,'UNKNOWN_LOCKED'
        if goal_id == self.goal_id and source == self.source and self.active:
            return True,'ALREADY_ACCEPTED'
        if expected_generation != self.generation or goal_id in self.retired:
            return False,'STALE_CONTEXT'
        if self.active and goal_id != self.goal_id:
            return False,'MEMBER_BUSY'
        if self.generation >= 2147483647:
            return False,'GENERATION_EXHAUSTED'
        self.generation+=1
        self.goal_id=goal_id
        self.source=source
        self.active=True
        return True,'ACCEPTED_REFERENCE_SOURCE_ONLY'

    def finish(self, goal_id, terminal_verified):
        if goal_id != self.goal_id or not self.active:
            return False
        self.retired.add(goal_id)
        self.active=False
        if not terminal_verified:
            self.locked=True
        # Keep the current source/reference until a valid successor takes over.
        return True

    def accepts(self, source, generation):
        return source == self.source and generation == self.generation


@dataclass(frozen=True)
class Segment:
    operation: str
    points: Tuple[Tuple[float,float,float], ...]
    duration: float

    def __post_init__(self):
        if self.operation not in ('ENTER_WATER','WATER_PATH','EXIT_WATER'):
            raise ValueError('unsupported qn operation')
        if len(self.points)<2 or any(len(p)!=3 or not all(math.isfinite(v) for v in p) for p in self.points):
            raise ValueError('finite map path with at least two points required')
        if not math.isfinite(self.duration) or self.duration<=0:
            raise ValueError('positive finite segment duration required')

    @property
    def target_mode(self):
        return 'AIR' if self.operation=='EXIT_WATER' else 'WATER'

    def reference(self, elapsed):
        lengths=[math.dist(a,b) for a,b in zip(self.points,self.points[1:])]
        distance=sum(lengths)*min(1.,max(0.,elapsed/self.duration))
        for a,b,length in zip(self.points,self.points[1:],lengths):
            if length>0 and distance<length:
                f=distance/length
                return tuple(a[i]+f*(b[i]-a[i]) for i in range(3))
            distance-=length
        return self.points[-1]


def validate_fragment(segments, mode, terminal_behavior, qualified_operations):
    if not segments or len(segments)>16:
        raise ValueError('fragment requires 1..16 finite segments')
    if terminal_behavior != 'FIXED_REFERENCE':
        raise ValueError('terminal behavior has no qn qualification adapter')
    for index,segment in enumerate(segments):
        if segment.operation not in qualified_operations:
            raise ValueError('operation is not qualified: '+segment.operation)
        required='AIR' if segment.operation=='ENTER_WATER' else 'WATER'
        if mode!=required:
            raise ValueError('operation is incompatible with actual/preceding mode')
        if index and math.dist(segments[index-1].points[-1],segment.points[0])>1e-9:
            raise ValueError('fragment paths must join continuously')
        mode=segment.target_mode
    return mode
