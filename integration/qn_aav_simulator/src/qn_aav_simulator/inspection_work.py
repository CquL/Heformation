"""Facility-relative control work, pure geometry and measured interval accounting.

Layered exterior contours: Galceran et al., JFR 2015, §4.2/§5.
Static blade side motion: Castelar Wembers et al., JFR 2024, §3/§4.2.
Profiles here are declared model-scale control tasks, not sensing certification.
No ROS, resource owner, simulator or scheduler lives in this module.
"""
import copy
import hashlib
import json
import math

INSPECTION_TEMPLATES=('WIND_INSPECTION','PLATFORM_PIPELINE_INSPECTION')
JOINT_TEMPLATES=('OFFSHORE_JOINT',)+INSPECTION_TEMPLATES
CONTOUR_LOOKAHEAD_M=.8


def inspection_motion_profile(scene):
    """The scene's declared reference speeds and native Otter force input."""
    profile=dict(air_planner_speed_mps=.35,air_nominal_speed_mps=.4,
                 aav_water_speed_mps=.1,uuv_water_speed_mps=.13,
                 usv_nominal_speed_mps=.2,usv_propulsion_effort_n=20.,usv_local_query_budget_s=.25)
    profile.update(scene.get('inspection_motion_profile',{}))
    for key,value in profile.items():
        if not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
            raise ValueError('invalid inspection motion profile '+key)
    if scene.get('inspection_motion_profile') and profile['air_nominal_speed_mps']>profile['air_planner_speed_mps']:
        raise ValueError('AIR nominal speed exceeds native planner speed')
    return profile


def inspection_reference_speed(work,member):
    profile=inspection_motion_profile({'inspection_motion_profile':work.get('motion_profile',{})})
    if work['domain']=='AIR':return profile['air_nominal_speed_mps']
    return profile['aav_water_speed_mps' if member.startswith('drone_') else 'uuv_water_speed_mps']


def inspection_exit_runout(work):
    """Declared uncredited straight exit alignment; local sensing admits it."""
    end=work_legs(work)[-1]['end']
    heading=float(work['transition_heading_rad']);distance=float(work['transition_runout_m'])
    return (end[0]+distance*math.cos(heading),end[1]+distance*math.sin(heading),end[2])


def transition_alignment_ready(work,quaternion,body_angular_velocity):
    """Measured pose/rate gate for the declared qn vertical conversion."""
    w,x,y,z=quaternion
    roll=math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
    pitch=math.asin(max(-1.,min(1.,2*(w*y-z*x))))
    heading_error=abs(math.remainder(yaw_of(quaternion)-work['transition_heading_rad'],2*math.pi))
    angular_speed=math.sqrt(sum(v*v for v in body_angular_velocity))
    return (all(math.isfinite(v) for v in (*quaternion,*body_angular_velocity)) and
        heading_error<=work['transition_heading_tolerance_rad'] and
        max(abs(roll),abs(pitch))<=work['transition_roll_pitch_limit_rad'] and
        angular_speed<=work['transition_angular_rate_limit_radps'])


def effective_contour_offset(requested,domain):
    """Fresh work geometry outside the unchanged inflated measured-map voxels.

    The .95/.45 m AIR/WATER navigation envelopes expand voxel boxes on each
    axis. A circular corner therefore needs sqrt(2) times that clearance,
    plus the .25 m voxel overhang. The 22.5 degree sampled chords and moving
    lookahead need further chord clearance. WATER uses its existing lookahead
    length as the geometric floor; an arrival tolerance is not extra hull size.
    This selects an intention before acceptance, never changes a work ledger.
    """
    if domain not in ('AIR','WATER') or not math.isfinite(requested) or requested<=0:
        raise ValueError('invalid contour offset')
    navigation_radius=.95 if domain=='AIR' else .45
    turn_floor=1.8 if domain=='AIR' else CONTOUR_LOOKAHEAD_M
    chord_factor=math.cos(math.pi/16)*math.cos(CONTOUR_LOOKAHEAD_M/(2*turn_floor))
    minimum=max(turn_floor,math.sqrt(2.)*(navigation_radius+.25)/chord_factor)
    return max(float(requested),minimum)


def yaw_of(quaternion):
    w,x,y,z=quaternion
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))


def layered_contour(center,half_size,levels,offset):
    """Rounded exterior rectangle; corners respect forward-motion turn radius."""
    cx,cy=center[:2];hx,hy=half_size[:2];out=[]
    for z in levels:
        points=[]
        for x,y,start in ((cx+hx,cy+hy,0.),(cx-hx,cy+hy,math.pi/2),
                          (cx-hx,cy-hy,math.pi),(cx+hx,cy-hy,3*math.pi/2)):
            for n in range(5):
                angle=start+n*math.pi/8
                points.append((x+offset*math.cos(angle),y+offset*math.sin(angle),z))
        out.append(dict(kind='CONTOUR',points=points+[points[0]],focus=(cx,cy,z)))
    return out


def offset_line(points,lateral=0.,vertical=0.):
    shifted=[]
    for i,p in enumerate(points):
        a=points[max(0,i-1)];b=points[min(len(points)-1,i+1)]
        dx,dy=b[0]-a[0],b[1]-a[1];norm=math.hypot(dx,dy) or 1.
        shifted.append((p[0]+lateral*dy/norm,p[1]-lateral*dx/norm,p[2]+vertical))
    return dict(kind='LINE',points=shifted)


def local_work(position,duration):
    return dict(kind='LOCAL',points=[tuple(position)],duration_s=float(duration))


def work_legs(work):
    rows=[]
    for section_index,section in enumerate(work['sections']):
        points=section['points']
        if section['kind']=='LOCAL':
            rows.append(dict(start=tuple(points[0]),end=tuple(points[0]),
                             length=float(section['duration_s']),kind='LOCAL',focus=section.get('focus'),section=section_index))
        else:
            for a,b in zip(points,points[1:]):
                if math.dist(a,b)>1e-9:
                    rows.append(dict(start=tuple(a),end=tuple(b),length=math.dist(a,b),
                                     kind=section['kind'],focus=section.get('focus'),section=section_index))
    return rows


def inspection_work_budget(work,start,nominal_speed=None):
    """Model time estimate for declared work and one full rescan reserve.

    Connecting section entries are navigation, not qualified work. LOCAL
    lengths are seconds; moving leg lengths are metres. Neither is credit.
    """
    sections=work['sections'];legs=work_legs(work)
    traversals=int(work.get('execution_work_traversals',2))
    margin=float(work.get('execution_search_margin_model_s',600.))
    speed=float(work['nominal_speed_mps'] if nominal_speed is None else nominal_speed)
    if traversals<1 or not math.isfinite(margin) or margin<0 or not math.isfinite(speed) or speed<=0:
        raise ValueError('invalid declared inspection execution reserve')
    approach=math.dist(start,sections[0]['points'][0])
    connections=sum(math.dist(a['points'][-1],b['points'][0]) for a,b in zip(sections,sections[1:]))
    repeat_connection=(traversals-1)*math.dist(sections[-1]['points'][-1],sections[0]['points'][0])
    navigation=approach+traversals*connections+repeat_connection
    movement=sum(l['length'] for l in legs if l['kind']!='LOCAL')
    hold=sum(l['length'] for l in legs if l['kind']=='LOCAL')
    duration=(navigation+traversals*movement)/speed+traversals*hold+margin
    return dict(model_duration_s=duration,nominal_speed_mps=speed,
                navigation_length_m=navigation,required_movement_length_m=movement,
                required_local_hold_s=hold,declared_work_traversals=traversals,model_search_margin_s=margin)


def inspection_wall_budget(work,budget,measured_rate=None,declared_speed=1.):
    """Freeze a finite computation watchdog from observed model/wall speed."""
    fallback=float(work.get('execution_wall_model_rate_fallback',1.))
    floor=float(work.get('execution_wall_model_rate_floor',.5))
    reserve=float(work.get('execution_wall_reserve_factor',2.))
    if any(not math.isfinite(v) or v<=0 for v in (fallback,floor,reserve,declared_speed)) or reserve<1:
        raise ValueError('invalid inspection wall reserve')
    observed=measured_rate is not None and math.isfinite(measured_rate) and measured_rate>0
    rate=max(floor,min(declared_speed,measured_rate if observed else fallback))
    return dict(model_duration_s=budget['model_duration_s'],wall_duration_s=budget['model_duration_s']*reserve/rate,
                wall_duration_ceiling_s=budget['model_duration_s']*reserve/floor,
                sampled_model_wall_rate=measured_rate if observed else None,
                watchdog_model_wall_rate=rate,wall_rate_source='ACTUAL_RECENT_WINDOW' if observed else 'FINITE_1X_FALLBACK',
                wall_reserve_factor=reserve,wall_model_rate_floor=floor)


def validate_work(work):
    if (not work.get('work_id') or not work.get('version') or not work.get('object_id') or
            work.get('domain') not in ('AIR','WATER') or not work.get('sections')):
        raise ValueError('inspection work needs identity, version, object, domain and sections')
    for field in ('position_tolerance_m','speed_limit_mps','nominal_speed_mps','heading_tolerance_rad'):
        if not math.isfinite(work[field]) or work[field]<=0:raise ValueError('invalid inspection '+field)
    if work['nominal_speed_mps']>work['speed_limit_mps']:
        raise ValueError('inspection nominal speed exceeds the declared speed envelope')
    if work.get('motion_profile'):
        inspection_motion_profile({'inspection_motion_profile':work['motion_profile']})
    for section in work['sections']:
        if section['kind'] not in ('CONTOUR','LINE','LOCAL') or not section['points']:
            raise ValueError('invalid inspection section')
        if any(len(p)!=3 or not all(math.isfinite(v) for v in p) for p in section['points']):
            raise ValueError('invalid inspection coordinates')
    if not work_legs(work):raise ValueError('empty inspection movement')


def new_progress(work):
    validate_work(work)
    return dict(work_id=work['work_id'],version=work['version'],intervals={},previous=None,
                _legs=work_legs(work),active_leg=0,approaching=True,complete=False,fraction=0.,qualified=0.,total=float(len(work_legs(work))))


def resume_progress(work, previous):
    """Retain qualified unions only for the same accepted work version.

    A new Goal must approach its active section afresh. Never join the last
    sample of a cancelled Goal to a sample after the handover or pause.
    Request identity is checked by the caller's ledger key.
    """
    if (previous is None or previous.get('work_id') != work['work_id'] or
            previous.get('version') != work['version']):
        return new_progress(work)
    validate_work(work)
    progress=copy.deepcopy(previous)
    progress['_legs']=work_legs(work)
    progress['previous']=None
    progress['approaching']=not progress['complete']
    progress.pop('entry_leg',None);progress.pop('entry_lead_reached',None)
    progress.pop('residual',None)
    progress.pop('air_connector',None)
    navigation=progress.get('contour_navigation')
    if navigation:
        navigation['previous']=None;navigation['net_forward_arc_m']=0.
    index=progress['active_leg']
    if (not progress['complete'] and progress['_legs'][index]['kind']=='LOCAL'):
        # A continuous hold cannot span a cancelled Goal or a pause.
        progress['intervals'].pop(str(index),None)
        progress['qualified']=sum(sum(b-a for a,b in values)
                                  for values in progress['intervals'].values())
        progress['fraction']=progress['qualified']/progress['total']
    return progress


def interval_union(intervals,low,high):
    result=[]
    for a,b in sorted(list(intervals)+[(max(0.,low),min(1.,high))],key=lambda pair:(pair[0],pair[1])):
        if a>b:continue
        if result and a<=result[-1][1]+1e-9:result[-1][1]=max(result[-1][1],b)
        else:result.append([a,b])
    return result


def projection(position,leg):
    a,b=leg['start'],leg['end'];d=tuple(b[i]-a[i] for i in range(3));norm=sum(v*v for v in d)
    t=sum((position[i]-a[i])*d[i] for i in range(3))/norm if norm else 0.
    closest=tuple(a[i]+max(0.,min(1.,t))*d[i] for i in range(3))
    return t,math.dist(position,closest)


def required_heading(work,leg,position):
    focus=leg.get('focus')
    if work['domain']=='AIR' and focus:
        return math.atan2(focus[1]-position[1],focus[0]-position[0])
    a,b=leg['start'],leg['end']
    if math.hypot(b[0]-a[0],b[1]-a[1])<1e-8:return None
    return math.atan2(b[1]-a[1],b[0]-a[0])


def contour_location(legs,index,position):
    group=[n for n,l in enumerate(legs) if l['section']==legs[index]['section']]
    offsets={};length=0.
    for n in group:offsets[n]=length;length+=legs[n]['length']
    centre=legs[index]['focus'];qx=position[0]-centre[0];qy=position[1]-centre[1]
    for n in group:
        a,b=legs[n]['start'],legs[n]['end'];dx=b[0]-a[0];dy=b[1]-a[1];den=qx*dy-qy*dx
        if abs(den)<1e-12:continue
        ax=a[0]-centre[0];ay=a[1]-centre[1]
        u=(ax*qy-ay*qx)/den;r=(ax*dy-ay*dx)/den
        if r>=0. and -1e-9<=u<=1.+1e-9:
            return n,offsets[n]+max(0.,min(1.,u))*legs[n]['length'],group,offsets,length
    n=min(group,key=lambda i:projection(position,legs[i])[1]);q,_=projection(position,legs[n])
    return n,offsets[n]+max(0.,min(1.,q))*legs[n]['length'],group,offsets,length


def contour_lookahead(legs,index,position):
    _,arc,group,offsets,length=contour_location(legs,index,position)
    # A moving intention ahead on the declared contour keeps the existing
    # forward-only controller steering. Connections/obstacles remain local.
    return contour_point(legs,group,offsets,length,arc+CONTOUR_LOOKAHEAD_M)


def contour_point(legs,group,offsets,length,arc):
    arc%=length
    n=next((n for n in group if arc<=offsets[n]+legs[n]['length']),group[-1])
    fraction=(arc-offsets[n])/legs[n]['length'];a,b=legs[n]['start'],legs[n]['end']
    return tuple(x+fraction*(y-x) for x,y in zip(a,b))


def note_contour_navigation(work,progress,index,stamp,position,mode,arc,length):
    """Observe a continuous net forward circuit; never credit qualified work."""
    section=progress['_legs'][index]['section']
    navigation=progress.get('contour_navigation')
    if navigation is None or navigation['section']!=section:
        navigation=dict(section=section,previous=None,net_forward_arc_m=0.,circuit_complete=False)
        progress['contour_navigation']=navigation
    previous=navigation['previous']
    if mode!=work['domain']:
        navigation['previous']=None;navigation['net_forward_arc_m']=0.;return
    if previous and stamp<=previous[0]:return
    connected=(previous is not None and 0<stamp-previous[0]<=.75 and
        math.dist(position,previous[1])<=max(.03,work['speed_limit_mps']*(stamp-previous[0])*1.5))
    delta=math.remainder(arc-previous[2],length) if connected else 0.
    if connected and abs(delta)<=max(.05,3.*math.dist(position,previous[1])):
        navigation['net_forward_arc_m']+=delta
        if navigation['net_forward_arc_m']>=length-1e-8:navigation['circuit_complete']=True
    else:navigation['net_forward_arc_m']=0.
    navigation['previous']=(float(stamp),tuple(position),arc)


def residual_target(work,progress,position,velocity,yaw=None):
    legs=progress['_legs'];index=progress['active_leg']
    _,arc,group,offsets,length=contour_location(legs,index,position)
    residual=progress.get('residual')
    if residual and residual['section']!=legs[index]['section']:
        progress.pop('residual');residual=None
    if residual is None:
        navigation=progress.get('contour_navigation',{})
        if navigation.get('section')!=legs[index]['section'] or not navigation.get('circuit_complete'):
            return None
        for n in group:
            edge=0.;gap=None
            for a,b in progress['intervals'].get(str(n),()):
                if a>edge+1e-8:gap=(edge,a);break
                edge=max(edge,b)
            if gap is None and edge<1.-1e-8:gap=(edge,1.)
            if gap:
                pad=2*work['position_tolerance_m']
                before=offsets[n]+gap[0]*legs[n]['length']-pad
                after=offsets[n]+gap[1]*legs[n]['length']+pad
                residual=dict(section=legs[index]['section'],phase='APPROACH',
                    before=contour_point(legs,group,offsets,length,before),
                    after=contour_point(legs,group,offsets,length,after),
                    before_arc_m=before,after_arc_m=after)
                progress['residual']=residual;progress['previous']=None;break
    if residual is None:return None
    target=residual['before'] if residual['phase']=='APPROACH' else residual['after']
    # The padded sweep already starts/ends two declared tolerances outside
    # the missing interval. Use that same admitted work-position tolerance;
    # a separate .1 m AIR stop gate can strand an otherwise valid terminal.
    gate=work['position_tolerance_m']
    desired=required_heading(work,legs[index],position)
    heading_ready=(yaw is None or desired is None or
        abs(math.remainder(yaw-desired,2*math.pi))<=work['heading_tolerance_rad'])
    if (math.dist(position,target)<=gate and
            (residual['phase']!='APPROACH' or work['domain']=='WATER' or heading_ready)):
        progress['previous']=None
        if residual['phase']=='APPROACH':
            residual['phase']='SWEEP'
        else:progress.pop('residual');return None
    if residual['phase']=='SWEEP':
        # Follow the declared forward arc through each corner. The whole
        # padded before-to-after chord can cross the inflated facility.
        current_arc=residual['before_arc_m']+math.remainder(arc-residual['before_arc_m'],length)
        target_arc=max(residual['before_arc_m'],min(residual['after_arc_m'],current_arc+CONTOUR_LOOKAHEAD_M))
        target=contour_point(legs,group,offsets,length,target_arc)
    return tuple(target)


def air_line_connector(work,progress,position):
    """Uncredited cross-side approach using this work's measured exterior ring."""
    legs=progress['_legs'];index=progress['active_leg'];leg=legs[index]
    connector=progress.get('air_connector')
    if connector and connector['target_leg']!=index:
        progress.pop('air_connector');connector=None
    if connector is None:
        focus=leg.get('focus')
        if (work['domain']!='AIR' or leg['kind']!='LINE' or focus is None or
                not any((position[a]-focus[a])*(leg['start'][a]-focus[a])<0 for a in (0,1))):
            return None
        ring=next((n for n in reversed(range(index)) if legs[n]['kind']=='CONTOUR' and
            all(sum(b-a for a,b in progress['intervals'].get(str(j),()))>=1.-1e-8
                for j,l in enumerate(legs) if l['section']==legs[n]['section'])),None)
        if ring is None:return None
        _,entry,group,offsets,length=contour_location(legs,ring,position)
        _,exit,_,_,_=contour_location(legs,ring,leg['start'])
        delta=math.remainder(exit-entry,length)
        connector=dict(target_leg=index,ring_leg=ring,phase='ENTRY',entry_arc_m=entry,
                       exit_arc_m=exit,delta_arc_m=delta)
        progress['air_connector']=connector;progress['previous']=None
    ring=connector['ring_leg']
    _,arc,group,offsets,length=contour_location(legs,ring,position)
    entry=contour_point(legs,group,offsets,length,connector['entry_arc_m'])
    exit=contour_point(legs,group,offsets,length,connector['exit_arc_m'])
    gate=work['position_tolerance_m']
    if connector['phase']=='ENTRY':
        if math.dist(position,entry)>gate:return entry
        connector['phase']='ARC'
    if connector['phase']=='ARC':
        if math.dist(position,exit)<=gate:connector['phase']='EXIT'
        else:
            direction=1. if connector['delta_arc_m']>=0. else -1.
            advance=max(0.,direction*math.remainder(arc-connector['entry_arc_m'],length))
            next_arc=connector['entry_arc_m']+direction*min(
                abs(connector['delta_arc_m']),advance+CONTOUR_LOOKAHEAD_M)
            return contour_point(legs,group,offsets,length,next_arc)
    return leg['start']


def line_residual(work,progress,position):
    """Padded local intention for the first measured gap of the active LINE."""
    index=progress['active_leg'];leg=progress['_legs'][index]
    residual=progress.get('residual',{})
    if residual.get('kind')=='LINE' and residual.get('leg')==index:return residual
    edge=0.;gap=None
    for low,high in progress['intervals'].get(str(index),()):
        if low>edge+1e-8:gap=(edge,low);break
        edge=max(edge,high)
    if gap is None and edge<1.-1e-8:gap=(edge,1.)
    if gap is None:return None
    delta=tuple(leg['end'][a]-leg['start'][a] for a in range(3))
    pad=2.*work['position_tolerance_m']/leg['length']
    before=tuple(leg['start'][a]+(gap[0]-pad)*delta[a] for a in range(3))
    after=tuple(leg['start'][a]+(gap[1]+pad)*delta[a] for a in range(3))
    reverse=work['domain']=='AIR' and math.dist(position,after)<math.dist(position,before)
    residual=dict(kind='LINE',leg=index,section=leg['section'],phase='APPROACH',
        gap=list(gap),before=before,after=after,
        entry=after if reverse else before,exit=before if reverse else after)
    progress['residual']=residual
    return residual


def sample_progress(work,progress,stamp,position,velocity,yaw,mode):
    """Measured unions; connected contours continue forward through vertices.

    A contour's residual interval is revisited on the next forward circuit,
    not by commanding an underactuated member backwards to every tiny vertex.
    Connection/approach motion is never credited to a work interval.
    """
    if progress['version']!=work['version'] or progress['work_id']!=work['work_id']:
        raise ValueError('inspection progress belongs to another definition')
    legs=progress['_legs']
    if progress['complete']:return legs[-1]['end']
    index=progress['active_leg'];leg=legs[index];tol=work['position_tolerance_m']
    residual=progress.get('residual')
    if residual and (residual.get('section')!=leg['section'] or
            residual.get('kind')=='LINE' and residual.get('leg')!=index):
        progress.pop('residual');residual=None
    arrival_tol=tol
    if progress['approaching']:
        entry_heading=required_heading(work,leg,position)
        heading_ready=(entry_heading is None or
            abs(math.remainder(yaw-entry_heading,2*math.pi))<=work['heading_tolerance_rad'])
        entry=(residual['entry'] if residual and residual.get('kind')=='LINE' else leg['start'])
        if mode==work['domain'] and heading_ready and math.dist(position,entry)<=arrival_tol:
            progress['approaching']=False;progress['previous']=None
            if residual and residual.get('kind')=='LINE':residual['phase']='SWEEP'
            progress.pop('air_connector',None)
        else:
            if residual and residual.get('kind')=='LINE':
                if work['domain']=='WATER' and entry_heading is not None and (
                        not heading_ready or math.hypot(position[0]-entry[0],position[1]-entry[1])>tol):
                    lead=(entry[0]-2.*math.cos(entry_heading),entry[1]-2.*math.sin(entry_heading),entry[2])
                    identity=(index,tuple(residual['gap']))
                    if progress.get('entry_leg')!=identity:
                        progress.update(entry_leg=identity,entry_lead_reached=False)
                    if math.dist(position,lead)<=tol:progress['entry_lead_reached']=True
                    return tuple(entry) if progress['entry_lead_reached'] else lead
                return tuple(entry)
            connector=air_line_connector(work,progress,position)
            if connector is not None:return connector
            if (work['domain']=='WATER' and entry_heading is not None and
                    (not heading_ready or math.hypot(position[0]-leg['start'][0],
                        position[1]-leg['start'][1])>tol)):
                # Forward-only WATER guidance needs a run-in, not a demand to
                # rotate at zero surge. This is an entry intention; the local
                # executor must still observe/plan every connecting short part.
                lead=(leg['start'][0]-2.*math.cos(entry_heading),
                      leg['start'][1]-2.*math.sin(entry_heading),leg['start'][2])
                if progress.get('entry_leg')!=index:
                    progress.update(entry_leg=index,entry_lead_reached=False)
                if math.dist(position,lead)<=tol:progress['entry_lead_reached']=True
                return leg['start'] if progress['entry_lead_reached'] else lead
            return leg['start']
    def qualified(candidate,p,v,angle,medium):
        _,error=projection(p,candidate);desired=required_heading(work,candidate,p)
        return (medium==work['domain'] and error<=tol and
            math.sqrt(sum(x*x for x in v))<=work['speed_limit_mps'] and
            (candidate['kind']!='LOCAL' or math.sqrt(sum(x*x for x in v))<=.03) and
            (desired is None or abs(math.remainder(angle-desired,2*math.pi))<=work['heading_tolerance_rad']))
    if leg['kind']=='CONTOUR':
        index,arc,_,_,length=contour_location(legs,index,position)
        progress['active_leg']=index;leg=legs[index]
        note_contour_navigation(work,progress,index,stamp,position,mode,arc,length)
    if progress.get('residual',{}).get('phase')=='APPROACH':progress['previous']=None
    t,_=projection(position,leg);valid=qualified(leg,position,velocity,yaw,mode)
    previous=progress['previous'];row=(float(stamp),tuple(position),t,valid,tuple(velocity),yaw,mode)
    if leg['kind']=='LOCAL' and (not valid or previous is not None and not 0<stamp-previous[0]<=.75):
        progress['intervals'].pop(str(index),None);previous=None
    if previous and 0<stamp-previous[0]<=.75 and valid and previous[3]:
        dt=stamp-previous[0]
        if math.dist(position,previous[1])<=max(.03,work['speed_limit_mps']*dt*1.5):
            if leg['kind']=='CONTOUR':
                group=[n for n,l in enumerate(legs) if l['section']==leg['section']]
                offsets={};length=0.
                for n in group:offsets[n]=length;length+=legs[n]['length']
                before,start,_,_,_=contour_location(legs,index,previous[1])
                after,end,_,_,_=contour_location(legs,index,position)
                if (qualified(legs[before],previous[1],previous[4],previous[5],previous[6]) and
                        qualified(legs[after],position,velocity,yaw,mode)):
                    delta=math.remainder(end-start,length)
                    # Nearest-point projection must not bridge a disconnected
                    # branch or a missing arc after a sample jump.
                    if abs(delta)<=max(.05,3.*math.dist(position,previous[1])):
                        low,high=sorted((start,start+delta))
                        ranges=[(low,high)]
                        if low<0.:ranges=[(0.,high),(length+low,length)]
                        elif high>length:ranges=[(low,length),(0.,high-length)]
                        for lo,hi in ranges:
                            for n in group:
                                x=max(lo,offsets[n]);y=min(hi,offsets[n]+legs[n]['length'])
                                if y>x:
                                    progress['intervals'][str(n)]=interval_union(
                                        progress['intervals'].get(str(n),()),
                                        (x-offsets[n])/legs[n]['length'],(y-offsets[n])/legs[n]['length'])
            else:
                key=str(index)
                if leg['kind']=='LOCAL':
                    low=sum(b-a for a,b in progress['intervals'].get(key,()))
                    high=min(1.,low+dt/leg['length'])
                else:
                    low,high=sorted((previous[2],t));cap=min(.1,tol/leg['length'])
                    if previous[2]<=cap and math.dist(previous[1],leg['start'])<=tol:low=0.
                    if t>=1.-cap and math.dist(position,leg['end'])<=tol:high=1.
                progress['intervals'][key]=interval_union(progress['intervals'].get(key,()),low,high)
    progress['previous']=row
    def covered(n):return sum(b-a for a,b in progress['intervals'].get(str(n),()))>=1.-1e-8
    passed=(leg['kind']!='LOCAL' and valid and t>=1.-min(.1,tol/leg['length']) and
            math.dist(position,leg['end'])<=tol)
    next_index=None;connected=False
    if leg['kind']=='CONTOUR':
        group=[n for n,l in enumerate(legs) if l['section']==leg['section']]
        if all(covered(n) for n in group):next_index=group[-1]+1
    elif covered(index):
        next_index=index+1
        connected=(next_index<len(legs) and leg['kind']=='LINE' and legs[next_index]['kind']=='LINE'
                   and math.dist(leg['end'],legs[next_index]['start'])<1e-9)
    elif leg['kind']=='LINE' and residual and residual.get('kind')=='LINE':
        if math.dist(position,residual['exit'])<=tol:
            progress.pop('residual');residual=line_residual(work,progress,position)
            progress['approaching']=True;progress['previous']=None
    elif passed:
        progress['approaching']=True;progress['previous']=None
        if leg['kind']=='LINE':residual=line_residual(work,progress,position)
        progress.pop('entry_leg',None);progress.pop('entry_lead_reached',None)
    if next_index is not None:
        progress.pop('residual',None)
        progress.pop('air_connector',None)
        progress['active_leg']=next_index
        if next_index==len(legs):progress['complete']=True;progress['previous']=None
        else:
            new=legs[next_index];progress['approaching']=not connected
            if connected:
                new_t,_=projection(position,new)
                progress['previous']=(float(stamp),tuple(position),new_t,
                    qualified(new,position,velocity,yaw,mode),tuple(velocity),yaw,mode)
            else:progress['previous']=None
    progress['qualified']=sum(sum(b-a for a,b in values) for values in progress['intervals'].values())
    progress['fraction']=1. if progress['complete'] else min(1.,progress['qualified']/progress['total'])
    if progress['complete']:return legs[-1]['end']
    active=legs[progress['active_leg']]
    residual=progress.get('residual')
    if active['kind']=='LINE' and residual and residual.get('kind')=='LINE':
        return tuple(residual['entry'] if progress['approaching'] else residual['exit'])
    if not progress['approaching'] and active['kind']=='CONTOUR':
        target=residual_target(work,progress,position,velocity,yaw)
        if target is not None:return target
        return contour_lookahead(legs,progress['active_leg'],position)
    return active['start'] if progress['approaching'] else active['end']


def control_report(request_id,work,progress,producer,goal_id,stamp):
    if not progress['complete'] or progress['fraction']<1.-1e-8:
        raise ValueError('cannot report incomplete inspection work')
    event=dict(event_type='INSPECTION_CONTROL_REPORT',product_id=goal_id+':work:'+work['work_id'],
        request_id=request_id,work_id=work['work_id'],work_version=work['version'],
        object_id=work['object_id'],point_id=work['work_id'],producer=producer,goal_id=goal_id,
        generated_at=float(stamp),observed=True,required_bytes=1,
        result=dict(model='CONTROL_INSPECTION',dwell_s=0.,domain=work['domain'],
                    qualified_intervals=copy.deepcopy(progress['intervals']),
                    qualified_work_units=progress['qualified'],required_work_units=progress['total'],fraction=progress['fraction'],
                    measured_length_m=sum(l['length']*sum(b-a for a,b in progress['intervals'].get(str(i),())) for i,l in enumerate(progress['_legs']) if l['kind']!='LOCAL'),
                    qualified_hold_s=sum(l['length']*sum(b-a for a,b in progress['intervals'].get(str(i),())) for i,l in enumerate(progress['_legs']) if l['kind']=='LOCAL')))
    from .observation_coverage import message_wire_size
    while event['required_bytes']!=message_wire_size(event):
        event['required_bytes']=message_wire_size(event)
    return event


def definition_version(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,allow_nan=False).encode()).hexdigest()


def validate_control_report(work,event):
    """Verify complete normalized unions against the accepted definition."""
    if (event.get('event_type')!='INSPECTION_CONTROL_REPORT' or
        event.get('work_id')!=work['work_id'] or event.get('work_version')!=work['version'] or
        event.get('object_id')!=work['object_id'] or event.get('point_id')!=work['work_id'] or
        event.get('product_id')!=event.get('goal_id','')+':work:'+work['work_id']):
        raise ValueError('inspection report identity/definition mismatch')
    result=event['result'];legs=work_legs(work);intervals=result['qualified_intervals']
    if (result.get('model')!='CONTROL_INSPECTION' or result.get('domain')!=work['domain'] or
            set(intervals)!={str(i) for i in range(len(legs))}):
        raise ValueError('inspection report missing required legs')
    for values in intervals.values():
        if not isinstance(values,list) or not values:raise ValueError('missing qualified interval')
        if any(len(pair)!=2 or not all(math.isfinite(v) for v in pair) or not 0<=pair[0]<=pair[1]<=1 for pair in values):
            raise ValueError('invalid qualified interval bounds')
        merged=[]
        for a,b in values:merged=interval_union(merged,a,b)
        if sum(b-a for a,b in merged)<1.-1e-8:raise ValueError('inspection report has unfinished interval')
    expected_distance=sum(l['length'] for l in legs if l['kind']!='LOCAL')
    expected_hold=sum(l['length'] for l in legs if l['kind']=='LOCAL')
    for field,expected in [('qualified_work_units',len(legs)),('required_work_units',len(legs)),
                           ('measured_length_m',expected_distance),('qualified_hold_s',expected_hold),('fraction',1.)]:
        if not math.isfinite(result[field]) or abs(result[field]-expected)>1e-6*max(1.,expected):
            raise ValueError('inspection report inconsistent '+field)
    return True
