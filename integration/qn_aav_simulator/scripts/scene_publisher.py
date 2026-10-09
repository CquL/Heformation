#!/usr/bin/env python3
"""Simulation world, range observations, display and task-level transport.

The complete cloud is world truth for the legacy renderer and independent
evaluation. ONLINE_MAPPING planners consume only per-member survey_cloud rays
generated from actual poses here, not the complete scene description.

The legacy profile has one optional box. The five-platform profile declares
multiple SOLID boxes and separate FORBIDDEN/task metadata; only SOLID is sampled
as physical sensor-map geometry. The ONLINE_MAPPING geometric range model also
casts against the declared seabed. It does not model sonar propagation or
camera imagery. A valid no-hit ray is distinct from a missing message.

The box centre and size are defined once and used for both the sampled cloud and
the analytic clearance the verifier reports.
"""

from __future__ import annotations

import struct
import copy
import threading
import time
import json
import io
import math
from collections import deque

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from qn_aav_simulator.experiment_verdict import box_sample_points,StaticSceneGeometry

DEFAULT_TOPIC = "/scene/global_cloud"
TASK_SCENE_FIELDS = ('communication', 'communication_sites', 'selected_monitoring_area',
                     'observation_targets', 'transition_sites', 'rendezvous_sites')


class SceneTransport:
    """Declared finite-channel simulation, independent of task decisions/view.

    Raw physical states are used only here to simulate links. Mother-ship
    consumers receive products on the receipt topic, never this private ledger.
    Rates/ranges are the frozen experiment assumptions, not hardware claims.
    """
    def __init__(self,scene,frame,request_file):
        from nav_msgs.msg import Odometry
        from diagnostic_msgs.msg import DiagnosticArray
        from std_msgs.msg import String
        from qn_aav_simulator.task_line import load_request
        from qn_aav_simulator.observation_coverage import FiniteDelivery,ObstacleBox,communication_settings
        self.String=String;self.request=load_request(request_file);self.frame=frame
        self.request_file=request_file;self.used_request_ids={self.request.request_id}
        self.communication=communication_settings(scene)
        self.stage_service=(self.request.template_id in ('OFFSHORE_JOINT','WIND_INSPECTION','PLATFORM_PIPELINE_INSPECTION') and
            self.communication['model']=='FINITE_STAGE_SERVICE')
        self.task_service=(self.request.template_id in ('OFFSHORE_JOINT','WIND_INSPECTION','PLATFORM_PIPELINE_INSPECTION') and not self.stage_service)
        self.support_sites=tuple(scene.get('communication_sites',()))
        self.mother=tuple(scene.get('mother_ship_receiver_position',scene['mother_ship_position']))
        self.air_contact_m=max((float(site['mother_contact_m']) for site in self.support_sites),default=0.)
        if self.task_service and (not self.support_sites or self.air_contact_m<=0 or
                any(float(site['acoustic_contact_m'])<=0 for site in self.support_sites)):
            raise ValueError('task service needs declared positive contact geometry')
        self.obstacles=tuple(ObstacleBox(tuple(o['center']),tuple(o['size'])) for o in scene['objects'] if o['kind']=='SOLID')
        if any(box.blocks(self.mother,self.mother) for box in self.obstacles):
            raise ValueError('declared mother receiver lies inside a solid; set its exterior attachment position')
        self.lock=threading.Lock();self.states={};self.modes={};self.local_diagnostics={};self.events={}
        self.state_samples={};self.state_sample_history={};self.diagnostic_history={};self.pending_states={}
        self.peer_payloads={};self.peer_publishers={}
        self.scan_poses={};self.scan_sent={}
        self.survey_publishers={}
        if self.request.execution_mode in ('ONLINE_MAPPING','INSPECTION_CONTROL'):
            # Declared geometric range-sensor experiment, not a camera/sonar
            # hardware claim. Ray casting belongs on the simulator side only.
            import numpy as np
            self.np=np
            # Panoramic geometric range scan for this experiment; the existing
            # forward Swarm camera remains a separate navigation input.
            az,el=np.meshgrid(np.deg2rad(np.arange(-180.,180.,6.)),
                             np.deg2rad(np.arange(-90.,91.,6.)))
            self.scan_directions=np.stack((np.cos(el)*np.cos(az),np.cos(el)*np.sin(az),np.sin(el)),axis=-1).reshape(-1,3)
            self.scan_range=5.
            self.scan_seabed=float(scene['seabed_z_m'])
            self.observed_points={}
            self.observed_cloud=rospy.Publisher('/scene/observed_cloud',PointCloud2,queue_size=1,latch=True)
            self.scan_boxes=[(np.array(o['center'])-np.array(o['size'])/2,
                              np.array(o['center'])+np.array(o['size'])/2)
                             for o in scene['objects'] if o['kind']=='SOLID']
            for member in ('drone_0','drone_1','drone_2','usv','uuv'):
                prefix='/'+member+('_qn' if member.startswith('drone_') else '')
                self.survey_publishers[member]=rospy.Publisher(prefix+'/survey_cloud',PointCloud2,queue_size=1)
        self.last_time=math.floor(rospy.Time.now().to_sec()*10.)/10.;self.previous={}
        self.started_at=self.last_time
        self.delivery=None if self.task_service else FiniteDelivery(self.last_time)
        self.delivered=set()
        self.task_relayed={}
        self.relay_receipts=set()
        self.receipts=rospy.Publisher('/mother/received_products',String,queue_size=100)
        self.notifications=rospy.Publisher('/mother/received_notifications',String,queue_size=100)
        self.state_receipts=rospy.Publisher('/mother/received_states',String,queue_size=100)
        self.command_deliveries=rospy.Publisher('/mother/command_deliveries',String,queue_size=100)
        self.local_receipts=rospy.Publisher('/usv/received_products',String,queue_size=100,latch=True)
        # Passive evaluation view only. The runner never consumes relay truth.
        self.progress=rospy.Publisher('/scene/delivery_progress',String,queue_size=1,latch=True)
        self.transfer_events=rospy.Publisher('/scene/communication_transfers',String,queue_size=100)
        self.transfer_cursor=0
        self.last_progress=-1.
        self.subs=[]
        if self.stage_service:
            from traj_utils.msg import PolyTraj
            self.peer_type=PolyTraj
            self.peer_publishers={member:rospy.Publisher('/'+member+'_received_traj',PolyTraj,queue_size=10)
                for member in ('drone_0','drone_1','drone_2')}
            self.subs.append(rospy.Subscriber('/broadcast_traj_from_planner',PolyTraj,self.peer_trajectory,queue_size=100))
        for member in ('drone_0','drone_1','drone_2','usv','uuv'):
            prefix='/'+member+('_qn' if member.startswith('drone_') else '')
            source='/'+member+('_qn_aav' if member.startswith('drone_') else '')
            self.subs.extend((
                rospy.Subscriber(prefix+'/odometry',Odometry,lambda m,k=member:self.odom(k,m),queue_size=5),
                rospy.Subscriber(prefix+'/diagnostics',DiagnosticArray,lambda m,k=member:self.mode(k,m),queue_size=5),
                rospy.Subscriber(source+'/local_products',String,lambda m,k=member:self.produce(k,m),queue_size=100)))
        if not self.task_service:
            self.subs.append(rospy.Subscriber('/mother/command_requests',String,self.command,queue_size=100))
        self.subs.append(rospy.Subscriber('/mother/state_claim_requests',String,
            self.state_claim_request,queue_size=20))
        self.timer=rospy.Timer(rospy.Duration(.1),self.tick)
        if self.survey_publishers:
            self.scan_timer=rospy.Timer(rospy.Duration(.25),self.publish_scans)

    def peer_trajectory(self,message):
        from qn_aav_simulator.observation_coverage import DeliveryProduct
        source='drone_'+str(message.drone_id)
        if source not in self.peer_publishers:return
        encoded=io.BytesIO();message.serialize(encoded)
        now=rospy.Time.now().to_sec()
        with self.lock:
            for receiver in self.peer_publishers:
                if receiver==source:continue
                ident='peer:'+source+':'+str(message.traj_id)+':'+receiver
                if ident in self.delivery.products:continue
                self.peer_payloads[ident]=(receiver,copy.deepcopy(message))
                self.delivery.produce(ident,DeliveryProduct(source,receiver,len(encoded.getvalue()),now,True))

    def replace_request(self,request_file,task_scene):
        """Adopt a new business batch after the runner ends the old workers.

        The request-file parameter is the final commit written by the existing
        runner. Only its task ledger/contact geometry changes here: physical
        state, sensor history, simulation time and world obstacles stay alive.
        """
        from qn_aav_simulator.task_line import load_request
        from qn_aav_simulator.observation_coverage import FiniteDelivery
        request=load_request(request_file)
        from qn_aav_simulator.inspection_work import JOINT_TEMPLATES
        if request.template_id not in JOINT_TEMPLATES or self.request.template_id not in JOINT_TEMPLATES:
            raise ValueError('live request must retain the running template and sensor mode')
        sites=tuple(task_scene.get('communication_sites',()))
        air_contact=max((float(site['mother_contact_m']) for site in sites),default=0.)
        if self.task_service and (not sites or air_contact<=0 or
                any(float(site['acoustic_contact_m'])<=0 for site in sites)):
            raise ValueError('new task service needs declared positive contact geometry')
        with self.lock:
            if request.request_id in self.used_request_ids:
                raise ValueError('live request_id must be new; old receipts cannot be reused')
            now=rospy.Time.now().to_sec()
            self.request=request;self.request_file=request_file
            self.used_request_ids.add(request.request_id)
            self.support_sites=sites;self.air_contact_m=air_contact
            self.events.clear();self.delivered.clear();self.task_relayed.clear()
            self.started_at=now;self.last_time=now;self.last_progress=-1.
            self.previous={}
            if not self.task_service:self.delivery=FiniteDelivery(now)
        return request

    def odom(self,key,msg):
        if msg.header.frame_id!=self.frame:return
        p=msg.pose.pose.position
        with self.lock:
            if self.stage_service:
                from qn_aav_simulator.odometry import parse_standard_odometry
                try:
                    sample=parse_standard_odometry(msg,key)
                    self.state_samples[key]=sample
                    history=self.state_sample_history.setdefault(key,deque(maxlen=64))
                    if not history or sample.stamp>history[-1].stamp:history.append(sample)
                except ValueError:return
            rows=self.states.setdefault(key,deque(maxlen=64))
            stamp=msg.header.stamp.to_sec()
            if not rows or stamp>rows[-1][0]:rows.append((stamp,(p.x,p.y,p.z)))
            q=msg.pose.pose.orientation
            # Keep the actual acquisition Time losslessly. Integer-nanosecond
            # model clocks cannot round-trip through epoch-scale float seconds.
            self.scan_poses[key]=(msg.header.stamp,(p.x,p.y,p.z),(q.w,q.x,q.y,q.z))

    def publish_scans(self,_event):
        """First intersection or explicit valid max-range miss for each ray.

        XYZI uses intensity 1=hit, 0=valid miss. Missing/stale odometry produces
        no scan, never a fabricated clear volume. Consumers use the same-stamp
        actual origin and keep untraversed cells unknown (OctoMap semantics).
        """
        from sensor_msgs import point_cloud2
        np=self.np
        with self.lock:poses=dict(self.scan_poses)
        fields=[PointField(name=n,offset=4*i,datatype=PointField.FLOAT32,count=1)
                for i,n in enumerate(('x','y','z','intensity'))]
        for member,(acquisition_stamp,origin,quat) in poses.items():
            stamp_ns=acquisition_stamp.to_nsec()
            if member not in self.survey_publishers or stamp_ns<=self.scan_sent.get(member,-1):continue
            if not 0<=(rospy.Time.now()-acquisition_stamp).to_sec()<=.5:continue
            w,x,y,z=quat
            rotation=np.array(((1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)),
                (2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)),
                (2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y))))
            directions=self.scan_directions@rotation.T;origin=np.asarray(origin)
            distance=np.full(len(directions),self.scan_range);hits=np.zeros(len(directions))
            # The declared seabed is a physical first return, even though the
            # aerial planner's compact obstacle cloud omits the seabed plane.
            downward=directions[:,2]<-1e-10
            floor_distance=np.full(len(directions),np.inf)
            floor_distance[downward]=(self.scan_seabed-origin[2])/directions[downward,2]
            floor_hit=(floor_distance>0.) & (floor_distance<distance)
            distance[floor_hit]=floor_distance[floor_hit];hits[floor_hit]=1.
            for low,high in self.scan_boxes:
                with np.errstate(divide='ignore',invalid='ignore'):
                    a=(low-origin)/directions;b=(high-origin)/directions
                parallel=np.abs(directions)<1e-10
                outside=parallel & ((origin<low)|(origin>high))
                near=np.max(np.where(parallel,-np.inf,np.minimum(a,b)),axis=1)
                far=np.min(np.where(parallel,np.inf,np.maximum(a,b)),axis=1)
                valid=(~np.any(outside,axis=1)) & (far>=np.maximum(near,0.)) & (near>0.) & (near<distance)
                distance[valid]=near[valid];hits[valid]=1.
            ends=origin+directions*distance[:,None]
            payload=np.column_stack((ends,hits)).astype(np.float32)
            cloud=PointCloud2(header=Header(frame_id=self.frame,stamp=acquisition_stamp),
                height=1,width=len(payload),fields=fields,is_bigendian=False,point_step=16,
                row_step=16*len(payload),data=payload.tobytes(),is_dense=False)
            self.survey_publishers[member].publish(cloud);self.scan_sent[member]=stamp_ns
            for point in ends[hits>0]:
                key=tuple(int(math.floor(float(v)/.25)) for v in point)
                self.observed_points[key]=tuple(float(v) for v in point)
        if self.observed_points:
            points=np.asarray(list(self.observed_points.values()),dtype=np.float32)
            self.observed_cloud.publish(PointCloud2(header=Header(frame_id=self.frame,stamp=rospy.Time.now()),
                height=1,width=len(points),fields=fields[:3],is_bigendian=False,point_step=12,
                row_step=12*len(points),data=points.tobytes(),is_dense=False))

    def mode(self,key,msg):
        from qn_aav_simulator.platform_execution import actual_mode
        values={v.key:v.value for s in msg.status for v in s.values}
        mode=values.get('actual_mode')
        if mode is None and 'medium_flag' in values:mode=actual_mode(float(values['medium_flag']))
        if mode not in ('AIR','WATER','SURFACE','TRANSITION'):return
        with self.lock:
            rows=self.modes.setdefault(key,deque(maxlen=16))
            stamp=msg.header.stamp.to_sec()
            if not rows or stamp>rows[-1][0]:
                rows.append((stamp,mode));self.local_diagnostics[key]=(stamp,values)
                self.diagnostic_history.setdefault(key,deque(maxlen=64)).append((stamp,values))

    def produce(self,member,msg):
        from qn_aav_simulator.observation_coverage import DeliveryProduct
        try:
            event=json.loads(msg.data);ident=event['product_id']
            points={p.point_id for r in self.request.regions for p in r.interest_points}
            inspection=event.get('event_type')=='INSPECTION_CONTROL_REPORT'
            if inspection:
                work=next((w for w in self.request.work_items if w['work_id']==event.get('work_id')),None)
                if work is None:raise ValueError('unknown current-request inspection work')
                from qn_aav_simulator.inspection_work import validate_control_report
                validate_control_report(work,event)
                points.add(work['work_id'])
            terminal=event.get('event_type')=='OBSERVATION_TERMINAL'
            action_terminal=event.get('event_type')=='ACTION_TERMINAL'
            if terminal:
                declared=event['point_ids'];observed=event['observed_ids']
                if (not isinstance(declared,list) or not declared or len(declared)!=len(set(declared)) or
                        not set(declared)<=points or not isinstance(observed,list) or
                        len(observed)!=len(set(observed)) or not set(observed)<=set(declared) or
                        ident!=event['goal_id']+':terminal'):
                    raise ValueError('invalid observation terminal report')
            if action_terminal and (
                    ident!=event['goal_id']+':action_terminal:'+member or
                    event['terminal_state'] not in ('SUCCEEDED','CANCELED','ABORTED') or
                    any(type(event[name]) is not bool for name in
                        ('task_completed','terminal_verified','resource_locked')) or
                    not isinstance(event['reason'],str)):
                raise ValueError('invalid Action terminal notice')
            digest=event.get('terminal_state_digest')
            if action_terminal and digest is not None and (
                    not isinstance(digest,str) or len(digest)!=64 or
                    any(char not in '0123456789abcdef' for char in digest)):
                raise ValueError('invalid native terminal state digest')
            if (not isinstance(ident,str) or not ident or event['producer']!=member or
                    event['request_id']!=self.request.request_id or
                    (not terminal and not action_terminal and (event['point_id'] not in points or
                     event['observed'] is not True or
                     not self.task_service and not self.stage_service and event['required_bytes']!=32*1024))):
                raise ValueError('product identity or declared size mismatch')
            generated=event['generated_at'];now=rospy.Time.now().to_sec()
            start_time=self.started_at if self.task_service else self.delivery.start_time
            if not math.isfinite(generated) or not start_time<=generated<=now:
                raise ValueError('product generation outside current run')
            with self.lock:
                # Validation above may overlap a request switch. Recheck at
                # insertion so an old Goal's delayed product cannot enter the
                # new batch even when its payload happened to pass earlier.
                if event['request_id']!=self.request.request_id:
                    raise ValueError('product belongs to the previous request')
                if ident in self.events:
                    if self.events[ident]!=event:raise ValueError('conflicting duplicate product')
                    return
                if self.stage_service and action_terminal:
                    sample=self.state_samples.get(member)
                    diagnostic=self.local_diagnostics.get(member)
                    if sample is not None and diagnostic is not None and 0<=now-sample.stamp<=.25:
                        event=dict(event,local_state=dict(position=sample.position,velocity=sample.velocity,
                            actual_mode=next((row[1] for row in reversed(self.modes.get(member,()))
                                if row[0]<=now),'UNKNOWN'),diagnostics=diagnostic[1],generated_at=sample.stamp))
                self.events[ident]=event
                # Notification and summary use the same capacity. A received
                # notice is explicitly not the 32 KiB business product.
                if not self.task_service:
                    size=4+len(json.dumps(event,allow_nan=False).encode('utf-8'))
                    if self.stage_service:
                        self.delivery.produce(('notice:' if terminal or action_terminal else 'data:')+ident,
                            DeliveryProduct(member,'mother',size,generated,True))
                    else:
                        self.delivery.produce('notice:'+ident,DeliveryProduct(member,'mother',size,generated,True))
                    if not self.stage_service and not terminal and not action_terminal:
                        self.delivery.produce('data:'+ident,DeliveryProduct(member,'mother',32*1024,generated,True))
        except (ValueError,KeyError,TypeError) as error:
            rospy.logerr_throttle(2.,'Rejected local product: %s',str(error))

    def command(self,msg):
        from qn_aav_simulator.observation_coverage import DeliveryProduct
        try:
            event=json.loads(msg.data)
            ident='command:'+event['command_id']
            generated=event['generated_at'];now=rospy.Time.now().to_sec()
            digest=event['goal_digest']
            if (event['request_id']!=self.request.request_id or
                    event['receiver'] not in ('drone_0','drone_1','drone_2','usv','uuv') or
                    not isinstance(event['execution_id'],str) or not event['execution_id'] or
                    not isinstance(event['command_id'],str) or not event['command_id'] or
                    type(event['plan_revision']) is not int or event['plan_revision']<0 or
                    type(event['required_bytes']) is not int or event['required_bytes']<=0 or
                    not isinstance(digest,str) or len(digest)!=64 or
                    any(char not in '0123456789abcdef' for char in digest) or
                    not math.isfinite(generated) or not self.delivery.start_time<=generated<=now):
                raise ValueError('invalid mother command identity, size or generation time')
            with self.lock:
                if event['request_id']!=self.request.request_id:
                    raise ValueError('command belongs to the previous request')
                if ident in self.events:
                    if self.events[ident]!=event:raise ValueError('conflicting duplicate command')
                    return
                self.events[ident]=event
                self.delivery.produce(ident,DeliveryProduct('mother',event['receiver'],
                    event['required_bytes'],generated,True))
        except (ValueError,KeyError,TypeError) as error:
            rospy.logerr_throttle(2.,'Rejected mother command: %s',str(error))

    def state_claim_request(self,msg):
        """A mother request reaches the local platform only through finite delivery."""
        from qn_aav_simulator.observation_coverage import DeliveryProduct
        try:
            event=json.loads(msg.data);member=event['receiver'];ident=event['claim_id']
            generated=event['generated_at'];now=rospy.Time.now().to_sec()
            if (event['request_id']!=self.request.request_id or
                    member not in ('drone_0','drone_1','drone_2','usv','uuv') or
                    not isinstance(ident,str) or len(ident)!=64 or
                    any(char not in '0123456789abcdef' for char in ident) or
                    type(generated) not in (int,float) or not math.isfinite(generated) or
                    not (self.started_at if self.task_service else self.delivery.start_time)<=generated<=now):
                raise ValueError('invalid finite state-claim request')
            with self.lock:
                if event['request_id']!=self.request.request_id:
                    raise ValueError('state claim belongs to the previous request')
                key='claim-request:'+ident
                if key in self.events:
                    if self.events[key]!=event:raise ValueError('conflicting state-claim request')
                    return
                self.events[key]=event
                if not self.task_service:
                    self.delivery.produce(key,DeliveryProduct('mother',member,
                        4+len(msg.data.encode('utf-8')),generated,True))
            if self.task_service:
                threading.Thread(target=self.capture_state_claim,args=(event,),daemon=True).start()
        except (ValueError,KeyError,TypeError) as error:
            rospy.logerr_throttle(2.,'Rejected state-claim request: %s',str(error))

    def capture_state_claim(self,event):
        """Read one local claim after its request arrives; uplink uses the same channel."""
        from qn_aav_simulator.observation_coverage import DeliveryProduct
        member=event['receiver'];digest=None;model_time=None;stamp=rospy.Time.now().to_sec()
        native_state=None;local_goal=None;local_locked=None
        if member in ('drone_0','drone_1','drone_2','usv','uuv'):
            try:
                from std_srvs.srv import Trigger
                service='/'+member+('_qn_aav' if member.startswith('drone_') else '')+'/state_digest'
                rospy.wait_for_service(service,timeout=.25)
                reply=rospy.ServiceProxy(service,Trigger)()
                if not reply.success:return
                local=json.loads(reply.message)
                if local.get('agent_id')!=member:return
                digest=local['digest'];model_time=float(local['model_time_s'])
                native_state=local.get('native_state')
                local_goal=local.get('active_goal_id')
                local_locked=local.get('resource_locked')
                stamp=float(local['ros_stamp_s'])
                if len(digest)!=64 or any(char not in '0123456789abcdef' for char in digest):return
            except (ValueError,KeyError,TypeError,rospy.ROSException,rospy.ServiceException):
                return
        with self.lock:
            # A state-digest service call may finish after its request has been
            # retired. Do not publish or store that reply in the next batch.
            if event['request_id']!=self.request.request_id:return
            sample=next((row for row in reversed(self.states.get(member,()))
                         if row[0]<=stamp),None)
            diagnostic=self.local_diagnostics.get(member)
            if (sample is None or diagnostic is None or
                    not 0<=stamp-sample[0]<=.25 or
                    not 0<=stamp-diagnostic[0]<=.25):return
            values=diagnostic[1]
            if model_time is None:
                try:model_time=float(values['model_time_s'])
                except (KeyError,TypeError,ValueError):return
            if not math.isfinite(model_time):return
            claim=dict(event_type='STATE_CLAIM',product_id=event['claim_id']+':state',
                claim_id=event['claim_id'],request_id=event['request_id'],producer=member,
                generated_at=stamp,position=sample[1],actual_mode=values.get('actual_mode'),
                model_time_s=model_time,terminal_state_digest=digest,
                reference_source=values.get('reference_source'),
                active_goal_id=(local_goal if local_goal is not None else
                    values.get('active_goal_id','')),
                resource_locked=str(local_locked if local_locked is not None else
                    values.get('resource_locked',values.get('platform_resource_locked','false'))).lower())
            if native_state is not None:claim['native_state']=native_state
            ident=claim['product_id'];encoded=json.dumps(claim,allow_nan=False)
            self.events[ident]=claim
            if self.task_service:
                self.delivered.add(ident)
            else:
                self.delivery.produce('notice:'+ident,DeliveryProduct(member,'mother',
                    4+len(encoded.encode('utf-8')),stamp,True))
        if self.task_service:
            self.notifications.publish(self.String(data=json.dumps(
                dict(claim,received_at=rospy.Time.now().to_sec()),allow_nan=False)))

    def queue_state_updates(self,now,states):
        from qn_aav_simulator.observation_coverage import DeliveryProduct,message_wire_size
        for member in states:
            if member=='mother':continue
            sample=next((row for row in reversed(self.state_sample_history.get(member,())) if row.stamp<=now),None)
            diagnostic=next((row for row in reversed(self.diagnostic_history.get(member,())) if row[0]<=now),None)
            if sample is None or diagnostic is None or not 0<=now-sample.stamp<=.25:continue
            values=diagnostic[1]
            signature=tuple(str(values.get(key,'')) for key in
                ('actual_mode','active_goal_id','pending_goal_id','resource_locked','platform_resource_locked','reference_active'))
            previous=self.pending_states.get(member)
            if previous:
                old_key,old_signature=previous
                old=self.delivery.products.get(old_key)
                if old is not None and old.received_at is None:
                    sent=any(key==old_key and prefix>0 for (key,destination),prefix in self.delivery.committed_prefixes.items())
                    if sent and old_signature==signature:continue
                    if not sent:
                        if old.generated_at>self.last_time-(now-self.last_time):continue
                        self.delivery.products.pop(old_key,None)
                        self.events.pop(old_key,None)
            ident='state:'+member+':'+str(sample.stamp)
            if ident in self.delivery.products:continue
            event=dict(event_type='PLATFORM_STATE',product_id=ident,request_id=self.request.request_id,
                producer=member,generated_at=max(sample.stamp,diagnostic[0]),position=sample.position,velocity=sample.velocity,
                actual_mode=states[member][1],diagnostics=values)
            self.events[ident]=event
            self.delivery.produce(ident,DeliveryProduct(member,'mother',message_wire_size(event),event['generated_at'],True))
            self.pending_states[member]=(ident,signature)

    def tick(self,_):
        if self.task_service:
            return self.tick_task_service()
        from qn_aav_simulator.observation_coverage import declared_delivery_channels
        now=math.floor(rospy.Time.now().to_sec()*10.)/10.;out=[];requests=[];progress=None;transfers=[]
        with self.lock:
            if now<=self.last_time:return
            states={'mother':(self.mother,'SURFACE')}
            for member,rows in self.states.items():
                stamp,pos=next((r for r in reversed(rows) if r[0]<=now),(-1.,None))
                mode_stamp,mode=next((r for r in reversed(self.modes.get(member,())) if r[0]<=now),(-1.,'UNKNOWN'))
                if 0<=now-stamp<=.25 and 0<=now-mode_stamp<=.25:states[member]=(pos,mode)
            continuous=now-self.last_time<=.25
            if self.stage_service:self.queue_state_updates(now,states)
            # No capacity is credited across a missed observation interval.
            delays={}
            channels=declared_delivery_channels(self.delivery.products,self.previous,states,
                self.obstacles,continuous,self.communication if self.stage_service else None,delays)
            outages=rospy.get_param_cached('/scene/communication_outages',[])
            elapsed=now-self.started_at
            for name,(rate,flows) in tuple(channels.items()):
                channels[name]=(rate,[(key,source,destination,available and not any(
                    entry['start_s']<=elapsed<entry['end_s'] and
                    entry.get('channel',name)==name and
                    entry.get('sender',source)==source and entry.get('receiver',destination)==destination
                    for entry in outages)) for key,source,destination,available in flows])
            receipts=self.delivery.advance_all(now,channels,delays)
            if self.stage_service:
                transfers=[dict(event,request_id=self.request.request_id,
                    sender_position=states[event['sender']][0],receiver_position=states[event['receiver']][0])
                    for event in self.delivery.transfers[self.transfer_cursor:]]
                self.transfer_cursor=len(self.delivery.transfers)
            for ident in receipts:
                kind,key=ident.split(':',1)
                if kind=='claim-request':requests.append(self.events[ident])
                elif kind=='command':out.append((self.command_deliveries,dict(self.events[ident],received_at=now)))
                elif kind=='state':out.append((self.state_receipts,dict(self.events[ident],received_at=now)))
                elif kind=='peer':
                    receiver,payload=self.peer_payloads.pop(ident)
                    out.append((self.peer_publishers[receiver],payload))
                else:out.append((self.notifications if kind=='notice' else self.receipts,
                                 dict(self.events[key],received_at=now)))
            self.last_time=now;self.previous=states
            if self.stage_service:
                for ident,product in self.delivery.products.items():
                    if not ident.startswith('data:') or ident in self.relay_receipts:continue
                    if product.received_prefix.get('usv',0.)>=product.required_bytes:
                        self.relay_receipts.add(ident)
                        out.append((self.local_receipts,dict(self.events[ident.split(':',1)[1]],
                            receiver='usv',received_at=now)))
            if now-self.last_progress>=.5:
                progress=dict(at_ros_s=now,request_id=self.request.request_id,
                    scope='FINITE_STAGE_SERVICE' if self.stage_service else 'INDEPENDENT_TRANSPORT_VIEW',
                    model='FINITE_STAGE_SERVICE' if self.stage_service else 'FINITE_DECLARED_EXPERIMENT',
                    started_at=self.started_at,
                    support_active=bool(states.get('usv') and any(
                        member!='usv' and state[1]=='WATER' and math.dist(state[0],states['usv'][0])<=self.communication['acoustic_range_m']
                        for member,state in states.items())),
                    queued_bytes=sum(max(0.,product.required_bytes-product.received_prefix.get(product.receiver,0.))
                        for product in self.delivery.products.values()),
                    received_state_age_s={member:now-self.events[key]['generated_at']
                        for member,(key,signature) in self.pending_states.items()
                        if key in self.delivery.products and self.delivery.products[key].received_at is not None},
                    products=[
                    dict(point_id=self.events[key]['point_id'],
                         required_bytes=p.required_bytes,
                         relay_bytes=p.received_prefix.get('usv',0.),
                         mother_bytes=p.received_prefix.get('mother',0.),
                         producer=p.producer,generated_at=p.generated_at,
                         received_at=p.received_at)
                    for ident,p in self.delivery.products.items() if ident.startswith('data:')
                    for key in [ident.split(':',1)[1]]])
                self.last_progress=now
        if progress is not None:
            self.progress.publish(self.String(data=json.dumps(progress,allow_nan=False)))
        for transfer in transfers:self.transfer_events.publish(self.String(data=json.dumps(transfer,allow_nan=False)))
        for publisher,event in out:
            publisher.publish(self.String(data=json.dumps(event,allow_nan=False)) if isinstance(event,dict) else event)
        for request in requests:
            threading.Thread(target=self.capture_state_claim,args=(request,),daemon=True).start()

    def tick_task_service(self):
        current_sites=tuple(rospy.get_param_cached('/scene/communication_sites',self.support_sites))
        from qn_aav_simulator.observation_coverage import task_service_ready
        now=rospy.Time.now().to_sec();out=[]
        with self.lock:
            if now<=self.last_time:return
            states={}
            for member,rows in self.states.items():
                stamp,pos=next((row for row in reversed(rows) if row[0]<=now),(-1.,None))
                mode_stamp,mode=next((row for row in reversed(self.modes.get(member,()))
                    if row[0]<=now),(-1.,'UNKNOWN'))
                if 0<=now-stamp<=.25 and 0<=now-mode_stamp<=.25:
                    states[member]=(pos,mode)
            self.support_sites=current_sites
            supported=task_service_ready(states,self.support_sites)
            usv=states.get('usv')
            site=next((entry for entry in self.support_sites if usv and usv[1]=='SURFACE' and
                math.dist(usv[0],entry['position'])<=entry['radius_m']),None)
            for ident,event in self.events.items():
                if 'product_id' not in event:continue
                if ident in self.delivered or event['generated_at']>now:continue
                notification=event.get('event_type') in ('OBSERVATION_TERMINAL','ACTION_TERMINAL')
                if not notification:
                    producer=event.get('producer','')
                    if producer=='uuv' or event.get('result',{}).get('domain')=='WATER':
                        source=states.get(producer)
                        if (ident not in self.task_relayed and site and source and
                                source[1] in ('WATER','AIR') and
                                math.dist(usv[0],source[0])<=site['acoustic_contact_m']):
                            self.task_relayed[ident]=now
                        if ident not in self.task_relayed or now<=self.task_relayed[ident]:
                            continue
                    else:
                        source=states.get(producer)
                        direct=(source is not None and source[1]=='AIR' and
                                math.dist(source[0],self.mother)<=self.air_contact_m)
                        # The declared task-level support is a shared RF
                        # service for every airborne producer, not a UUV-only
                        # resource.  Radio packet scheduling remains outside
                        # this first-version abstraction.
                        relayed=(source is not None and source[1]=='AIR' and site is not None and
                                 math.dist(source[0],usv[0])<=self.air_contact_m)
                        if not (direct or relayed):
                            continue
                self.delivered.add(ident)
                out.append((self.notifications if notification else self.receipts,
                    dict(event,received_at=now)))
            self.last_time=now
            progress=dict(at_ros_s=now,request_id=self.request.request_id,
                scope='TASK_SERVICE',support_active=supported,
                products=[dict(point_id=event['point_id'],received=ident in self.delivered)
                    for ident,event in self.events.items() if event.get('point_id')])
        self.progress.publish(self.String(data=json.dumps(progress,allow_nan=False)))
        for publisher,event in out:
            publisher.publish(self.String(data=json.dumps(event,allow_nan=False)))


class SceneView:
    """Optional passive RViz view; never publishes control or task completion.

    Odometry is independent evaluation truth, NOT mother-ship received state.
    Detailed action state belongs in the existing dashboard, not scene labels.
    """
    def __init__(self, scene, frame, request_id=None):
        from visualization_msgs.msg import Marker, MarkerArray
        from nav_msgs.msg import Odometry
        self.Marker, self.MarkerArray = Marker, MarkerArray
        self.scene, self.frame = scene, frame
        self.lock = threading.Lock()
        self.members = ('drone_0', 'drone_1', 'drone_2', 'usv', 'uuv')
        self.colours = ((0.,.68,.84),(.49,.20,.92),(.48,.53,.61),(0.,.64,.41),(.93,.67,0.))
        self.poses = {}
        self.trails = {key: deque(maxlen=2400) for key in self.members}
        self.last_sample = {}
        self.subs = []
        self.publisher = rospy.Publisher('/scene/view', MarkerArray, queue_size=1, latch=True)
        for i, key in enumerate(self.members):
            prefix = '/' + key + ('_qn' if i < 3 else '')
            self.subs.append(rospy.Subscriber(prefix+'/odometry', Odometry,
                lambda msg, k=key: self.odom(k, msg), queue_size=1))
        self.last_publish = 0.
        self.marker_keys = None
        self.cooperative=bool(rospy.get_param('/mission/request_file',''))
        self.request_id=request_id
        self.received_points=set()
        if self.cooperative:
            from std_msgs.msg import String
            def received(msg):
                try:
                    event=json.loads(msg.data)
                    with self.lock:
                        if event.get('request_id')==self.request_id:
                            self.received_points.add(event['point_id'])
                except (ValueError,KeyError,TypeError):
                    return
            self.subs.append(rospy.Subscriber('/mother/received_products',String,received,queue_size=10))

    def replace_request(self,request,task_scene):
        with self.lock:
            for key in TASK_SCENE_FIELDS:
                if key in task_scene:self.scene[key]=copy.deepcopy(task_scene[key])
                else:self.scene.pop(key,None)
            self.request_id=request.request_id
            self.received_points.clear()
        # The next full marker snapshot replaces matching IDs and deletes IDs
        # no longer present. Keep marker_keys so old region marks are removed;
        # keep real odometry trails across successive business requests.
        self.last_publish=0.

    def odom(self, key, msg):
        if msg.header.frame_id != self.frame:
            return  # no implicit coordinate conversion in a viewer
        now = time.monotonic()
        with self.lock:
            self.poses[key] = (msg.pose.pose, now)
            if now-self.last_sample.get(key, 0.) >= .25:
                self.trails[key].append(msg.pose.pose.position)
                self.last_sample[key] = now

    def publish(self):
        now = time.monotonic()
        if now-self.last_publish < .25:
            return
        self.last_publish = now
        M = self.Marker
        markers = []
        counters = {}
        def add(ns, kind, position, scale, colour, text=''):
            marker = M()
            marker.header.frame_id = self.frame
            marker.header.stamp = rospy.Time.now()
            marker.ns, marker.id, marker.type = ns, counters.get(ns,0), kind
            counters[ns]=marker.id+1
            marker.pose.orientation.w = 1.
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = position
            marker.scale.x, marker.scale.y, marker.scale.z = scale
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = colour
            marker.text = text
            markers.append(marker)
            return marker
        def label(ns, p, text, size=.55, colour=(.95,.97,1.,1.)):
            return add(ns, M.TEXT_VIEW_FACING, p, (0.,0.,size), colour, text)
        # Finite display extents only: they do not create a navigation boundary.
        extent=self.scene.get('display_extent',{'center':[-9.,0.],'size':[62.,32.]})
        cx,cy=extent['center'];sx,sy=extent['size']
        add('water', M.CUBE, (cx,cy,self.scene['surface_z_m']), (sx,sy,.025), (.02,.35,.55,.28))
        add('seabed', M.CUBE, (cx,cy,self.scene['seabed_z_m']), (sx,sy,.08), (.06,.28,.34,.90))
        label('legend', (-4.,21.,6.), self.scene.get('scenario_label','五平台 · 实际状态与障碍'), 1.5)
        if self.scene.get('online_mapping'):
            label('map_legend',(-4.,21.,4.4),'共用部署区 · 用户选择作业区域 · 共用规划与控制链路',.72)
        label('water_label', (18.,12.,.2), '海面', .65)
        label('bed_label', (18.,12.,-5.5), '海底', .65)
        for item in self.scene.get('objects', []):
            p, size = item['center'], item['size']
            colour = ((.65,.65,.7,.95) if item.get('appearance')=='quay' else
                      (.9,.2,.25,.18) if item['kind']=='FORBIDDEN' else (.65,.65,.7,.95))
            appearance=item.get('appearance','box')
            assets=self.scene.get('visual_assets')
            if appearance=='proxy' and assets:
                continue  # detailed model below; the proxy remains in sensing/safety
            if appearance=='mother_ship' and assets:
                continue  # rendered once at the declared vessel origin below
            if appearance=='rock' and assets:
                body=add('geometry',M.MESH_RESOURCE,p,tuple(v/2 for v in size),(.48,.5,.48,1.))
                body.mesh_resource='file://'+assets+'/rock.dae'
            elif appearance=='jetty':
                top=p[2]+size[2]/2
                add('geometry',M.CUBE,(p[0],p[1],top-.15),(size[0],size[1],.3),(.58,.42,.27,1.))
                # Timber slats and support piles stay within the declared box.
                for j in range(max(1,int(size[1]/.5))):
                    y=p[1]-size[1]/2+.2+j*.5
                    add('geometry',M.CUBE,(p[0],y,top-.01),(size[0],.035,.015),(.25,.19,.12,1.))
                for x in (p[0]-size[0]/2+.2,p[0]+size[0]/2-.2):
                    for y in (p[1]-size[1]/2+.2,p[1]+size[1]/2-.2):
                        add('geometry',M.CYLINDER,(x,y,p[2]-.15),(.3,.3,size[2]-.3),(.42,.38,.29,1.))
            else:
                add('geometry', M.CUBE, p, size, colour)
                if appearance=='quay':
                    top=p[2]+size[2]/2
                    for j in range(int(size[0]/2)):
                        x=p[0]-size[0]/2+1+j*2
                        add('geometry',M.CUBE,(x,p[1],top-.02),(.035,size[1],.02),(.25,.28,.3,1.))
            name = {'pier':'码头','rock':'岩石','coast_rock':'尾段障碍','quay':'岸壁'}.get(item['id'],
                '栈桥' if appearance=='jetty' else '礁石' if appearance=='rock' else '禁入区' if item['kind']=='FORBIDDEN' else '障碍')
            if item.get('label',True):label('geometry_label', (p[0],p[1],p[2]+size[2]/2+.6), name, .65)
        assets=self.scene.get('visual_assets')
        if assets:
            for model in self.scene.get('visual_models',()):
                if model['model'] in ('aav','command_vessel'):continue
                body=add('facilities',M.MESH_RESOURCE,model['position'],(1.,1.,1.),(1.,1.,1.,1.))
                body.mesh_resource='file://'+assets+'/'+model['mesh']
                body.mesh_use_embedded_materials=True
        from geometry_msgs.msg import Point
        for area in self.scene.get('business_areas',()):
            x,y=area['center'];radius=float(area['radius_m']);colour=tuple(area['color'])
            circle=add('business_ranges',M.LINE_LIST,(0.,0.,0.),(.055,0.,0.),colour+(.9,))
            for i in range(64):
                if i%2:continue
                for t in (2*math.pi*i/64,2*math.pi*(i+1)/64):
                    circle.points.append(Point(x=x+radius*math.cos(t),y=y+radius*math.sin(t),z=.10))
            label('business_label',(x,y+radius+1.,1.8),area['label'],1.05,colour+(1.,))
            label('business_detail',(x,y+radius+1.,.6),area['detail'],.6)
            for facility in area.get('facilities',()):
                p=facility['position'];label('facility_name',(p[0],p[1],.5),facility['label'],.5,colour+(1.,))
        for model in self.scene.get('visual_models',()):
            if model['model']=='seabed_pipeline':
                source=next(s for s in self.scene['world_models'] if s['id']==model['id'])
                for i,p in enumerate(source['points'][1:]):
                    label('pipeline_segment',(p[0],p[1],p[2]+.9),'管段 '+str(i+1),.55,(1.,.78,.27,1.))
        if self.scene.get('world_models'):
            label('port_name',(-44.8,7.,5.4),'岸边指挥中心',.6)
            label('deployment',(-30.,3.,1.8),'母船 / 共同部署区\n3 × AAV · 1 × USV · 1 × UUV',.58)
            sites=self.scene['return_sites'].values()
            xs=[site['position'][0] for site in sites];ys=[site['position'][1] for site in self.scene['return_sites'].values()]
            for obj in self.scene['objects']:
                if obj.get('facility')=='command_vessel':
                    xs.extend((obj['center'][0]-obj['size'][0]/2,obj['center'][0]+obj['size'][0]/2))
                    ys.extend((obj['center'][1]-obj['size'][1]/2,obj['center'][1]+obj['size'][1]/2))
            corners=((min(xs)-1.5,min(ys)-2.),(max(xs)+1.5,min(ys)-2.),
                     (max(xs)+1.5,max(ys)+1.),(min(xs)-1.5,max(ys)+1.))
            outline=add('deployment_range',M.LINE_LIST,(0.,0.,0.),(.06,0.,0.),(.55,.86,1.,.8))
            for a,b in zip(corners,corners[1:]+corners[:1]):
                for j in range(0,20,2):
                    for t in (j/20,(j+1)/20):outline.points.append(Point(x=a[0]+t*(b[0]-a[0]),y=a[1]+t*(b[1]-a[1]),z=.12))
            route=add('safe_channel',M.ARROW,(0.,0.,0.),(.16,.32,.55),(.6,.86,1.,1.))
            route.points=[Point(x=30.,y=17.,z=.3),Point(x=34.,y=20.,z=.3)]
            label('channel_label',(30.,19.,1.),'通往外海（航道示意）',.5)
        air_targets=[item for item in self.scene.get('observation_targets', [])
                     if item['domain']=='AIR']
        deep_targets=[item for item in self.scene.get('observation_targets', [])
                      if item['id'].startswith('deep_swath_')]
        selected=self.scene.get('selected_monitoring_area')
        if selected and selected.get('shape')=='CIRCLE':
            center=selected['center'];diameter=2*selected['radius_m']
            add('joint_survey_area',M.CYLINDER,(center[0],center[1],-.06),
                (diameter,diameter,.04),(.25,.85,1.,.12))
            label('joint_survey_label',(center[0],center[1]+selected['radius_m']+1.,1.3),
                  '用户选择的联合监测区',.7)
        if len(air_targets)>1:
            all_targets=self.scene.get('observation_targets', [])
            joint_x=[item['position'][0] for item in all_targets]
            joint_y=[item['position'][1] for item in all_targets]
            selected=self.scene.get('selected_monitoring_area')
            if selected and selected.get('shape')=='CIRCLE':
                pass  # selected region is drawn independently of legacy points
            else:
                add('joint_survey_area',M.CUBE,
                    ((min(joint_x)+max(joint_x))/2,(min(joint_y)+max(joint_y))/2,-.06),
                    (max(joint_x)-min(joint_x)+2.,max(joint_y)-min(joint_y)+2.,.04),
                    (.25,.85,1.,.12))
                label('joint_survey_label',((min(joint_x)+max(joint_x))/2,
                    max(joint_y)+1.6,1.3),'联合监测区',.7)
            xs=[item['position'][0] for item in air_targets]
            ys=[item['position'][1] for item in air_targets]
            center=((min(xs)+max(xs))/2,(min(ys)+max(ys))/2,.04)
            add('air_survey_area',M.CUBE,center,(max(xs)-min(xs)+2.,max(ys)-min(ys)+2.,.04),
                (1.,.82,.2,.16))
        if len(deep_targets)>1:
            label('deep_survey_label',(-1.5,4.,-1.),'深水扫测段',.55)
        for item in self.scene.get('observation_targets', []):
            p = item['position']
            add('targets', M.SPHERE, p, (.45,.45,.45) if len(air_targets)>1 and item['domain']=='AIR'
                else (.6,.6,.6), (1.,.9,.25,.8))
            name = ('空中样点' if item['domain']=='AIR' else
                    '浅水点' if item['id']=='offshore_aav_water' else '深水段')
            if item['domain']=='AIR' and len(air_targets)>1:continue
            if item['id'].startswith('deep_swath_') and len(deep_targets)>1:continue
            offset_y = -2.5 if item['domain']=='AIR' else 0.
            label('target_label', (p[0],p[1]+offset_y,p[2]+.5), name, .55)
        for item in self.scene.get('transition_sites', []):
            p = item['position']
            add('transition', M.CYLINDER, p, (1.5,1.5,.08), (.5,1.,.7,.5))
            label('transition_label', (p[0]-4.,p[1],p[2]-.5), '入水与出水区', .55)
        for item in self.scene.get('communication_sites', []):
            p=item['position'];diameter=2*float(item['radius_m'])
            add('shared_support',M.CYLINDER,(p[0],p[1],.03),
                (diameter,diameter,.06),(.1,.9,.5,.2))
            label('shared_support_label',(p[0],p[1],1.),'共享支援区',.55)
        p = self.scene['mother_ship_position']
        assets=self.scene.get('visual_assets')
        if assets:
            mother=add('mother',M.MESH_RESOURCE,p,(1.,1.,1.),(1.,1.,1.,1.))
            mother.mesh_resource='file://'+assets+'/'+self.scene.get('mother_ship_mesh','mother_ship.dae')
            mother.mesh_use_embedded_materials=True
            # Stonefish's original marine asset uses x-forward/y-right/z-down.
            if not self.scene.get('mother_ship_mesh'):
                mother.pose.orientation.x=1.;mother.pose.orientation.w=0.
        else:
            add('mother', M.CUBE, p, (2.,1.,.5), (.8,.9,.95,.7))
        with self.lock:received_count=len(self.received_points)
        mother_text=('母船 · 已接收 '+str(received_count)+' 份' if received_count else '母船 · 等待结果') if self.cooperative else '母船 · 固定接收站'
        label('mother_label', (p[0],p[1],p[2]+3.4), mother_text, .65)
        with self.lock:
            poses = dict(self.poses)
            trails = {k:list(v) for k,v in self.trails.items()}
        for i, key in enumerate(self.members):
            name = ('AAV 1','AAV 2','AAV 3','USV','UUV')[i]
            if key not in poses:
                missing=label('missing', (-34.,-10.+i*2.,2.), name+'：状态缺失')
                missing.id=i
                continue
            pose, received = poses[key]
            p = pose.position
            colour = (*self.colours[i], 1.)
            # AIR meshes remain the original odom_visualization output in RViz.
            # PVS glyph dimensions are illustrative; collision proxies are unchanged.
            def part(kind, scale, offset=(0.,0.,0.)):
                q = pose.orientation
                x,y,z,w = q.x,q.y,q.z,q.w
                ox,oy,oz = offset
                dx=(1-2*y*y-2*z*z)*ox+2*(x*y-z*w)*oy+2*(x*z+y*w)*oz
                dy=2*(x*y+z*w)*ox+(1-2*x*x-2*z*z)*oy+2*(y*z-x*w)*oz
                dz=2*(x*z-y*w)*ox+2*(y*z+x*w)*oy+(1-2*x*x-2*y*y)*oz
                body=add(key+'/body',kind,(p.x+dx,p.y+dy,p.z+dz),scale,colour)
                body.pose.orientation=copy.deepcopy(q)
            if i<3 and assets and self.scene.get('aav_mesh'):
                body=add(key+'/body',M.MESH_RESOURCE,(p.x,p.y,p.z),(1.,1.,1.),(1.,1.,1.,1.))
                body.mesh_resource='file://'+assets+'/'+self.scene['aav_mesh']
                body.mesh_use_embedded_materials=True;body.pose.orientation=copy.deepcopy(pose.orientation)
            elif i == 3:
                part(M.SPHERE,(2.,.35,.4),(0.,-.4,0.))
                part(M.SPHERE,(2.,.35,.4),(0.,.4,0.))
                part(M.CUBE,(1.,.85,.2),(0.,0.,.2))
                part(M.CUBE,(.4,.4,.35),(-.1,0.,.45))
            elif i == 4:
                part(M.SPHERE,(1.6,.3,.3))
                part(M.CUBE,(.35,.7,.045),(-.5,0.,0.))
                part(M.CUBE,(.35,.045,.5),(-.5,0.,0.))
            if len(trails[key])>=2:
                line = add('actual_trails', M.LINE_STRIP, (0.,0.,0.), (.07,0.,0.), colour)
                line.id=i
                line.points = trails[key]
            stale = '（状态过期）' if now-received > 1. else ''
            item=label('platform_labels', (p.x,p.y,p.z+.9), name+stale, .55, colour)
            item.id=i
        # Reuse RViz objects and font geometry; DELETEALL on every frame rebuilds
        # every label/material. Delete only markers no longer in this snapshot.
        keys={(m.ns,m.id) for m in markers}
        removed=[]
        if self.marker_keys is None:
            clear=M();clear.header.frame_id=self.frame;clear.action=M.DELETEALL;removed.append(clear)
        else:
            for ns,ident in self.marker_keys-keys:
                old=M();old.header.frame_id=self.frame;old.ns=ns;old.id=ident;old.action=M.DELETE;removed.append(old)
        self.marker_keys=keys
        # Repeat removal of startup placeholders: a late RViz subscriber may
        # miss a one-shot DELETE, while every ADD snapshot is intentionally full.
        for i,key in enumerate(self.members):
            if key in poses and not any(m.ns=='missing' and m.id==i for m in removed):
                old=M();old.header.frame_id=self.frame;old.ns='missing';old.id=i;old.action=M.DELETE;removed.append(old)
        self.publisher.publish(self.MarkerArray(markers=removed+markers))


def cloud_message(frame_id, stamp, points):
    message = PointCloud2()
    message.header = Header(stamp=stamp, frame_id=frame_id)
    message.height = 1
    message.width = len(points)
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = 12 * len(points)
    message.is_dense = True
    buffer = bytearray()
    for point in points:
        buffer.extend(struct.pack("<fff", *point))
    message.data = bytes(buffer)
    return message


def main():
    rospy.init_node("scene_publisher")
    speed=float(rospy.get_param('/mission/simulation_speed',1.))
    if not math.isfinite(speed) or not 1.<=speed<=16.:raise ValueError('simulation speed must be between 1 and 16')
    if rospy.get_param('/use_sim_time',False):
        from rosgraph_msgs.msg import Clock
        clock_pub=rospy.Publisher('/clock',Clock,queue_size=1,latch=True)
        from nav_msgs.msg import Odometry
        # Accelerate completed fixed model steps, not the arrival of setpoints
        # independently of physics. A slower WATER process must not make AIR
        # references and peer predictions run hundreds of seconds ahead of it.
        model_topics=('/drone_0_qn/odometry','/drone_1_qn/odometry',
                      '/drone_2_qn/odometry','/usv/odometry','/uuv/odometry')
        step_ns=round(float(rospy.get_param('/drone_0_qn/outer_dt_s',.01))*1e9)
        clock_condition=threading.Condition()
        acknowledgments={topic:0 for topic in model_topics}
        def model_step(message,topic):
            # Each of these publishers emits exactly once per completed
            # model step. Do not add a second timestamp gate to this barrier;
            # native freshness/Goal checks remain at their original boundary.
            with clock_condition:
                acknowledgments[topic]+=1
                clock_condition.notify_all()
        clock_subscriptions=[rospy.Subscriber(topic,Odometry,model_step,
            callback_args=topic,queue_size=1) for topic in model_topics]
        def publish_clock():
            epoch_ns=round(time.time()*1e9);bootstrap_started=time.monotonic()
            # ROS/model publishers and the sensor timers first need a live
            # clock to initialize. No task is admitted before those actual
            # streams are ready. Start the step barrier only after bootstrap.
            while not rospy.is_shutdown():
                stamp_ns=epoch_ns+round(speed*(time.monotonic()-bootstrap_started)*1e9)
                clock_pub.publish(Clock(clock=rospy.Time(
                    secs=stamp_ns//1000000000,nsecs=stamp_ns%1000000000)))
                with clock_condition:
                    if all(acknowledgments[topic]>0 for topic in model_topics):
                        seen=dict(acknowledgments)
                        break
                time.sleep(.005)
            stamp_ns+=step_ns
            while not rospy.is_shutdown():
                began=time.monotonic()
                clock_pub.publish(Clock(clock=rospy.Time(
                    secs=stamp_ns//1000000000,nsecs=stamp_ns%1000000000)))
                with clock_condition:
                    while not rospy.is_shutdown() and not all(
                            acknowledgments[topic]>seen[topic] for topic in model_topics):
                        clock_condition.wait(.005)
                    seen=dict(acknowledgments)
                delay=step_ns/1e9/speed-(time.monotonic()-began)
                if delay>0:time.sleep(delay)
                stamp_ns+=step_ns
        threading.Thread(target=publish_clock,daemon=True).start()
        while not rospy.is_shutdown() and rospy.Time.now().to_sec()==0.:time.sleep(.005)
    scene = rospy.get_param("/scene", {})
    geometry=StaticSceneGeometry.from_mapping(scene)
    topic = rospy.get_param("~topic", scene.get("topic", DEFAULT_TOPIC))
    frame_id = rospy.get_param("~frame_id", scene.get("frame_id", "world"))
    obstacle = bool(rospy.get_param("~obstacle", scene.get("obstacle_present", False)))
    center = [float(value) for value in rospy.get_param(
        "~center", scene.get("obstacle_center", [-23.0, 0.0, 0.5]))]
    size = [float(value) for value in rospy.get_param(
        "~size", scene.get("obstacle_size", [1.0, 1.0, 1.2]))]
    resolution = float(rospy.get_param("~resolution", scene.get("resolution", 0.2)))
    rate_hz = float(rospy.get_param("~rate", 10.0))
    if resolution <= 0.0 or rate_hz <= 0.0:
        raise ValueError("resolution and rate must be positive")

    points = box_sample_points(center, size, resolution) if obstacle else []
    if geometry:
        if frame_id!=geometry.frame:raise ValueError('scene frame mismatch')
        obstacle=any(kind=='SOLID' for _,kind,_,_ in geometry.objects)
        points=[p for _,kind,c,s in geometry.objects if kind=='SOLID'
                for p in box_sample_points(c,s,resolution)]
    publisher = rospy.Publisher(topic, PointCloud2, queue_size=1, latch=True)
    request_file=rospy.get_param('/mission/request_file','')
    transport=SceneTransport(scene,frame_id,request_file) if request_file else None
    view = None
    if geometry and rospy.get_param('~visualize', False):
        try:
            view = SceneView(scene, frame_id,transport.request.request_id if transport else None)
        except Exception as error:
            rospy.logerr('Scene view unavailable; sensor map continues: %s', error)
    rospy.loginfo("scene publisher: %d points on %s (obstacle=%s, frame=%s)",
                  len(points), topic, obstacle, frame_id)
    rate = rospy.Rate(rate_hz)
    # Scene is static for this launch: serialize geometry once, refresh only its
    # timestamp. Dense scenes must not repack hundreds of thousands of vertices
    # every frame merely to keep the existing sensor heartbeat fresh.
    cloud = cloud_message(frame_id, rospy.Time.now(), points)
    if transport:rospy.set_param('/mission/transport_request_id',transport.request.request_id)
    while not rospy.is_shutdown():
        if transport:
            next_file=rospy.get_param('/mission/request_file',transport.request_file)
            if next_file and next_file!=transport.request_file:
                try:
                    task_scene={key:rospy.get_param('/scene/'+key) for key in TASK_SCENE_FIELDS
                                if rospy.has_param('/scene/'+key)}
                    request=transport.replace_request(next_file,task_scene)
                    if view:view.replace_request(request,task_scene)
                    # Existing runner waits for this identity before issuing
                    # new Goals; it remains the sole task/commitment authority.
                    rospy.set_param('/mission/transport_request_id',request.request_id)
                    rospy.loginfo('scene task request switched to %s; world/state retained',request.request_id)
                except (ValueError,KeyError,TypeError,OSError) as error:
                    rospy.logerr_throttle(2.,'Scene task request update rejected: %s',str(error))
        cloud.header.stamp=rospy.Time.now()
        publisher.publish(cloud)
        if view:
            try:
                view.publish()
            except Exception as error:
                rospy.logerr_throttle(5., 'Scene view failed; sensor map continues: %s', str(error))
        rate.sleep()


if __name__ == "__main__":
    main()
