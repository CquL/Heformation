#!/usr/bin/env python3
"""Publish the one complete scene map the planners actually consume.

The upstream CPU renderer builds each drone's local cloud from a single global
map.  This node is the only publisher on that scene topic, so the map is always
one complete, consistent description of the world instead of two publishers
overwriting each other.

The legacy profile has one optional box. The five-platform profile declares
multiple SOLID boxes and separate FORBIDDEN/task metadata; only SOLID is sampled
as physical sensor-map geometry. Seabed clearance is checked analytically, not
presented as a sonar measurement. A declared empty solid map is an empty cloud,
not a missing message.

The box centre and size are defined once and used for both the sampled cloud and
the analytic clearance the verifier reports.
"""

from __future__ import annotations

import struct
import copy
import threading
import time
import json
import math
from collections import deque

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from qn_aav_simulator.experiment_verdict import box_sample_points,StaticSceneGeometry

DEFAULT_TOPIC = "/scene/global_cloud"


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
        from qn_aav_simulator.observation_coverage import FiniteDelivery,ObstacleBox
        self.String=String;self.request=load_request(request_file);self.frame=frame
        self.task_service=self.request.template_id=='OFFSHORE_JOINT'
        self.support_sites=tuple(scene.get('communication_sites',()))
        self.mother=tuple(scene.get('mother_ship_receiver_position',scene['mother_ship_position']))
        self.obstacles=tuple(ObstacleBox(tuple(o['center']),tuple(o['size'])) for o in scene['objects'] if o['kind']=='SOLID')
        if any(box.blocks(self.mother,self.mother) for box in self.obstacles):
            raise ValueError('declared mother receiver lies inside a solid; set its exterior attachment position')
        self.lock=threading.Lock();self.states={};self.modes={};self.local_diagnostics={};self.events={}
        self.last_time=math.floor(rospy.Time.now().to_sec()*10.)/10.;self.previous={}
        self.started_at=self.last_time
        self.delivery=None if self.task_service else FiniteDelivery(self.last_time)
        self.delivered=set()
        self.receipts=rospy.Publisher('/mother/received_products',String,queue_size=100)
        self.notifications=rospy.Publisher('/mother/received_notifications',String,queue_size=100)
        self.command_deliveries=rospy.Publisher('/mother/command_deliveries',String,queue_size=100)
        # Passive evaluation view only. The runner never consumes relay truth.
        self.progress=rospy.Publisher('/scene/delivery_progress',String,queue_size=1,latch=True)
        self.last_progress=-1.
        self.subs=[]
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

    def odom(self,key,msg):
        if msg.header.frame_id!=self.frame:return
        p=msg.pose.pose.position
        with self.lock:
            rows=self.states.setdefault(key,deque(maxlen=64))
            stamp=msg.header.stamp.to_sec()
            if not rows or stamp>rows[-1][0]:rows.append((stamp,(p.x,p.y,p.z)))

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

    def produce(self,member,msg):
        from qn_aav_simulator.observation_coverage import DeliveryProduct
        try:
            event=json.loads(msg.data);ident=event['product_id']
            points={p.point_id for r in self.request.regions for p in r.interest_points}
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
                     not self.task_service and event['required_bytes']!=32*1024))):
                raise ValueError('product identity or declared size mismatch')
            generated=event['generated_at'];now=rospy.Time.now().to_sec()
            start_time=self.started_at if self.task_service else self.delivery.start_time
            if not math.isfinite(generated) or not start_time<=generated<=now:
                raise ValueError('product generation outside current run')
            with self.lock:
                if ident in self.events:
                    if self.events[ident]!=event:raise ValueError('conflicting duplicate product')
                    return
                self.events[ident]=event
                # Notification and summary use the same capacity. A received
                # notice is explicitly not the 32 KiB business product.
                if not self.task_service:
                    size=4+len(msg.data.encode('utf-8'))
                    self.delivery.produce('notice:'+ident,DeliveryProduct(member,'mother',size,generated,True))
                    if not terminal and not action_terminal:
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
        if member.startswith('drone_'):
            try:
                from std_srvs.srv import Trigger
                service='/'+member+'_qn_aav/state_digest'
                rospy.wait_for_service(service,timeout=.25)
                reply=rospy.ServiceProxy(service,Trigger)()
                if not reply.success:return
                local=json.loads(reply.message)
                if local.get('agent_id')!=member:return
                digest=local['digest'];model_time=float(local['model_time_s'])
                stamp=float(local['ros_stamp_s'])
                if len(digest)!=64 or any(char not in '0123456789abcdef' for char in digest):return
            except (ValueError,KeyError,TypeError,rospy.ROSException,rospy.ServiceException):
                return
        with self.lock:
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
                active_goal_id=values.get('active_goal_id',''),
                resource_locked=str(values.get('resource_locked',
                    values.get('platform_resource_locked','false'))).lower())
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

    def tick(self,_):
        if self.task_service:
            return self.tick_task_service()
        from qn_aav_simulator.observation_coverage import declared_delivery_channels
        now=math.floor(rospy.Time.now().to_sec()*10.)/10.;out=[];requests=[];progress=None
        with self.lock:
            if now<=self.last_time:return
            states={'mother':(self.mother,'SURFACE')}
            for member,rows in self.states.items():
                stamp,pos=next((r for r in reversed(rows) if r[0]<=now),(-1.,None))
                mode_stamp,mode=next((r for r in reversed(self.modes.get(member,())) if r[0]<=now),(-1.,'UNKNOWN'))
                if 0<=now-stamp<=.25 and 0<=now-mode_stamp<=.25:states[member]=(pos,mode)
            continuous=now-self.last_time<=.25
            # No capacity is credited across a missed observation interval.
            receipts=self.delivery.advance_all(now,declared_delivery_channels(
                self.delivery.products,self.previous,states,self.obstacles,continuous))
            for ident in receipts:
                kind,key=ident.split(':',1)
                if kind=='claim-request':requests.append(self.events[ident])
                elif kind=='command':out.append((self.command_deliveries,dict(self.events[ident],received_at=now)))
                else:out.append((self.notifications if kind=='notice' else self.receipts,
                                 dict(self.events[key],received_at=now)))
            self.last_time=now;self.previous=states
            if now-self.last_progress>=.5:
                progress=dict(at_ros_s=now,scope='INDEPENDENT_TRANSPORT_VIEW',products=[
                    dict(point_id=self.events[key]['point_id'],
                         required_bytes=p.required_bytes,
                         relay_bytes=p.received_prefix.get('usv',0.),
                         mother_bytes=p.received_prefix.get('mother',0.))
                    for ident,p in self.delivery.products.items() if ident.startswith('data:')
                    for key in [ident.split(':',1)[1]]])
                self.last_progress=now
        if progress is not None:
            self.progress.publish(self.String(data=json.dumps(progress,allow_nan=False)))
        for publisher,event in out:
            publisher.publish(self.String(data=json.dumps(event,allow_nan=False)))
        for request in requests:
            threading.Thread(target=self.capture_state_claim,args=(request,),daemon=True).start()

    def tick_task_service(self):
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
            supported=task_service_ready(states,self.support_sites)
            for ident,event in self.events.items():
                if 'product_id' not in event:continue
                if ident in self.delivered or event['generated_at']>now:continue
                notification=event.get('event_type') in ('OBSERVATION_TERMINAL','ACTION_TERMINAL')
                if not notification and not supported:continue
                self.delivered.add(ident)
                out.append((self.notifications if notification else self.receipts,
                    dict(event,received_at=now)))
            self.last_time=now
            progress=dict(at_ros_s=now,scope='TASK_SERVICE',support_active=supported,
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
    def __init__(self, scene, frame):
        from visualization_msgs.msg import Marker, MarkerArray
        from nav_msgs.msg import Odometry
        self.Marker, self.MarkerArray = Marker, MarkerArray
        self.scene, self.frame = scene, frame
        self.lock = threading.Lock()
        self.members = ('drone_0', 'drone_1', 'drone_2', 'usv', 'uuv')
        self.colours = ((1., .75, .15), (.2, .8, 1.), (.7, .4, 1.), (.2, 1., .5), (1., .4, .2))
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
        self.received_points=set()
        if self.cooperative:
            from std_msgs.msg import String
            def received(msg):
                event=json.loads(msg.data)
                with self.lock:self.received_points.add(event['point_id'])
            self.subs.append(rospy.Subscriber('/mother/received_products',String,received,queue_size=10))

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
        add('water', M.CUBE, (-9.,0.,self.scene['surface_z_m']), (62.,32.,.025), (.1,.55,.8,.14))
        add('seabed', M.CUBE, (-9.,0.,self.scene['seabed_z_m']), (62.,32.,.08), (.28,.3,.25,.65))
        label('legend', (-9.,14.,4.), self.scene.get('scenario_label','五平台 · 实际状态与障碍'), .85)
        label('water_label', (18.,12.,.2), '海面', .65)
        label('bed_label', (18.,12.,-5.5), '海底', .65)
        for item in self.scene.get('objects', []):
            p, size = item['center'], item['size']
            colour = ((.65,.65,.7,.95) if item.get('appearance')=='quay' else
                      (.9,.2,.25,.18) if item['kind']=='FORBIDDEN' else (.65,.65,.7,.95))
            appearance=item.get('appearance','box')
            assets=self.scene.get('visual_assets')
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
            label('geometry_label', (p[0],p[1],p[2]+size[2]/2+.6), name, .65)
        for item in self.scene.get('observation_targets', []):
            p = item['position']
            add('targets', M.SPHERE, p, (.6,.6,.6), (1.,.9,.25,.8))
            name = '空中样点' if item['domain']=='AIR' else '水下样点'
            offset_y = -2.5 if item['domain']=='AIR' else 0.
            label('target_label', (p[0],p[1]+offset_y,p[2]+.5), name, .55)
        for item in self.scene.get('transition_sites', []):
            p = item['position']
            add('transition', M.CYLINDER, p, (1.5,1.5,.08), (.5,1.,.7,.5))
            label('transition_label', (p[0]-4.,p[1],p[2]-.5), '入水与出水区', .55)
        p = self.scene['mother_ship_position']
        assets=self.scene.get('visual_assets')
        if assets:
            mother=add('mother',M.MESH_RESOURCE,p,(1.,1.,1.),(1.,1.,1.,1.))
            mother.mesh_resource='file://'+assets+'/mother_ship.dae'
            mother.mesh_use_embedded_materials=True
            # Stonefish's original marine asset uses x-forward/y-right/z-down.
            mother.pose.orientation.x=1.;mother.pose.orientation.w=0.
        else:
            add('mother', M.CUBE, p, (2.,1.,.5), (.8,.9,.95,.7))
        with self.lock:received_count=len(self.received_points)
        mother_text=('母船 · 已接收 '+str(received_count)+' 份' if received_count else '母船 · 等待结果') if self.cooperative else '母船 · 固定接收站'
        label('mother_label', (p[0],p[1],p[2]+1.8), mother_text, .65)
        with self.lock:
            poses = dict(self.poses)
            trails = {k:list(v) for k,v in self.trails.items()}
        for i, key in enumerate(self.members):
            name = ('无人机1','无人机2','无人机3','无人船','潜航器')[i]
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
            if i == 3:
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
            view = SceneView(scene, frame_id)
        except Exception as error:
            rospy.logerr('Scene view unavailable; sensor map continues: %s', error)
    rospy.loginfo("scene publisher: %d points on %s (obstacle=%s, frame=%s)",
                  len(points), topic, obstacle, frame_id)
    rate = rospy.Rate(rate_hz)
    # Scene is static for this launch: serialize geometry once, refresh only its
    # timestamp. Dense scenes must not repack hundreds of thousands of vertices
    # every frame merely to keep the existing sensor heartbeat fresh.
    cloud = cloud_message(frame_id, rospy.Time.now(), points)
    while not rospy.is_shutdown():
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
