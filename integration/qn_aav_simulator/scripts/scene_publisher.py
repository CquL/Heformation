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
from collections import deque

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from qn_aav_simulator.experiment_verdict import box_sample_points,StaticSceneGeometry

DEFAULT_TOPIC = "/scene/global_cloud"


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
        label('legend', (-9.,14.,4.), '五平台联合运动验证', .85)
        label('water_label', (18.,12.,.2), '海面', .65)
        label('bed_label', (18.,12.,-5.5), '海底', .65)
        for item in self.scene.get('objects', []):
            p, size = item['center'], item['size']
            colour = (.9,.2,.25,.18) if item['kind']=='FORBIDDEN' else (.65,.65,.7,.95)
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
            name = '预设空中观测点' if item['domain']=='AIR' else '预设水下样点'
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
        label('mother_label', (p[0],p[1],p[2]+1.8), '母船位置', .65)
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
