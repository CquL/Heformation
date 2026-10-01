#!/usr/bin/env python3
"""Qt operator entry for the existing runner and its authoritative task state.

The runner performs expansion/planning. The joint panel approves its exact
request/revision; it never dispatches motion Goals or infers business completion.
"""
import argparse
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import threading
import time
import uuid

import rospy
from actionlib_msgs.msg import GoalID
from python_qt_binding.QtCore import QTimer,Qt,QPointF,QRectF
from python_qt_binding.QtGui import QFont,QFontDatabase,QPainter,QPainterPath,QPen,QBrush,QColor,QPixmap
from python_qt_binding.QtSvg import QSvgRenderer
from python_qt_binding.QtWidgets import (QApplication,QFileDialog,QHBoxLayout,QLabel,
    QMessageBox,QPlainTextEdit,QPushButton,QSplitter,QVBoxLayout,QWidget,
    QDialog,QFrame,QTableWidget,QTableWidgetItem,
    QHeaderView,QAbstractItemView,QProgressBar,QStackedWidget,QComboBox)
from qn_aav_simulator.task_line import load_request
from qn_aav_simulator.monitoring_request import circle_joint_mission_mappings


class CircleMap(QWidget):
    """One direct world-coordinate circle gesture; no task state duplication."""
    def __init__(self,scene,center,radius,on_change):
        super().__init__();self.scene=scene;self.center=tuple(center);self.radius=float(radius)
        self.dragging=False;self.on_change=on_change;self.setMinimumSize(800,560)
        self.editable=True;self.plan_items=[];self.observed_points=()
        self.draft_circle=None;self.editing_draft=False
        self.online_mapping=bool(scene.get('online_mapping',False))
        self.mapping_layers={};self.mapping_layer='offshore_uuv'
        self.mapping_request_id='';self.received_mapping_ids=set()
        xs=[self.center[0]-self.radius,self.center[0]+self.radius]
        ys=[self.center[1]-self.radius,self.center[1]+self.radius]
        for item in scene.get('objects',()):
            if self.online_mapping and item['kind']!='FORBIDDEN':continue
            xs.extend((item['center'][0]-item['size'][0]/2,item['center'][0]+item['size'][0]/2))
            ys.extend((item['center'][1]-item['size'][1]/2,item['center'][1]+item['size'][1]/2))
        for item in scene.get('return_sites',{}).values():xs.append(item['position'][0]);ys.append(item['position'][1])
        xs.append(scene['mother_ship_position'][0]);ys.append(scene['mother_ship_position'][1])
        for area in scene.get('business_areas',()):
            x,y=area['center'];r=area['radius_m']
            xs.extend((x-r,x+r));ys.extend((y-r,y+r))
        self.bounds=(min(xs)-4,max(xs)+12,min(ys)-5,max(ys)+5)
        self.base_bounds=self.bounds

    def set_mapping_request(self,request):
        """Display cells use the same sampled geometry as the task contract."""
        from qn_aav_simulator.monitoring_request import mapping_cell_samples
        self.mapping_request_id=request.get('request_id','');self.mapping_layers={}
        self.received_mapping_ids=set()
        if request.get('execution_mode')!='ONLINE_MAPPING':return
        for region in request.get('regions',()):
            if region.get('shape')!='CIRCLE':continue
            name=region['region_id']
            if name not in ('offshore_air','offshore_uuv'):continue
            self.mapping_layers[name]=dict(
                label='UUV 深水层' if name=='offshore_uuv' else 'AAV 空中层',
                mode='WATER' if name=='offshore_uuv' else 'AIR',
                center=region['center'],radius=region['radius_m'],
                cells={point['point_id']:mapping_cell_samples(region['center'][:2],
                    region['radius_m'],point['position'],region['coverage_resolution_m'],.25)
                    for point in region['interest_points']})
        self.update()

    def set_received_mapping(self,products,request_id):
        # The runner already validates identity/GoalID and actual receipt.
        # A planned path, local point cloud or terminal report never paints a
        # received report cell; only this authority's received_products does.
        self.received_mapping_ids={event['point_id'] for event in products.values()
            if request_id==self.mapping_request_id and
            event.get('request_id')==request_id and event.get('observed') is True and
            'received_at' in event and event.get('result',{}).get('model')=='MAPPING_PROXY'}

    def set_mapping_layer(self,layer):
        self.mapping_layer=layer;self.update()

    def fit_scene(self):
        xmin,xmax,ymin,ymax=self.base_bounds;cx,cy=self.center;r=self.radius
        self.bounds=(min(xmin,cx-r-2),max(xmax,cx+r+2),min(ymin,cy-r-2),max(ymax,cy+r+2))
        if self.draft_circle:
            (cx,cy),r=self.draft_circle;xmin,xmax,ymin,ymax=self.bounds
            self.bounds=(min(xmin,cx-r-2),max(xmax,cx+r+2),min(ymin,cy-r-2),max(ymax,cy+r+2))
        self.update()

    def fit_region(self):
        center,radius=self.draft_circle if self.editing_draft and self.draft_circle else (self.center,self.radius)
        cx,cy=center;half_y=max(radius+1.,2.)
        half_x=half_y*max(1.,self.width()-40)/max(1.,self.height()-40)
        self.bounds=(cx-half_x,cx+half_x,cy-half_y,cy+half_y);self.update()

    def _scale(self):
        xmin,xmax,ymin,ymax=self.bounds
        return min(max(1.,self.width()-40)/(xmax-xmin),max(1.,self.height()-40)/(ymax-ymin))

    def to_screen(self,point):
        xmin,xmax,ymin,ymax=self.bounds;s=self._scale()
        return QPointF(20+(point[0]-xmin)*s,self.height()-20-(point[1]-ymin)*s)

    def to_world(self,point):
        xmin,xmax,ymin,ymax=self.bounds;s=self._scale()
        return ((point.x()-20)/s+xmin,(self.height()-20-point.y())/s+ymin)

    def mousePressEvent(self,event):
        if self.editable and event.button()==Qt.LeftButton:
            center=self.to_world(event.pos())
            if self.editing_draft:self.draft_circle=(center,0.)
            else:self.center=center;self.radius=0.
            self.dragging=True;self.update()

    def mouseMoveEvent(self,event):
        if not self.editable:
            self.dragging=False
            return
        if self.dragging:
            point=self.to_world(event.pos());center=self.draft_circle[0] if self.editing_draft else self.center
            radius=math.hypot(point[0]-center[0],point[1]-center[1])
            if self.editing_draft:self.draft_circle=(center,radius)
            else:self.radius=radius
            self.on_change(center,radius);self.update()

    def mouseReleaseEvent(self,event):
        if self.dragging and event.button()==Qt.LeftButton:
            self.mouseMoveEvent(event);self.dragging=False

    def paintEvent(self,event):
        painter=QPainter(self);painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(),QColor('#102736'))
        xmin,xmax,ymin,ymax=self.bounds
        painter.setPen(QPen(QColor(80,120,140,90),1))
        for x in range(math.floor(xmin),math.ceil(xmax)+1,2):
            a=self.to_screen((x,ymin));b=self.to_screen((x,ymax));painter.drawLine(a,b)
        for y in range(math.floor(ymin),math.ceil(ymax)+1,2):
            a=self.to_screen((xmin,y));b=self.to_screen((xmax,y));painter.drawLine(a,b)
        for item in self.scene.get('objects',()):
            if self.online_mapping and item['kind']!='FORBIDDEN':continue
            cx,cy,_=item['center'];sx,sy,_=item['size'];a=self.to_screen((cx-sx/2,cy+sy/2))
            painter.setPen(QPen(QColor('#c8c9cf') if item['kind']=='SOLID' else QColor('#ee5963'),2))
            painter.setBrush(QBrush(QColor(160,165,175,170) if item['kind']=='SOLID' else QColor(220,55,70,90)))
            painter.drawRect(QRectF(a.x(),a.y(),sx*self._scale(),sy*self._scale()))
        # Business ranges/declared facility locations are operator metadata,
        # not a hidden free-space map for the planner.
        for area in self.scene.get('business_areas',()):
            color=QColor(*(round(v*255) for v in area['color']))
            c=self.to_screen(area['center']);r=area['radius_m']*self._scale()
            painter.setPen(QPen(color,1.5,Qt.DashLine));fill=QColor(color);fill.setAlpha(17)
            painter.setBrush(fill);painter.drawEllipse(c,r,r)
            painter.setPen(color);painter.drawText(QRectF(c.x()-r,c.y()-r-30,2*r,25),Qt.AlignCenter,area['label'])
            for facility in area.get('facilities',()):
                p=self.to_screen(facility['position']);painter.setPen(QPen(color,2));painter.setBrush(Qt.NoBrush)
                if area['id']=='wind':
                    painter.drawEllipse(p,4,4)
                    for angle in (0,2*math.pi/3,4*math.pi/3):
                        painter.drawLine(p,p+QPointF(13*math.cos(angle),13*math.sin(angle)))
                else:painter.drawRect(QRectF(p.x()-12,p.y()-8,24,16))
                painter.drawText(p+QPointF(10,20),facility['label'])
        for model in self.scene.get('world_models',()):
            if model['model']!='seabed_pipeline':continue
            points=model['points'];painter.setPen(QPen(QColor('#e2b258'),3))
            for a,b in zip(points,points[1:]):painter.drawLine(self.to_screen(a),self.to_screen(b))
            for i,p in enumerate(points[1:]):
                screen=self.to_screen(p);painter.setBrush(QColor('#e2b258'));painter.drawEllipse(screen,4,4)
                painter.drawText(screen+QPointF(6,16),'管段 '+str(i+1))
        for member,item in self.scene.get('return_sites',{}).items():
            p=self.to_screen(item['position']);painter.setBrush(QBrush(QColor('#48d597')))
            painter.setPen(Qt.NoPen);painter.drawEllipse(p,4,4)
        mother=self.to_screen(self.scene['mother_ship_position']);painter.setBrush(QBrush(QColor('#f1d38b')))
        painter.drawEllipse(mother,7,7)
        painter.setPen(QColor('#e5eef4'));painter.drawText(mother+QPointF(12,-10),'母船 / 岸边部署区')
        if self.radius>0:
            c=self.to_screen(self.center);r=self.radius*self._scale()
            painter.setPen(QPen(QColor('#42d7f1'),2));painter.setBrush(QBrush(QColor(66,215,241,32)))
            painter.drawEllipse(c,r,r);painter.setBrush(QBrush(QColor('#42d7f1')));painter.drawEllipse(c,4,4)
            painter.drawText(c+QPointF(12,-r-12),'联合监测区')
        layer=self.mapping_layers.get(self.mapping_layer)
        if self.online_mapping and layer:
            painter.save()
            clip=QPainterPath();centre=self.to_screen(layer['center']);radius=layer['radius']*self._scale()
            clip.addEllipse(centre,radius,radius);painter.setClipPath(clip)
            painter.setPen(Qt.NoPen)
            for ident,cells in layer['cells'].items():
                received=ident in self.received_mapping_ids
                painter.setBrush(QColor(48,202,154,210) if received else QColor(225,237,244,30))
                for x,y,_ in cells:
                    corner=self.to_screen((x-.125,y+.125));side=.25*self._scale()
                    painter.drawRect(QRectF(corner.x(),corner.y(),side,side))
            painter.restore()
            total=sum(len(cells) for cells in layer['cells'].values())
            received=sum(len(cells) for ident,cells in layer['cells'].items()
                         if ident in self.received_mapping_ids)
            painter.fillRect(QRectF(12,12,min(self.width()-24,500),47),QColor(14,35,48,235))
            painter.setPen(QColor('#e2f5f0'))
            painter.drawText(QRectF(23,17,self.width()-46,20),Qt.AlignLeft,
                '{} · 母船已收切片（采样） {:.1f}%'.format(layer['label'],100.*received/total if total else 0.))
            painter.setPen(QColor('#9fb9c7'))
            painter.drawText(QRectF(23,37,self.width()-46,18),Qt.AlignLeft,
                '绿色：已收　浅色：未收 / 未知　0.25 m 采样格；不代表完整三维重建')
        # Measured first hits remain a separate display layer, not receipt or
        # coverage evidence. Draw after the fill so mapped obstacles stay visible.
        if self.online_mapping:
            painter.setPen(QPen(QColor('#74daf0'),2));painter.setBrush(Qt.NoBrush)
            for point in self.observed_points:painter.drawPoint(self.to_screen(point))
        for item in self.plan_items:
            member=item.get('coalition',[''])[0];color=QColor(COLORS.get(member,'#38b6d0'))
            painter.setPen(QPen(color,1.4,Qt.DashLine));painter.setBrush(Qt.NoBrush)
            for step in item.get('execution_steps',()):
                path=step.get('native_prediction',{}).get('reference_path',())
                for a,b in zip(path,path[1:]):painter.drawLine(self.to_screen(a),self.to_screen(b))
            observations=[step for step in item.get('execution_steps',()) if step.get('observation_ids')]
            for step in observations:
                point=step.get('native_prediction',{}).get('terminal_position')
                if point:painter.drawEllipse(self.to_screen(point),3,3)
        if self.draft_circle:
            center,radius=self.draft_circle;c=self.to_screen(center);r=radius*self._scale()
            painter.setPen(QPen(QColor('#ffca5a'),2,Qt.DashLine));painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(c,r,r);painter.drawText(c+QPointF(12,r+20),'新任务草稿 · 尚未采用')
        painter.setPen(QColor('#b5c9d7'))
        painter.drawText(QRectF(14,self.height()-30,self.width()-28,22),Qt.AlignLeft,
            '青色点：实测命中 · 虚线仅为目标意图 · 填色仅来自母船实际收件' if self.online_mapping else
            '二维区域与预计路线 · 实际运动请查看独立 RViz' if self.plan_items else '按住并拖动鼠标绘制监测圆 · 已知场景几何')


class CircleMissionSelector(QWidget):
    def __init__(self,request_path,scene_path,output_request,output_scene,smoke=False):
        super().__init__();import yaml
        self.yaml=yaml;self.base_request=yaml.safe_load(request_path.read_text())
        self.base_scene=yaml.safe_load(scene_path.read_text());self.output_request=output_request
        self.output_scene=output_scene;self.accepted=False
        air=next(entry for entry in self.base_request['regions'] if entry['region_id']=='offshore_air')
        center=air.get('center') or [(air['corner_a'][0]+air['corner_b'][0])/2,
                                     (air['corner_a'][1]+air['corner_b'][1])/2]
        radius=air.get('radius_m') or min(air['corner_b'][0]-air['corner_a'][0],
                                          air['corner_b'][1]-air['corner_a'][1])/2
        scene=self.base_scene['scene'];self.setWindowTitle('Heformation — 选择联合监测区域')
        self.resize(1040,720);layout=QVBoxLayout(self)
        layout.addWidget(QLabel('在海域图上按下鼠标确定圆心，拖动确定半径。黄色圆为本次联合监测区。'))
        self.status=QLabel();layout.addWidget(self.status)
        self.map=CircleMap(scene,center[:2],radius,self.changed);layout.addWidget(self.map,1)
        row=QHBoxLayout();confirm=QPushButton('确认区域并生成任务');cancel=QPushButton('取消')
        row.addWidget(confirm);row.addWidget(cancel);layout.addLayout(row)
        confirm.clicked.connect(self.confirm);cancel.clicked.connect(self.close);self.changed(self.map.center,self.map.radius)
        if smoke:QTimer.singleShot(100,self.confirm)

    def changed(self,center,radius):
        self.status.setStyleSheet('')
        self.status.setText('圆心 ({:.2f}, {:.2f}) m　半径 {:.2f} m（允许 2–10 m）'.format(
            center[0],center[1],radius))

    def confirm(self):
        try:
            request,scene=circle_joint_mission_mappings(
                self.base_request,self.base_scene,self.map.center,self.map.radius)
            self.output_request.write_text(self.yaml.safe_dump(request,allow_unicode=True,sort_keys=False))
            self.output_scene.write_text(self.yaml.safe_dump(scene,allow_unicode=True,sort_keys=False))
            load_request(self.output_request);self.accepted=True;self.close()
        except Exception as error:
            reason=str(error)
            translations=(
                ('no free AIR coverage witness','圈内没有可供空中平台执行的自由观测位置'),
                ('no free deep-water coverage witness','圈内没有可供UUV执行的自由深水观测位置'),
                ('no qualified entry/exit point','圈区周围没有满足障碍净距的入水/出水位置'),
                ('no free USV contact point','圈区附近没有满足障碍净距的USV支援位置'),
                ('radius must be within','半径必须在2至10模型米之间'))
            message=next((cn for token,cn in translations if token in reason),reason)
            self.status.setStyleSheet('color: #ff6666; font-weight: bold;')
            self.status.setText('区域不可用：'+message)
            QMessageBox.warning(self,'监测区域不可用',message+'\n请重新圈选。')


MEMBERS=('drone_0','drone_1','drone_2','usv','uuv')
MEMBER_NAMES=dict(zip(MEMBERS,('AAV 1','AAV 2','AAV 3','USV','UUV')))
COLORS=dict(zip(MEMBERS,('#00add7','#7c32eb','#7a879c','#00a368','#edab00')))


def terminal_kind(state):
    status=state.get('status','')
    if status.startswith('PASS'):return 'success'
    if status=='NOT_CONFIRMED':return 'unconfirmed'
    stopped=status in ('STOPPED','CANCELLED','CANCELED')
    if status.startswith('FAIL') or status=='UNKNOWN_LOCKED' or stopped:
        first=state.get('first_failure') or {}
        reason=first.get('reason') or state.get('failure_reason') or ''
        if state.get('runtime_safety_failure') or any(token in reason for token in
                ('SAFETY_VIOLATION','SCENE_CLEARANCE','DOMAIN_VIOLATION','actual fleet safety:',
                 'actual fleet clearance')):return 'failure'
        # The runner records only a validated stop request. Its later native
        # Result may say CANCEL_REQUEST or timeout while bookings stay locked.
        if state.get('operator_stop_requested') or stopped:return 'stopped'
        return 'failure'
    return ''


def failure_summary(state,items,scene):
    """Use the runner's first cause; old snapshots may only retain safety text."""
    first=state.get('first_failure') or {}
    member=first.get('member','');ident=first.get('execution_id','')
    reason=first.get('reason') or state.get('runtime_safety_failure') or ''
    if not reason:
        failed=next((row for row in state.get('executions',())
            if row.get('result') in ('FAILED','FAIL','ABORTED','REJECTED_BEFORE_ACCEPTANCE','UNKNOWN_LOCKED')),None)
        failed=failed or next((row for row in state.get('step_results',()) if row.get('verified') is False),{})
        ident=ident or failed.get('activity_id') or failed.get('execution_id','')
        action=state.get('current_actions',{}).get(ident,{})
        reason=failed.get('reason') or failed.get('failure_reason') or action.get('failure_reason') or state.get('failure_reason') or ''
    if not reason:return ''
    # This names the member explicitly identified by the existing safety record.
    match=re.search(r'actual fleet safety: (\S+) SCENE_CLEARANCE: ([^=]+)=([-+\d.eE]+)m',reason)
    if match:
        member,obstacle,clearance=match.groups()
        obstacle={'rock':'岩石','quay':'码头','seabed':'海底'}.get(obstacle,obstacle)
        threshold=scene.get('required_clearance_m')
        reason='与{}净距不足：{} m'.format(obstacle,clearance)
        if threshold is not None:reason+='（要求 ≥ {:.2f} m）'.format(threshold)
    if not member and ident:
        item=next((i for i in items if ident==i['execution_id'] or ident.startswith(i['execution_id']+':step:')),None)
        if item:member='、'.join(MEMBER_NAMES.get(m,m) for m in item.get('coalition',()))
    name=MEMBER_NAMES.get(member,member)
    return (name+(' ' if reason.startswith('与') else '：') if name else '')+reason


def icon_label(kind,color='#132d57',size=26):
    """Small vector icons shared by the member rows and section headings."""
    shapes={
        'wave':'<path d="M2 10 Q8 3 15 9 T29 6 M2 21 Q8 14 15 20 T29 17"/>',
        'aav':'<path d="M8 9 L23 22 M23 9 L8 22 M10 15 H21"/><ellipse cx="7" cy="7" rx="5" ry="3"/><ellipse cx="24" cy="7" rx="5" ry="3"/><ellipse cx="7" cy="24" rx="5" ry="3"/><ellipse cx="24" cy="24" rx="5" ry="3"/><circle cx="15.5" cy="15.5" r="3"/>',
        'usv':'<path d="M2 19 H29 L24 26 H8 Z"/><path d="M8 18 V13 H21 V18 M15 4 V12 M10 8 H19"/>',
        'uuv':'<path d="M6 16 H22 A5 5 0 0 1 22 26 H6 A5 5 0 0 1 6 16 Z M15 8 V16 M25 20 L29 16 V27 L25 24"/>',
        'clipboard':'<rect x="7" y="6" width="19" height="24" rx="2"/><rect x="12" y="2" width="9" height="7" rx="2"/><path d="M12 15 H21 M12 20 H21 M12 25 H19"/>',
        'link':'<circle cx="8" cy="7" r="4"/><circle cx="24" cy="16" r="4"/><circle cx="8" cy="26" r="4"/><path d="M11 9 L20 14 M11 24 L20 18 M8 11 V22"/>',
        'radio':'<path d="M16 14 L10 29 H22 Z M10 10 Q3 17 9 23 M22 10 Q29 17 23 23 M7 5 Q-2 16 5 27 M25 5 Q34 16 27 27"/><circle cx="16" cy="12" r="2"/>',
        'clock':'<circle cx="16" cy="16" r="13"/><path d="M16 7 V17 L23 21"/>',
        'lock':'<rect x="7" y="14" width="19" height="15" rx="2"/><path d="M11 14 V9 A6 6 0 0 1 12 0 V14 M16 20 V24"/>',
        'circle':'<circle cx="16" cy="16" r="12"/>',
        'stop':'<rect x="4" y="4" width="24" height="24" rx="4"/><rect x="11" y="11" width="10" height="10" fill="'+color+'"/>',
    }
    svg=('<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 32 32"><g fill="none" stroke="'+color+
        '" stroke-width="'+('4' if kind=='wave' else '2.2')+'" stroke-linecap="round" stroke-linejoin="round">'+shapes[kind]+'</g></svg>')
    pixmap=QPixmap(size*2,size*2);pixmap.fill(Qt.transparent)
    painter=QPainter(pixmap);QSvgRenderer(svg.encode()).render(painter);painter.end();pixmap.setDevicePixelRatio(2)
    label=QLabel();label.setPixmap(pixmap);label.setFixedSize(size,size);return label


def activity_role(item):
    if item.get('executor_id')=='aav_formation':return 'Swarm 编队返航'
    if '::formation-assembly:' in item.get('task_id',''):return '编队集结'
    if not item.get('fulfills_task',True):return '共享通信支援'
    if 'uuv' in item.get('coalition',()):return '深水扫测'
    if any(s.get('operation')=='ENTER_WATER' for step in item.get('execution_steps',())
           for s in (step.get('native_action') or {}).get('segments',())):return '跨介质点测'
    return '空中扫测'


def step_label(step):
    ref=step.get('target_ref','')
    if ref=='return:swarm-formation-handoff':return '共同编队返航'
    if ref=='return:formation-assemble':return '编队集结'
    if ref=='return:formation-transit':return '编队返航'
    if ref=='return:formation-home':return '编队返回部署区'
    if ref.startswith(('return:formation-climb:','return:formation-slot:')):return '编队集结'
    if ref.startswith(('return:','return-climb:','return-overhead:','return-via:')):return '返航'
    if ref.startswith('departure-wait:'):return '等待出发'
    if ref.startswith(('outbound-','air-route:','survey-entry:','transition:','transition-stage:')):return '转场'
    segments=(step.get('native_action') or {}).get('segments',())
    if any(s.get('operation')=='ENTER_WATER' for s in segments):return '入水 · 点测 · 出水'
    if any(s.get('operation')=='WATER_PATH' for s in segments):return '水下扫测'
    if any(s.get('operation')=='SURFACE_PATH' for s in segments):return '支援'
    return '扫测'


class PlanTimeline(QWidget):
    """Draw only the selected Plan; filled steps require matching actual results."""
    def __init__(self):
        super().__init__();self.state={};self.setMinimumHeight(166)

    def paintEvent(self,event):
        painter=QPainter(self);painter.setRenderHint(QPainter.Antialiasing)
        items=sorted((self.state.get('selected_plan') or self.state.get('plan') or {}).get('items',[]),
            key=lambda item:MEMBERS.index(item['coalition'][0]) if item.get('coalition') and item['coalition'][0] in MEMBERS else 9)
        if not items:
            painter.setPen(QColor('#708095'));painter.drawText(self.rect(),Qt.AlignCenter,'生成方案后，在这里查看并行活动与返航安排')
            return
        horizon=max((i.get('planned_finish',0) for i in items),default=1) or 1
        left=68.;width=max(1.,self.width()-left-10);row_height=24
        rows=[(MEMBERS.index(member),member,item) for item in items
              for member in item.get('coalition',()) if member in MEMBERS]
        group_return=any(item.get('executor_id')=='aav_formation' for item in items)
        bottom=30+(len(MEMBERS)-1)*row_height+21
        verified={row['execution_id'] for row in self.state.get('step_results',()) if row.get('verified')}
        current={} if terminal_kind(self.state) else self.state.get('current_actions',{})
        painter.setFont(QFont(self.font().family(),9))
        for tick in range(5):
            x=left+width*tick/4
            painter.setPen(QColor('#dce6ed'));painter.drawLine(QPointF(x,20),QPointF(x,bottom))
            painter.setPen(QColor('#738197'));painter.drawText(QRectF(min(x-28,self.width()-60),0,60,20),Qt.AlignCenter,
                '{:02d}:{:02d}'.format(int(horizon*tick/4)//60,int(horizon*tick/4)%60))
        return_gate=None
        for row,member,item in rows:
            color=QColor(COLORS.get(member,'#159aaf'))
            y=30+row*row_height;painter.setPen(color)
            painter.drawText(QRectF(0,y,65,22),Qt.AlignVCenter,MEMBER_NAMES.get(member,member))
            clock=item.get('planned_start',0.);groups=[]
            steps=item.get('execution_steps',());action=current.get(item['execution_id'],{})
            for index,step in enumerate(steps):
                ident=item['execution_id']+':step:'+str(index) if len(steps)>1 else item['execution_id']
                done=ident in verified
                active=action.get('step')==index and action.get('phase') not in ('WAITING_FOR_GROUP_RETURN','WAITING_FOR_RECEIPTS')
                label=step_label(step);duration=step.get('duration_s',0.)
                prediction=step.get('native_prediction',{})
                if prediction.get('joint_return_boundary'):return_gate=clock
                if group_return and step.get('target_ref')=='return:swarm-formation-handoff':
                    clock+=duration;continue
                wait=prediction.get('joint_wait_s',0.)
                if wait:
                    groups.append((clock,duration-wait,label,done,active))
                    groups.append((clock+duration-wait,wait,'目标区等待',bool(self.state.get('joint_return_release_at_ros_s')),
                        action.get('phase')=='WAITING_FOR_GROUP_RETURN'))
                    clock+=duration;continue
                if member=='usv' and prediction.get('arrival_duration_s') is not None:
                    arrival=prediction['arrival_duration_s']
                    groups.append((clock,arrival,'转场',done,active and not done))
                    groups.append((clock+arrival,duration-arrival,'共享支援',bool(self.state.get('joint_return_release_at_ros_s')),
                        action.get('phase') in ('WAITING_FOR_RECEIPTS','WAITING_FOR_GROUP_RETURN')))
                    clock+=duration;continue
                if groups and groups[-1][2:]==(label,done,active):
                    groups[-1]=(groups[-1][0],groups[-1][1]+duration,label,done,active)
                else:groups.append((clock,duration,label,done,active))
                clock+=duration
            for start,duration,label,done,active in groups:
                rect=QRectF(left+width*start/horizon,y,max(2.,width*duration/horizon-1),21)
                fill=QColor('#cbd3df') if label=='返航' and not done else QColor(color)
                fill.setAlpha(205 if done else 170 if label=='返航' else 55)
                painter.setBrush(fill);painter.setPen(QPen(color,2 if active else 0))
                painter.drawRoundedRect(rect,3,3)
                if rect.width()>48:
                    painter.setPen(QColor('white') if done else QColor('#263d53'))
                    painter.drawText(rect,Qt.AlignCenter,label)
        if return_gate is not None:
            x=left+width*return_gate/horizon;painter.setPen(QPen(QColor('#8998ac'),1.3,Qt.DashLine))
            painter.drawLine(QPointF(x,24),QPointF(x,bottom))
            painter.setPen(QColor('#456083'))
            painter.drawText(QRectF(x+5,3,min(190,self.width()-x-5),18),Qt.AlignLeft,
                '确认共同返航后' if self.state.get('session_phase') else '共同返航条件满足后')
        epoch=(self.state.get('confirmation') or {}).get('at_ros_s')
        if epoch and self.state.get('status','').startswith('RUNNING'):
            elapsed=max(0,self.state.get('updated_at_ros_s',epoch)-epoch)
            x=left+min(1.,elapsed/horizon)*width;painter.setPen(QPen(QColor('#086bff'),2))
            painter.drawLine(QPointF(x,21),QPointF(x,bottom))
            painter.drawText(QRectF(max(left,min(x-25,self.width()-80)),0,78,18),Qt.AlignCenter,
                '当前' if elapsed<=horizon else '超出预计时长')


class JointMissionPanel(QWidget):
    """Independent task window; RViz is a separate process, runner is authority."""
    def __init__(self,request,scene,output,select_region=True):
        super().__init__()
        import yaml
        self.yaml=yaml;self.output=output;output.mkdir(parents=True,exist_ok=True)
        self.base_request=yaml.safe_load(request.read_text());self.base_scene=yaml.safe_load(scene.read_text())
        self.select_region=select_region;self.state={};self.confirmation_sent=False
        self.draft_circle=None;self.pending_command_id=None;self.confirmed_preview_revision=None
        self.preview_revision_shown=None;self.active_request_id=None
        self.submitted=(output/'ui-scene.yaml').exists()
        if self.submitted:
            self.base_request=yaml.safe_load((output/'ui-request.yaml').read_text())
            self.base_scene=yaml.safe_load((output/'ui-scene.yaml').read_text())
        air=next(r for r in self.base_request['regions'] if r['region_id']=='offshore_air')
        self.center=tuple((air.get('center') or [(a+b)/2 for a,b in zip(air['corner_a'],air['corner_b'])])[:2])
        self.radius=air.get('radius_m') or min(air['corner_b'][i]-air['corner_a'][i] for i in (0,1))/2
        self.point_count=sum(len(r.get('interest_points',())) for r in self.base_request['regions'])
        self.last_snapshot=None;self.last_update=time.monotonic()
        self.setObjectName('taskPanel');self.setWindowTitle('Heformation · 空—海—潜协同任务')
        self.resize(1588,990)
        self.telemetry={};self.transport={};self.survey_hits={};self.telemetry_lock=threading.Lock();self.subscribers=[]
        self.subscriptions_started=False;self.stop_sent=False
        self.setStyleSheet('''
            QWidget { color: #102954; font-size: 13px; }
            QWidget#taskPanel { background: #eaf0f7; }
            QFrame#panel, QFrame#topbar { background: #ffffff; border: 1px solid #e1e9f1; border-radius: 6px; }
            QFrame#topbar { border-radius: 0; }
            QFrame#subcard { background: #fbfdff; border: 1px solid #e4edf6; border-radius: 6px; }
            QLabel#brand { font-size: 29px; font-weight: 750; }
            QLabel#subtitle { font-size: 20px; font-weight: 600; }
            QLabel#section { font-weight: 700; font-size: 17px; }
            QLabel#muted { color: #8390a4; font-size: 11px; }
            QLabel#status { color: #0967ff; font-weight: 750; font-size: 23px; }
            QPushButton { background: #fbfdff; border: 1px solid #d8e2f0; border-radius: 4px; padding: 6px 10px; }
            QPushButton:hover { background: #eff7ff; border-color: #91bafa; }
            QPushButton:disabled { color: #a2aec0; background: #f2f5fa; border-color: #e7edf5; }
            QPushButton#primary, QPushButton#tab:checked { background: #0967ff; color: white; border: 1px solid #0967ff; font-weight: 700; }
            QPushButton#primary:disabled { background: #e6eef9; color: #93a9c6; border-color: #e0e9f4; }
            QPushButton#stop { background: white; color: #ff324b; border: 1px solid #ff324b; padding: 8px 12px; }
            QPushButton#stop:disabled { color: #c6a9af; border-color: #e7d9dc; }
            QProgressBar { border: none; background: #e8eff7; border-radius: 4px; max-height: 9px; }
            QProgressBar::chunk { background: #069f6a; border-radius: 4px; }
            QProgressBar#observation::chunk { background: #146bfb; }
            QTableWidget { border: none; gridline-color: #eaf0f7; background: white; }
            QHeaderView::section { background: #f4f8fc; color: #6d819d; border: none; padding: 10px; }
            QPlainTextEdit { border: 1px solid #e1e9f2; background: #f8fbff; }
        ''')
        outer=QVBoxLayout(self);outer.setContentsMargins(0,0,0,0);outer.setSpacing(0)
        bar=QFrame();bar.setObjectName('topbar');bar.setFixedHeight(51);header=QHBoxLayout(bar);header.setContentsMargins(21,3,21,3)
        header.addWidget(icon_label('wave','#1ab7bd',42))
        brand=QLabel('Heformation');brand.setObjectName('brand');header.addWidget(brand)
        subtitle=QLabel('空—海—潜协同任务');subtitle.setObjectName('subtitle');header.addSpacing(13);header.addWidget(subtitle)
        header.addStretch();badge=QLabel('任务控制台 · RViz 独立窗口')
        badge.setStyleSheet('border: 1px solid #d7e1f0; border-radius: 5px; padding: 5px 9px; color: #6b7c98;')
        header.addWidget(badge);header.addSpacing(8)
        self.connection=QLabel('● 等待仿真');self.connection.setStyleSheet('color: #7a8ba5; background: #f3f6fb; border-radius: 13px; padding: 5px 12px;')
        header.addWidget(self.connection);header.addSpacing(12);self.elapsed=QLabel('尚未开始');header.addWidget(self.elapsed)
        outer.addWidget(bar)
        flow=QFrame();flow.setObjectName('topbar');flow.setFixedHeight(44);flow_row=QHBoxLayout(flow);flow_row.setContentsMargins(70,8,70,8)
        self.workflow=[]
        for i,name in enumerate(('选择区域','生成协同方案','确认执行','协同作业','共同返航')):
            label=QLabel();label.setMinimumWidth(165);flow_row.addWidget(label);self.workflow.append((label,name))
            if i<4:
                line=QFrame();line.setFixedHeight(3);line.setMinimumWidth(25);line.setMaximumWidth(90)
                line.setStyleSheet('background: #cbd7e6; border-radius: 1px;');flow_row.addWidget(line,1)
        outer.addWidget(flow)
        body=QHBoxLayout();body.setContentsMargins(5,8,5,8);body.setSpacing(7)
        left,left_layout=self.card('本次监测任务');left.setFixedWidth(300)
        title=QHBoxLayout();title.addWidget(icon_label('clipboard',size=24));title.addWidget(QLabel('远端海域联合监测'));title.addStretch();left_layout.addLayout(title)
        region_box=QFrame();region_box.setObjectName('subcard');rb=QVBoxLayout(region_box);rb.setContentsMargins(11,12,11,10)
        region_line=QHBoxLayout();circle=icon_label('circle','#086cff',46);circle.setStyleSheet('background: #d0e8ff; border-radius: 23px;')
        region_line.addWidget(circle);self.region=QLabel();self.region.setWordWrap(True);region_line.addWidget(self.region,1);rb.addLayout(region_line)
        region_buttons=QHBoxLayout();self.view_region_button=QPushButton('查看区域');self.view_region_button.clicked.connect(
            lambda:(self.show_page(0),self.map.fit_region()))
        self.edit_button=QPushButton('编辑监测区域');self.edit_button.clicked.connect(self.edit_region)
        region_buttons.addWidget(self.view_region_button);region_buttons.addWidget(self.edit_button);rb.addLayout(region_buttons);left_layout.addWidget(region_box)
        label=QLabel('当前分工');label.setObjectName('section');left_layout.addWidget(label)
        self.member_rows={}
        for member in MEMBERS:
            row=QHBoxLayout();row.setSpacing(7);color=COLORS[member]
            row.addWidget(icon_label('aav' if member.startswith('drone') else member,color,29))
            name=QLabel(MEMBER_NAMES[member]);name.setMinimumWidth(49);name.setStyleSheet('color: '+color+'; font-weight: 700; font-size: 15px;')
            row.addWidget(name);role=QLabel('待分配');role.setWordWrap(True);row.addWidget(role,1)
            mode=QLabel('—');mode.setAlignment(Qt.AlignCenter);mode.setMinimumWidth(52)
            mode.setStyleSheet('background: '+self.tint(color)+'; color: '+color+'; border-radius: 5px; padding: 5px 6px;')
            row.addWidget(mode);left_layout.addLayout(row);self.member_rows[member]=(role,mode)
        divider=QFrame();divider.setFrameShape(QFrame.HLine);divider.setStyleSheet('color: #e5edf5;');left_layout.addWidget(divider)
        label=QLabel('协同关系');label.setObjectName('section');left_layout.addWidget(label)
        cr=QHBoxLayout();cr.addWidget(icon_label('link','#617086',28));self.cooperation=QLabel();self.cooperation.setWordWrap(True)
        cr.addWidget(self.cooperation,1);left_layout.addLayout(cr)
        left_layout.addStretch()
        explain,explain_layout=self.card('方案说明',icon='clipboard');explain.setObjectName('subcard')
        explain_layout.addWidget(QLabel('按当前状态分配角色。\n路径由执行端规划。'))
        self.plan_info=QLabel('生成后显示方案版本与决策用时');self.plan_info.setWordWrap(True);self.plan_info.setObjectName('muted')
        explain_layout.addWidget(self.plan_info);left_layout.addWidget(explain)
        muted=QLabel('预计时刻仅供调度参考。');muted.setObjectName('muted');left_layout.addWidget(muted)
        body.addWidget(left)
        middle=QFrame();middle.setObjectName('panel');middle_layout=QVBoxLayout(middle);middle_layout.setContentsMargins(0,0,0,0);middle_layout.setSpacing(0)
        toolbar=QHBoxLayout();toolbar.setContentsMargins(11,8,11,8);toolbar.addWidget(icon_label('link',size=24))
        title=QLabel('区域与协同方案');title.setObjectName('section');toolbar.addWidget(title);toolbar.addStretch()
        self.area_tab=QPushButton('二维区域');self.area_tab.setObjectName('tab');self.area_tab.setCheckable(True)
        self.plan_tab=QPushButton('任务方案');self.plan_tab.setObjectName('tab');self.plan_tab.setCheckable(True)
        self.preview_tab=QPushButton('新方案预览');self.preview_tab.setObjectName('tab');self.preview_tab.setCheckable(True);self.preview_tab.setVisible(False)
        self.area_tab.clicked.connect(lambda:self.show_page(0));self.plan_tab.clicked.connect(lambda:self.show_page(1))
        self.preview_tab.clicked.connect(lambda:self.show_page(2))
        toolbar.addWidget(self.area_tab);toolbar.addWidget(self.plan_tab);toolbar.addWidget(self.preview_tab);toolbar.addStretch()
        self.fit_button=QPushButton('适应场景');self.fit_button.clicked.connect(lambda:self.map.fit_scene());toolbar.addWidget(self.fit_button)
        self.detail_button=QPushButton('任务记录');self.detail_button.clicked.connect(self.show_details);toolbar.addWidget(self.detail_button)
        middle_layout.addLayout(toolbar)
        self.pages=QStackedWidget();self.map=CircleMap(self.base_scene['scene'],self.center,self.radius,self.region_changed)
        self.map.set_mapping_request(self.base_request)
        layers=QHBoxLayout();layers.setContentsMargins(12,4,12,6)
        layers.addWidget(QLabel('已收建图层：'))
        self.deep_layer=QPushButton('UUV 深水');self.air_layer=QPushButton('AAV 空中')
        for button in (self.deep_layer,self.air_layer):button.setCheckable(True);button.setObjectName('tab');layers.addWidget(button)
        self.deep_layer.setChecked(True)
        self.deep_layer.clicked.connect(lambda:self.select_mapping_layer('offshore_uuv'))
        self.air_layer.clicked.connect(lambda:self.select_mapping_layer('offshore_air'))
        layers.addStretch();self.mapping_layer_bar=QWidget();self.mapping_layer_bar.setLayout(layers)
        self.mapping_layer_bar.setVisible(self.map.online_mapping);middle_layout.addWidget(self.mapping_layer_bar)
        self.map.setMinimumSize(460,370);self.pages.addWidget(self.map)
        self.table=QTableWidget(5,5);self.table.setHorizontalHeaderLabels(('平台','角色','当前动作','结果已收','资源'))
        self.table.verticalHeader().hide();self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows);self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.verticalHeader().setDefaultSectionSize(55);self.pages.addWidget(self.table);middle_layout.addWidget(self.pages,1)
        preview_page=QWidget();preview_layout=QVBoxLayout(preview_page)
        self.preview_summary=QLabel();self.preview_summary.setWordWrap(True);preview_layout.addWidget(self.preview_summary)
        self.preview_table=QTableWidget(0,4);self.preview_table.setHorizontalHeaderLabels(('平台','任务角色','动作链','预计时段'))
        self.preview_table.verticalHeader().hide();self.preview_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.preview_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.preview_table.verticalHeader().setDefaultSectionSize(64);preview_layout.addWidget(self.preview_table,1)
        note=QLabel('切换前按实际状态重算分工与时序，当前动作安全结束前不派发。预计时段不含人工驻留，返航需另行确认。')
        note.setWordWrap(True);note.setObjectName('muted');preview_layout.addWidget(note);self.pages.addWidget(preview_page)
        draft_row=QHBoxLayout();draft_row.setContentsMargins(12,5,12,0)
        self.draft_hint=QLabel('');self.draft_hint.setWordWrap(True);draft_row.addWidget(self.draft_hint,1)
        self.policy_box=QComboBox();self.policy_box.addItem('排队执行 · 当前任务完成后','queue')
        self.policy_box.addItem('立即替换 · 中断当前任务','replace');draft_row.addWidget(self.policy_box)
        self.current_button=QPushButton('查看当前任务');self.current_button.clicked.connect(self.view_current);draft_row.addWidget(self.current_button)
        self.draft_controls=QWidget();self.draft_controls.setLayout(draft_row);self.draft_controls.setVisible(False);middle_layout.addWidget(self.draft_controls)
        commands=QHBoxLayout();commands.setContentsMargins(12,9,12,9)
        self.central_hint=QLabel('圈选区域 → 生成方案 → 确认执行');self.central_hint.setWordWrap(True);self.central_hint.setObjectName('muted')
        commands.addWidget(self.central_hint,1)
        self.generate_button=QPushButton('生成协同方案');self.generate_button.clicked.connect(self.generate)
        self.confirm_button=QPushButton('确认并执行');self.confirm_button.setObjectName('primary');self.confirm_button.clicked.connect(self.confirm)
        self.confirm_button.setEnabled(False);commands.addWidget(self.generate_button);commands.addWidget(self.confirm_button)
        middle_layout.addLayout(commands);body.addWidget(middle,1)
        right,right_layout=self.card('执行状态');right.setFixedWidth(300);right_layout.setSpacing(8)
        self.status=QLabel('● 等待选择区域');self.status.setObjectName('status');self.status.setWordWrap(True);right_layout.addWidget(self.status)
        self.status_detail=QLabel('尚未生成协同方案');self.status_detail.setWordWrap(True);right_layout.addWidget(self.status_detail)
        self.observed=QLabel('— / —');self.received=QLabel('— / —')
        self.observation_progress=QProgressBar();self.observation_progress.setObjectName('observation')
        self.progress=QProgressBar()
        mapping=bool(self.base_scene['scene'].get('online_mapping',False))
        for name,bar,count in (('切片建图' if mapping else '有效观测',self.observation_progress,self.observed),('母船已收',self.progress,self.received)):
            row=QHBoxLayout();row.addWidget(QLabel(name));bar.setTextVisible(False);row.addWidget(bar,1);row.addWidget(count);right_layout.addLayout(row)
        self.observed.setToolTip(('切片占据建图（几何采样），不代表全部三维表面已重建。\n' if mapping else '')+
            '由已收到的观测终结报告确认，不根据预计航迹或显示点云推断。')
        support=QFrame();support.setObjectName('subcard');sl=QVBoxLayout(support);sl.setContentsMargins(10,10,10,10)
        sr=QHBoxLayout();sr.addWidget(icon_label('radio','#059c6a',29));heading=QLabel('共享支援');heading.setObjectName('section');sr.addWidget(heading);sr.addStretch()
        self.support_status=QLabel('待命');sr.addWidget(self.support_status);sl.addLayout(sr)
        self.support_members=QLabel('等待方案选择服务对象');self.support_members.setWordWrap(True);sl.addWidget(self.support_members)
        small=QLabel('按实际会合与接收更新。');small.setObjectName('muted');sl.addWidget(small);right_layout.addWidget(support)
        self.action_rows={}
        for member in MEMBERS:
            frame=QFrame();frame.setObjectName('activityRow');row=QHBoxLayout(frame);row.setContentsMargins(9,5,8,5);row.setSpacing(7)
            color=COLORS[member];frame.setStyleSheet('QFrame#activityRow {background: '+self.tint(color)+'; border: 1px solid #edf2f8; border-radius: 4px;}')
            row.addWidget(icon_label('aav' if member.startswith('drone') else member,color,25))
            label=QLabel(MEMBER_NAMES[member]);label.setStyleSheet('color: '+color+'; font-weight: 700;');row.addWidget(label)
            phase=QLabel('待分配');phase.setWordWrap(True);row.addWidget(phase,1)
            right_layout.addWidget(frame);self.action_rows[member]=(frame,phase)
        self.return_card=QFrame();self.return_card.setObjectName('returnCard');self.return_card.setStyleSheet('QFrame#returnCard {background: #fffbf0; border: 1px solid #f5cd79; border-radius: 5px;}')
        rr=QVBoxLayout(self.return_card);rr.setContentsMargins(11,10,11,10)
        self.return_title=QLabel('ⓘ 共同返航尚未放行');self.return_title.setStyleSheet('color: #b96900; font-weight: 700; font-size: 15px;');rr.addWidget(self.return_title)
        self.returns=QLabel('等待作业完成与必要结果收齐。');self.returns.setWordWrap(True);rr.addWidget(self.returns);right_layout.addWidget(self.return_card)
        self.return_button=QPushButton('确认共同返航');self.return_button.clicked.connect(self.return_home);rr.addWidget(self.return_button)
        self.end_button=QPushButton('结束本次会话');self.end_button.clicked.connect(self.end_session);rr.addWidget(self.end_button)
        self.return_button.setVisible(False);self.end_button.setVisible(False)
        recent=QHBoxLayout();recent.addWidget(icon_label('clock',size=23));label=QLabel('最近事件');label.setObjectName('section');recent.addWidget(label);recent.addStretch();right_layout.addLayout(recent)
        self.recent=QLabel('等待任务事件');self.recent.setWordWrap(True);self.recent.setMinimumHeight(68);right_layout.addWidget(self.recent)
        right_layout.addStretch()
        stop_row=QHBoxLayout();self.stop_button=QPushButton('▣  停止任务');self.stop_button.setObjectName('stop')
        self.stop_button.clicked.connect(self.stop);self.stop_button.setEnabled(False);stop_row.addWidget(self.stop_button,1)
        note=QLabel('须确认');note.setStyleSheet('color: #dc3346;');stop_row.addWidget(note);right_layout.addLayout(stop_row)
        body.addWidget(right);outer.addLayout(body,1)
        timeline=QFrame();timeline.setObjectName('panel');tl=QVBoxLayout(timeline);tl.setContentsMargins(16,8,16,6);tl.setSpacing(3)
        th=QHBoxLayout();th.addWidget(icon_label('clock',size=23));label=QLabel('协同执行时间线');label.setObjectName('section');th.addWidget(label);th.addStretch()
        small=QLabel('实色：已完成步骤　 描边：当前步骤　 浅色：预计');small.setObjectName('muted');th.addWidget(small);tl.addLayout(th)
        self.timeline=PlanTimeline();self.timeline.setMinimumHeight(158);self.timeline.setMaximumHeight(158);tl.addWidget(self.timeline)
        self.timeline_note=QLabel('同时放行返航，分别按实际到位完成。');self.timeline_note.setObjectName('muted');self.timeline_note.setAlignment(Qt.AlignRight);tl.addWidget(self.timeline_note)
        outer.addWidget(timeline)
        self.details=QPlainTextEdit();self.details.setReadOnly(True)
        self.details_dialog=QDialog(self);self.details_dialog.setWindowTitle('任务记录');self.details_dialog.resize(820,460)
        dl=QVBoxLayout(self.details_dialog);dl.addWidget(self.details)
        self.show_page(0)
        self.timer=QTimer(self);self.timer.timeout.connect(self.refresh);self.timer.start(500)
        self.update_region();self.refresh()

    @staticmethod
    def card(title,icon=None):
        frame=QFrame();frame.setObjectName('panel');layout=QVBoxLayout(frame)
        layout.setContentsMargins(14,11,14,10);layout.setSpacing(10)
        label=QLabel(title);label.setObjectName('section')
        if icon:
            row=QHBoxLayout();row.addWidget(icon_label(icon,size=22));row.addWidget(label);row.addStretch();layout.addLayout(row)
        else:layout.addWidget(label)
        return frame,layout

    @staticmethod
    def tint(color):
        c=QColor(color)
        return QColor(*[round(.07*v+.93*255) for v in (c.red(),c.green(),c.blue())]).name()

    def show_page(self,index):
        self.pages.setCurrentIndex(index);self.area_tab.setChecked(index==0);self.plan_tab.setChecked(index==1);self.preview_tab.setChecked(index==2)
        if index==2:self.preview_revision_shown=(self.state.get('pending_plan') or {}).get('revision')

    def select_mapping_layer(self,layer):
        self.deep_layer.setChecked(layer=='offshore_uuv');self.air_layer.setChecked(layer=='offshore_air')
        self.map.set_mapping_layer(layer);self.show_page(0)

    def show_details(self):
        self.details_dialog.show();self.refresh()

    def region_changed(self,center,radius):
        if self.submitted:
            self.draft_circle=(tuple(center),radius);self.update_draft_hint()
        else:self.center,self.radius=tuple(center),radius;self.update_region()

    def update_region(self):
        if self.select_region:
            self.region.setText('<b>圆形监测区</b><br>圆心 ({:.2f}, {:.2f}) m<br>半径 {:.2f} m'.format(*self.center,self.radius))
        else:self.region.setText('已加载配置中的监测区域\n'+self.base_request['request_id'])

    def edit_region(self):
        if self.submitted:
            if not self.state.get('session_phase'):return
            self.draft_circle=self.draft_circle or (self.center,self.radius)
            self.map.draft_circle=self.draft_circle;self.map.editing_draft=True
            self.map.editable=True;self.draft_controls.setVisible(True);self.update_draft_hint()
        self.show_page(0);self.map.setFocus();self.central_hint.setText('按下鼠标确定圆心，拖动确定半径。')

    def update_draft_hint(self):
        if self.draft_circle:
            center,radius=self.draft_circle
            self.draft_hint.setText('新区域草稿：({:.2f}, {:.2f}) m，半径 {:.2f} m'.format(*center,radius))
        else:self.draft_hint.setText('可在执行、驻留和返航中圈选下一任务区域。')
        queued=len(self.state.get('task_queue') or ())
        if queued:self.draft_hint.setText(self.draft_hint.text()+' · 已排队 {} 项'.format(queued))

    def view_current(self):
        self.map.editing_draft=False;self.map.draft_circle=None;self.map.editable=False
        self.show_page(0);self.map.update()

    def send_session_command(self,action,**values):
        if self.pending_command_id:return False
        command=dict(command_id=uuid.uuid4().hex,request_id=self.state.get('request_id'),action=action,**values)
        path=self.output/'operator-command.json';temp=path.with_suffix('.tmp')
        try:temp.write_text(json.dumps(command,ensure_ascii=False));temp.replace(path)
        except OSError as error:
            QMessageBox.warning(self,'命令未提交',str(error));return False
        self.pending_command_id=command['command_id'];self.refresh();return True

    def generate(self):
        if self.submitted:
            if not self.draft_circle or not self.state.get('session_phase'):return
            center,radius=self.draft_circle
            # Only the runner expands/solves a dynamic request. This validation
            # does not replace the active request, scene or received map cells.
            if not math.isfinite(radius) or not all(math.isfinite(v) for v in center) or not 2.<=radius<=10.:
                QMessageBox.warning(self,'区域暂不可用','圆形监测半径需在 2～10 模型米之间。');return
            policy='replace' if self.state.get('session_phase') in ('HOLDING','HOME') else self.policy_box.currentData()
            self.send_session_command('preview_region',center=list(center),radius_m=radius,policy=policy)
            return
        try:
            request,scene=(circle_joint_mission_mappings(self.base_request,self.base_scene,self.center,self.radius)
                if self.select_region else (self.base_request,self.base_scene))
            request_path=self.output/'ui-request.yaml';scene_path=self.output/'ui-scene.yaml'
            if request_path.exists() or scene_path.exists():raise ValueError('本次运行已提交区域，请使用新运行目录。')
            temp=request_path.with_suffix('.tmp');temp.write_text(self.yaml.safe_dump(request,allow_unicode=True,sort_keys=False))
            load_request(temp);temp.replace(request_path)
            # Scene is the last file: its atomic rename releases the launcher.
            temp=scene_path.with_suffix('.tmp');temp.write_text(self.yaml.safe_dump(scene,allow_unicode=True,sort_keys=False));temp.replace(scene_path)
            self.base_request=request;self.base_scene=scene
            self.map.scene=scene['scene'];self.map.online_mapping=bool(scene['scene'].get('online_mapping',False))
            self.map.set_mapping_request(request);self.mapping_layer_bar.setVisible(self.map.online_mapping)
            self.point_count=sum(len(r.get('interest_points',())) for r in request['regions'])
            self.submitted=True;self.refresh()
        except (ValueError,OSError,KeyError) as error:
            QMessageBox.warning(self,'区域暂不可用',str(error)+'\n请调整区域后重新生成。')

    def confirm(self):
        # Re-read authority when clicking, not a cached rendering of an old Plan.
        try:state=json.loads((self.output/'metrics.json').read_text())
        except (OSError,ValueError):return
        pending=state.get('pending_plan') or {}
        if state.get('session_phase') and self.pages.currentIndex()==2:
            if (pending.get('state')!='READY' or pending.get('revision')!=self.preview_revision_shown or
                    state.get('request_id')!=self.state.get('request_id') or
                    self.pending_command_id or pending.get('revision')==self.confirmed_preview_revision):
                self.refresh();return
            if self.send_session_command('confirm_plan',preview_revision=pending['revision']):
                self.confirmed_preview_revision=pending['revision']
            return
        if (self.confirmation_sent or state.get('status')!='AWAITING_CONFIRMATION' or
                state.get('request_id')!=self.state.get('request_id') or
                state.get('plan_revision')!=self.state.get('plan_revision')):
            self.refresh();return
        air=next((r for r in self.base_request.get('regions',()) if r.get('region_id')=='offshore_air'),None)
        if air and air.get('shape')=='CIRCLE':
            center=tuple(air['center'][:2]);radius=float(air['radius_m'])
            if (self.center!=center or self.radius!=radius or
                    self.map.center!=center or self.map.radius!=radius):
                self.center,self.radius=center,radius;self.map.center,self.map.radius=center,radius
                self.map.dragging=False;self.update_region();self.map.update()
                return  # operator must see the restored authoritative region before confirming
        command=dict(decision='confirm',request_id=state['request_id'],plan_revision=state['plan_revision'])
        path=self.output/'operator-confirmation.json';temp=path.with_suffix('.tmp')
        temp.write_text(json.dumps(command));temp.replace(path)
        self.confirmation_sent=True;self.confirm_button.setEnabled(False)
        self.confirm_button.setText('已确认，等待派发')

    def return_home(self):
        if self.state.get('session_phase')!='HOLDING':return
        if QMessageBox.question(self,'确认共同返航','当前区域任务已结束。现在从驻留位置共同返航？',
                QMessageBox.Yes|QMessageBox.No,QMessageBox.No)==QMessageBox.Yes:
            self.send_session_command('return_home')

    def end_session(self):
        if self.state.get('session_phase')!='HOME':return
        self.send_session_command('end_session')

    def subscribe_display(self):
        """Passive live modes/support only; task progress still comes from runner."""
        try:
            from diagnostic_msgs.msg import DiagnosticArray
            from std_msgs.msg import String
            from sensor_msgs.msg import PointCloud2
            from sensor_msgs import point_cloud2
            if not rospy.core.is_initialized():rospy.init_node('joint_task_panel',anonymous=True,disable_signals=True)
            def diagnostic(member,message):
                values={v.key:v.value for status in message.status for v in status.values}
                with self.telemetry_lock:self.telemetry[member]=(time.monotonic(),values)
            def progress(message):
                try:value=json.loads(message.data)
                except ValueError:return
                with self.telemetry_lock:self.transport=dict(value,received_monotonic=time.monotonic())
            def survey(message):
                if message.header.frame_id!='world' or not message.header.stamp.to_sec():return
                if not {'x','y','z','intensity'}.issubset({field.name for field in message.fields}):return
                hits={}
                for x,y,z,hit in point_cloud2.read_points(message,field_names=('x','y','z','intensity'),skip_nans=True):
                    if hit==1.0 and math.isfinite(x) and math.isfinite(y):
                        # Persistent 25 cm XY sampling is only for bounded UI
                        # drawing cost; all task completion stays in runner.
                        hits[(math.floor(x/.25),math.floor(y/.25))]=(x,y)
                with self.telemetry_lock:self.survey_hits.update(hits)
            for member in MEMBERS:
                topic='/'+member+('_qn' if member.startswith('drone') else '')+'/diagnostics'
                self.subscribers.append(rospy.Subscriber(topic,DiagnosticArray,
                    lambda msg,m=member:diagnostic(m,msg),queue_size=1))
                if self.map.online_mapping:
                    scan='/'+member+('_qn' if member.startswith('drone') else '')+'/survey_cloud'
                    self.subscribers.append(rospy.Subscriber(scan,PointCloud2,survey,queue_size=1))
            self.subscribers.append(rospy.Subscriber('/scene/delivery_progress',String,progress,queue_size=1))
        except Exception:
            pass  # Missing display telemetry is shown as unknown, never as AIR/ready.

    def stop(self):
        if self.stop_sent or not self.state.get('status','').startswith('RUNNING'):return
        reply=QMessageBox.question(self,'停止当前任务','停止当前请求？平台将执行已有停止处置，未确认终态的成员保留占用。',
            QMessageBox.Yes|QMessageBox.No,QMessageBox.No)
        if reply!=QMessageBox.Yes:return
        path=self.output/'operator-stop.json';temp=path.with_suffix('.tmp')
        temp.write_text(json.dumps(dict(request_id=self.state['request_id'])));temp.replace(path)
        self.stop_sent=True;self.stop_button.setText('已请求停止');self.stop_button.setEnabled(False)

    def refresh(self):
        try:
            path=self.output/'metrics.json';stamp=path.stat().st_mtime_ns
            if stamp!=self.last_snapshot:
                self.state=json.loads(path.read_text());self.last_snapshot=stamp;self.last_update=time.monotonic()
        except (OSError,ValueError):pass
        state=self.state;status=state.get('status','STARTING' if self.submitted else 'SELECT_REGION')
        session_phase=state.get('session_phase','')
        session_closed=status=='SESSION_ENDED'
        ack=state.get('command_ack') or {}
        if self.pending_command_id and ack.get('command_id')==self.pending_command_id:self.pending_command_id=None
        active_request=state.get('active_request') or {}
        if active_request.get('request_id')==state.get('request_id') and active_request.get('request_id')!=self.active_request_id:
            previous_request_id=self.active_request_id
            self.active_request_id=active_request['request_id'];self.base_request=active_request
            air=next((r for r in active_request.get('regions',()) if r.get('region_id')=='offshore_air'),None)
            if air and air.get('shape')=='CIRCLE':
                self.center=tuple(air['center'][:2]);self.radius=air['radius_m']
                self.map.center=self.center;self.map.radius=self.radius
                if previous_request_id and self.draft_circle and self.draft_circle==(self.center,self.radius):
                    self.draft_circle=None;self.map.draft_circle=None;self.map.editing_draft=False;self.show_page(0)
            self.map.set_mapping_request(active_request)
            self.point_count=sum(len(r.get('interest_points',())) for r in active_request.get('regions',()))
            self.update_region();self.confirmation_sent=bool(state.get('confirmation'));self.stop_sent=False
        # Reattaching to a request launched through the terminal has no
        # ui-scene.yaml. Its authoritative Plan still makes the input read-only.
        if (state.get('request_id')==self.base_request.get('request_id') and
                (state.get('plan') or state.get('selected_plan'))):
            self.submitted=True
            air=next((r for r in self.base_request.get('regions',()) if r.get('region_id')=='offshore_air'),None)
            if air and air.get('shape')=='CIRCLE':
                self.center,self.radius=tuple(air['center'][:2]),float(air['radius_m'])
                self.map.center,self.map.radius=self.center,self.radius
                self.update_region()
        kind=terminal_kind(state)
        if session_phase in ('EXECUTING','HOLDING','RETURNING','HOME') and kind=='success':kind=''
        terminal=bool(kind) or session_closed
        try:exit_code=(self.output/'runner-exit-code.txt').read_text().strip()
        except OSError:exit_code=None
        failed_start=(self.output/'session-ended').exists() and not terminal
        stale=(status.startswith('RUNNING') or status=='AWAITING_CONFIRMATION' or (session_phase and not terminal)) and time.monotonic()-self.last_update>5
        names={'SELECT_REGION':'等待选择区域','STARTING':'正在启动仿真','STANDBY':'等待平台就绪',
            'PLANNING':'正在联合求解','AWAITING_CONFIRMATION':'等待确认执行',
            'RUNNING':'协同作业中','RUNNING_RETEST':'补测执行中','NOT_CONFIRMED':'任务未确认',
            'UNKNOWN_LOCKED':'任务异常锁定','FAIL':'任务未完成','FAILED':'任务未完成',
            'PASS_GEOMETRIC_PROXY_QUALIFICATION':'协同任务完成'}
        title=names.get(status,status)
        if not kind and session_phase:
            title={'EXECUTING':'协同作业中','HOLDING':'监测完成 · 原地驻留' if state.get('monitoring_complete') is True else '作业结束 · 仍有缺测','RETURNING':'共同返航中',
                'HOME':'已返回 · 等待新任务','SWITCHING':'正在安全切换任务'}.get(session_phase,title)
        if session_closed:title='会话已结束'
        if kind=='success':title='协同任务完成'
        elif kind=='failure':title='任务异常锁定' if status=='UNKNOWN_LOCKED' else '任务未完成'
        elif kind=='stopped':title='任务已停止 · 状态待确认' if state.get('resource_locks') else '任务已停止'
        if failed_start or (exit_code is not None and not terminal):title='任务进程已退出'
        elif stale:title='状态更新中断'
        elif state.get('operator_stop_requested') and not terminal:title='停止处置中'
        self.status.setText('● '+title)
        color=('#b84537' if failed_start or stale or (exit_code is not None and not terminal) else
            '#738197' if session_closed else
            {'failure':'#b84537','stopped':'#b96900','unconfirmed':'#738197','success':'#079b62'}.get(kind,
                '#b96900' if state.get('operator_stop_requested') else '#0967ff'))
        self.status.setStyleSheet('color: '+color+'; font-weight: 750; font-size: 23px;')
        session_open=bool(session_phase) and not terminal and not stale and not failed_start and exit_code is None
        pending=state.get('pending_plan') or {};pending_state=pending.get('state','')
        self.edit_button.setEnabled(self.select_region and (not self.submitted or session_open))
        self.edit_button.setText('圈选新任务' if session_open else '已提交区域' if self.submitted else '编辑区域')
        self.generate_button.setText('生成新任务方案' if session_open else '生成协同方案')
        self.generate_button.setEnabled(not self.submitted or (session_open and bool(self.draft_circle) and not self.pending_command_id))
        self.map.editable=self.select_region and (not self.submitted or (session_open and self.map.editing_draft))
        if not self.map.editable:self.map.dragging=False
        self.draft_controls.setVisible(session_open)
        self.update_draft_hint()
        self.policy_box.setVisible(session_phase not in ('HOLDING','HOME'))
        self.policy_box.setEnabled(not self.pending_command_id)
        self.current_button.setEnabled(session_open)
        self.preview_tab.setVisible(bool(pending))
        if not pending and self.pages.currentIndex()==2:self.show_page(0)
        if pending:
            region=pending.get('region') or {};center=region.get('center') or ('—','—')
            self.preview_summary.setText('新区域方案 · {} · {}\n圆心 ({}, {}) m · 半径 {} m\n请求 {} · 版本 {}\n{}'.format(
                {'READY':'等待确认','PLANNING':'正在求解','QUEUED':'已排队','STALE':'状态已变化，需重新生成','ERROR':'方案生成失败',
                 'WAITING_SAFE_STATE':'等待当前模式转换结束后生成方案',
                 'ADOPTED':'已采用'}.get(pending_state,pending_state),
                '立即替换当前任务' if pending.get('policy')=='replace' else '当前任务完成后执行',
                center[0],center[1],region.get('radius_m','—'),
                pending.get('request_id','—'),pending.get('revision','—'),pending.get('error','')))
            preview_items=(pending.get('plan') or {}).get('items',[])
            self.preview_table.setRowCount(len(preview_items))
            for row,item in enumerate(preview_items):
                labels=[]
                for step in item.get('execution_steps',()):
                    label=step_label(step)
                    if not labels or labels[-1]!=label:labels.append(label)
                values=('、'.join(MEMBER_NAMES.get(m,m) for m in item.get('coalition',())),activity_role(item),
                    ' → '.join(labels),
                    '{:.0f}～{:.0f} s'.format(item.get('planned_start',0),item.get('planned_finish',0)))
                for col,value in enumerate(values):self.preview_table.setItem(row,col,QTableWidgetItem(value))
            if pending_state=='READY' and self.preview_revision_shown!=pending.get('revision'):
                self.show_page(2)
        preview_ready=(session_open and self.pages.currentIndex()==2 and pending_state=='READY' and
            not self.pending_command_id and pending.get('revision')!=self.confirmed_preview_revision)
        self.confirm_button.setEnabled(preview_ready or (status=='AWAITING_CONFIRMATION' and not self.confirmation_sent and not stale and not failed_start and exit_code is None))
        if self.pages.currentIndex()==2 and pending:
            self.confirm_button.setText('确认新方案' if pending_state=='READY' else '新方案已排队' if pending_state=='QUEUED' else '等待新方案')
        else:self.confirm_button.setText('已确认执行' if state.get('confirmation') else '确认并执行')
        if state.get('confirmation_error'):
            self.confirmation_sent=False;self.confirm_button.setText('重新确认')
        if state.get('confirmation') and self.pages.currentIndex()!=2:self.confirm_button.setText('已确认执行')
        self.return_button.setVisible(session_phase=='HOLDING' and not session_closed);self.return_button.setEnabled(session_open and not self.pending_command_id)
        self.end_button.setVisible(session_phase=='HOME' and not session_closed);self.end_button.setEnabled(session_open and not self.pending_command_id)
        self.stop_button.setEnabled(status.startswith('RUNNING') and not self.stop_sent and not stale)
        stage=(4 if state.get('joint_return_release_at_ros_s') or kind=='success' else
            3 if status.startswith('RUNNING') or state.get('confirmation') or state.get('step_results') or state.get('current_actions') else
            2 if status=='AWAITING_CONFIRMATION' or kind=='unconfirmed' else 1 if self.submitted else 0)
        for i,(label,name) in enumerate(self.workflow):
            digit=('①','②','③','④','⑤')[i]
            label.setText(digit+'  '+name+('  ✓' if i<stage or kind=='success' else '  !' if i==stage and kind=='failure' else ''))
            label.setStyleSheet('font-size: 16px; color: '+(color if i==stage else '#112951' if i<stage else '#8998ac')+
                '; font-weight: '+('750' if i==stage else '500')+';')
        items=(state.get('plan') or state.get('selected_plan') or {}).get('items',[])
        failure=failure_summary(state,items,self.base_scene['scene'])
        self.map.plan_items=(state.get('selected_plan') or {}).get('items',items);self.map.update()
        actions=state.get('current_actions',{});products=state.get('received_products',{})
        self.map.set_received_mapping(products,state.get('request_id',''))
        required_points={p['point_id'] for region in self.base_request.get('regions',()) for p in region.get('interest_points',())}
        received={p['point_id'] for p in products.values() if p.get('observed') is True and p.get('request_id')==state.get('request_id')}
        observed=received|{point for r in state.get('received_terminal_reports',{}).values() for point in r.get('observed_ids',())
            if point in required_points}
        for label,bar,count in ((self.received,self.progress,len(received)),(self.observed,self.observation_progress,len(observed))):
            label.setText('{} / {}'.format(count,self.point_count) if self.submitted else '— / —')
            bar.setRange(0,max(1,self.point_count));bar.setValue(count)
        if state and not terminal and not self.subscriptions_started:
            self.subscriptions_started=True;threading.Thread(target=self.subscribe_display,daemon=True).start()
        with self.telemetry_lock:
            modes=dict(self.telemetry);transport=dict(self.transport)
            self.map.observed_points=tuple(self.survey_hits.values())
        phases={'MOVING':'空中转场','HOLDING':'到位确认','AIR_MOVE':'空中转场','ENTER_WATER':'入水中',
            'EXIT_WATER':'出水中','WATER_PATH':'水下航行 / 观测','SURFACE_PATH':'水面转场','PREPARED':'等待启动',
            'WAITING_FOR_SUPPORT_CLEARANCE':'等待无人船到达侧向支援位',
            'SUPPORT_CLEARANCE_CONFIRMED':'支援到位，释放跨介质进场',
            'REGION_MAPPING':'区域扫描建图','WAIT_LOCAL_OBSERVATION':'等待安全局部运动',
            'TRIM_PROPULSION':'减速与终态确认','COAST_STOP':'滑行减速',
            'WAITING_FOR_RECEIPTS':'等待作业结果','WAITING_FOR_GROUP_RETURN':'等待共同返航',
            'DISPATCHING':'正在派发','SAFETY_HOLD':'安全保持','UNKNOWN_LOCKED':'异常锁定'}
        selected={m for item in items for m in item.get('coalition',())}
        for row,member in enumerate(MEMBERS):
            item=next((i for i in reversed(items) if member in i.get('coalition',()) and
                       i['execution_id'] in actions),None)
            if item is None:item=next((i for i in reversed(items) if member in i.get('coalition',())),None)
            phase='岸边待命' if items else '待分配';role='备用' if items else '待分配';resource='未占用' if items else '—'
            if item:
                role=activity_role(item);action=actions.get(item['execution_id'],{})
                phase=phases.get(action.get('phase'),{'PLANNED':'待确认' if status=='AWAITING_CONFIRMATION' else '等待派发',
                    'COMPLETED':'活动已完成','FAILED':'执行失败','UNKNOWN_LOCKED':'异常锁定','RUNNING':'执行中'}.get(item.get('status'),'状态待同步'))
                index=action.get('step');steps=item.get('execution_steps',())
                if isinstance(index,int) and 0<=index<len(steps) and action.get('phase') not in ('WAITING_FOR_RECEIPTS','WAITING_FOR_GROUP_RETURN','UNKNOWN_LOCKED'):
                    step=step_label(steps[index])
                    if step in ('返航','扫测','等待出发','编队集结','编队返航','编队返回部署区'):
                        phase=step+('中' if step!='等待出发' else '')
                resource=('异常锁定' if status=='UNKNOWN_LOCKED' else '执行占用') if item['executor_id'] in state.get('resource_locks',()) else '未占用'
                if item.get('status') in ('COMPLETED','CANCELED_BY_REPLACEMENT'):resource='已释放'
                elif terminal:
                    phase=('异常锁定' if item['executor_id'] in state.get('resource_locks',()) else
                        '未执行' if kind=='unconfirmed' else '已停止 · 终态待确认' if kind=='stopped' else '任务未完成')
                if session_phase=='HOLDING' and not terminal:phase='目标区驻留'
                elif session_phase=='HOME' and not kind:phase='已返回 · 待命'
                elif session_phase=='SWITCHING' and not terminal:
                    phase=('已安全停止' if item.get('status')=='CANCELED_BY_REPLACEMENT' else
                        '停止并确认终态' if action else '等待切换')
            own={p['point_id'] for p in products.values() if p.get('producer')==member and p.get('observed') is True}
            for col,value in enumerate((MEMBER_NAMES[member],role,phase,str(len(own))+' 份' if member!='usv' else '共享支援',resource)):
                cell=QTableWidgetItem(value);cell.setForeground(QColor(COLORS[member] if col==0 else '#263d53'));self.table.setItem(row,col,cell)
            role_label,mode_label=self.member_rows[member];role_label.setText(role if role!='备用' else '岸边待命')
            stamp,values=modes.get(member,(0,{}));mode=values.get('actual_mode','') if time.monotonic()-stamp<2 else ''
            mode_label.setText(('水下' if member=='uuv' and mode=='WATER' else {'SURFACE':'水面','TRANSITION':'转换中'}.get(mode,mode)) or '—')
            mode_label.setToolTip('实际模式' if mode else '实际模式尚未收到或已过期')
            frame,phase_label=self.action_rows[member];phase_label.setText(phase)
            frame.setVisible(member in selected if items else member!='drone_2')
        if items:
            active=sum(a.get('phase') not in ('UNKNOWN_LOCKED','FAILED','SAFETY_HOLD') for a in actions.values())
            completed=sum(i.get('status')=='COMPLETED' for i in items)
            detail=('{} 项活动完成 · {} 台待命'.format(completed,len(set(MEMBERS)-selected)) if kind=='success' else
                '{} / {} 项活动完成 · {} 项占用待确认'.format(completed,len(items),len(state.get('resource_locks',()))) if kind in ('failure','stopped') else
                '{} 项计划活动 · 未确认执行'.format(len(items)) if kind=='unconfirmed' else
                '{} 项活动执行 · {} 台待命'.format(active,len(set(MEMBERS)-selected)) if active else
                '{} 项计划活动 · {} 台待命'.format(len(items),len(set(MEMBERS)-selected)))
            self.status_detail.setText(detail)
            if session_phase=='HOLDING':self.status_detail.setText('{} 台目标区驻留 · {} 项后续任务已排队'.format(len(selected),len(state.get('task_queue') or ())))
            elif session_phase=='HOME':self.status_detail.setText('{} 台已返回 · {} 台待命'.format(len(selected),len(set(MEMBERS)-selected)))
            elif session_phase=='SWITCHING':self.status_detail.setText('{} 项旧活动已安全停止 · {} 项占用待确认'.format(
                sum(i.get('status')=='CANCELED_BY_REPLACEMENT' for i in items),len(state.get('resource_locks',()))))
        else:self.status_detail.setText('生成方案后核对分工，确认后启动。')
        plan_wall=state.get('planning_wall_s')
        self.plan_info.setText('方案版本 {} · 决策 {:.2f} 秒'.format(state.get('plan_revision',0),plan_wall)
            if plan_wall is not None else '生成后显示方案版本与决策用时')
        service_fresh=time.monotonic()-transport.get('received_monotonic',0)<2
        support_active=service_fresh and transport.get('support_active',False)
        self.support_status.setText('支援已结束' if kind=='success' or session_phase=='HOME' else '支援已中止' if kind in ('failure','stopped') else
            '未确认执行' if kind=='unconfirmed' else '● USV 已到位' if support_active else '● 支援待就绪' if service_fresh else '状态待同步')
        self.support_status.setStyleSheet('color: '+(color if terminal else '#069e67' if support_active else '#8190a7')+'; background: #effaf5; border-radius: 11px; padding: 4px 7px; font-size: 11px;')
        clients=[MEMBER_NAMES.get(i.get('coalition',[''])[0],'') for i in items if i.get('fulfills_task',True)]
        self.support_members.setText('服务对象：'+'、'.join(clients) if clients else '等待方案选择服务对象')
        self.cooperation.setText('USV 同时支援作业平台\n结果收齐、作业到位后共同返航' if items else '按当前状态选择分工与支援\n必要条件满足后共同返航')
        if session_phase:self.cooperation.setText('USV 同时支援作业平台\n完成后驻留，可继续新任务或确认返航')
        returned=state.get('return_completion',{})
        if session_phase=='SWITCHING':
            self.return_title.setText('ⓘ 新任务已确认 · 正在切换')
            self.returns.setText('等待旧动作安全结束，未确认占用保留。')
        elif session_phase=='HOLDING':
            self.return_title.setText('ⓘ '+('监测完成' if state.get('monitoring_complete') is True else '作业结束 · 仍有缺测')+' · 等待安排')
            self.returns.setText('作业终态驻留；可圈选新区域，或确认返航。' if state.get('monitoring_complete') is True else
                '缺测仍保留；可重新监测或确认返航。')
        elif session_phase=='HOME':
            self.return_title.setText('✓ 共同返航已完成');self.returns.setText('会话已结束，当前画面保留最终任务记录。' if session_closed else '可从当前部署位置生成下一监测任务，也可结束会话。')
        elif status.startswith('PASS'):
            self.return_title.setText('✓ 共同返航已完成');self.returns.setText('规定返回已确认：{} / {}'.format(sum(bool(v.get('completed')) for v in returned.values()),len(items)))
        elif state.get('joint_return_release_at_ros_s'):
            self.return_title.setText('✓ 共同返航已放行');self.returns.setText('分别按实际到位完成，尚未完成的成员继续执行。')
        else:
            self.return_title.setText('ⓘ 共同返航尚未放行');self.returns.setText('等待作业完成及必要结果收齐。')
        if kind in ('failure','stopped','unconfirmed'):
            self.return_title.setText('! 任务未完成' if kind=='failure' else 'ⓘ 任务已停止' if kind=='stopped' else 'ⓘ 未确认执行')
            self.returns.setText(('首个异常：'+failure if failure and kind=='failure' else
                '已请求停止，未确认终态的成员保留占用。' if kind=='stopped' else
                '本次方案未获确认，未启动执行。' if kind=='unconfirmed' else '未收到任务完成依据，请查看任务记录。'))
        self.return_title.setStyleSheet('color: '+(color if terminal else '#b96900')+'; font-weight: 700; font-size: 15px;')
        self.central_hint.setText('首个异常：'+failure if kind=='failure' and failure else
            '任务已停止，请查看右侧处置与资源状态。' if kind=='stopped' else
            '本次任务未确认执行。' if kind=='unconfirmed' else
            '请核对当前分工，再确认执行。' if status=='AWAITING_CONFIRMATION' else
            '区域已提交，正在启动独立 RViz。' if status in ('STARTING','STANDBY') else
            ('切片占据建图（几何采样）· 点云不是完整三维重建证明' if self.map.online_mapping else
             '二维区域与方案视图 · 实际仿真在独立 RViz 中显示') if self.submitted else
            '在地图上拖动圈选区域，再生成协同方案。')
        if session_open:
            self.central_hint.setText('正在提交命令，等待任务程序接纳。' if self.pending_command_id else
                ('监测完成，保持驻留；可圈选新区域或确认返航。' if state.get('monitoring_complete') is True else
                 '作业结束，仍有缺测；可重新监测或确认返航。') if session_phase=='HOLDING' else
                '黄色虚线为新任务草稿；当前任务与实际收件保持显示。' if self.map.editing_draft else
                '可圈选新区域；生成和预览方案不会中断当前任务。')
        if state.get('command_error'):
            self.central_hint.setText('命令未采用：'+str(state['command_error']))
        if session_closed:self.central_hint.setText('会话已结束；本窗口保留最终成果与执行记录。')
        self.central_hint.setStyleSheet('color: '+(color if terminal else '#8390a4')+'; font-size: 11px;')
        self.connection.setText('● 会话已结束' if session_closed else '● '+{'success':'任务已完成','failure':'任务异常锁定' if status=='UNKNOWN_LOCKED' else '任务未完成',
            'stopped':'任务已停止','unconfirmed':'任务未确认'}[kind] if terminal else '● 进程已退出' if failed_start or exit_code is not None else
            '● 更新中断' if stale else '● 仿真已连接' if state else '● 等待仿真')
        badge_color=color if terminal or failed_start or stale or exit_code is not None else '#079b62' if state else '#8794a8'
        self.connection.setStyleSheet('color: '+badge_color+'; background: '+self.tint(badge_color)+
            '; border: 1px solid '+self.tint(badge_color)+'; border-radius: 13px; padding: 4px 10px;')
        epoch=(state.get('confirmation') or {}).get('at_ros_s')
        elapsed=max(0,state.get('updated_at_ros_s',epoch or 0)-(epoch or 0)) if epoch else 0
        self.elapsed.setText('已运行 {:02d}:{:02d}'.format(int(elapsed)//60,int(elapsed)%60) if epoch else '尚未开始')
        events=[]
        if epoch:
            for product in products.values():events.append((product['received_at'],'母船收到'+('深水' if product.get('producer')=='uuv' else '观测')+'结果'))
            for result in state.get('step_results',()):
                if not result.get('verified') or not result.get('result_received_at'):continue
                item=next((i for i in items if i['execution_id']==result['activity_id']),None)
                if item:
                    steps=item.get('execution_steps',())
                    final_return=(steps and step_label(steps[-1])=='返航' and
                        result['execution_id']==item['execution_id']+':step:'+str(len(steps)-1))
                    events.append((result['result_received_at'],MEMBER_NAMES.get(item['coalition'][0],'平台')+
                        (' 返回完成' if final_return else ' 完成作业步骤')))
            if state.get('joint_return_release_at_ros_s'):events.append((state['joint_return_release_at_ros_s'],'全体共同返航已放行'))
        self.recent.setText('\n'.join('{:02d}:{:02d}  {}'.format(int(max(0,t-epoch))//60,int(max(0,t-epoch))%60,text)
            for t,text in sorted(events,reverse=True)[:3]) or '等待实际任务事件')
        self.timeline.state=state;self.timeline.update()
        if self.details_dialog.isVisible():
            text=('首个异常：'+failure+'\n' if failure else '')+(state.get('failure_reason') or state.get('confirmation_error') or '当前无失败记录。')
            if state.get('command_error'):text+='\n最近命令：'+str(state['command_error'])
            if state.get('request_id'):text+='\n请求：'+state['request_id']
            if exit_code is not None:text+='\n任务进程退出码：'+exit_code
            for filename in ('runner.log','launch.log'):
                path=self.output/filename
                if path.exists():
                    with path.open('rb') as stream:
                        stream.seek(max(0,path.stat().st_size-2400));text+='\n'+stream.read().decode('utf8','replace')
            self.details.setPlainText(text)

    def closeEvent(self,event):
        if self.submitted and not self.confirmation_sent and not self.state.get('confirmation'):
            (self.output/'operator-closed').touch()
        event.accept()


class MissionConsole(QWidget):
    def __init__(self,request,output,serial=True,smoke=False):
        super().__init__()
        self.output=output
        self.serial=serial
        self.process=None
        self.log=None
        self.state={}
        self.submitted=False
        self.setWindowTitle('Heformation — 任务预览、确认与执行')
        self.resize(1180,760)
        layout=QVBoxLayout(self)
        layout.addWidget(QLabel('当前接线：三 AAV 请求；五平台/受限通信尚未完成在线验收。'))
        self.status=QLabel('待命：编辑或加载请求，再预览计划。未确认不派发。')
        layout.addWidget(self.status)
        split=QSplitter()
        self.editor=QPlainTextEdit(request.read_text())
        self.view=QPlainTextEdit()
        self.view.setReadOnly(True)
        self.view.setPlaceholderText('这里显示 runner 的实际计划、当前动作、接收结果和资源占用。')
        split.addWidget(self.editor)
        split.addWidget(self.view)
        layout.addWidget(split,1)
        row=QHBoxLayout()
        self.load_button=QPushButton('加载请求')
        self.preview_button=QPushButton('预览实际计划')
        self.confirm_button=QPushButton('确认并执行此计划')
        self.cancel_button=QPushButton('取消当前显示的动作')
        self.confirm_button.setEnabled(False)
        self.cancel_button.setEnabled(False)
        for button in [self.load_button,self.preview_button,self.confirm_button,self.cancel_button]:row.addWidget(button)
        layout.addLayout(row)
        layout.addWidget(QLabel('取消不等于瞬时停止；关闭窗口不结束已确认的执行。恢复需要整链重启。'))
        self.load_button.clicked.connect(self.load)
        self.preview_button.clicked.connect(self.preview)
        self.confirm_button.clicked.connect(self.confirm)
        self.cancel_button.clicked.connect(self.cancel)
        self.timer=QTimer(self)
        self.timer.timeout.connect(self.refresh)
        if not smoke:self.timer.start(500)

    def load(self):
        name,_=QFileDialog.getOpenFileName(self,'选择请求','','YAML (*.yaml *.yml)')
        if name:self.editor.setPlainText(Path(name).read_text())

    def preview(self):
        if self.process is not None or self.submitted:return
        try:
            self.output.mkdir(parents=True,exist_ok=True)
            if (self.output/'metrics.json').exists():
                raise ValueError('此目录已有请求记录，请为新批次重启整链并使用新实验目录。')
            request=self.output/'ui-request.yaml'
            request.write_text(self.editor.toPlainText())
            load_request(request)
            self.log=(self.output/'runner-ui.log').open('w')
            # A detached runner survives destruction of the Qt window/process.
            self.process=subprocess.Popen([sys.executable,str(Path(__file__).with_name('formation_mission_runner.py')),
                '_planning_mode:=executor','_request_file:='+str(request),
                '_output_dir:='+str(self.output),'_executor_serial:='+str(self.serial).lower()],
                stdin=subprocess.PIPE,stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True)
            self.editor.setReadOnly(True)
            self.preview_button.setEnabled(False)
            self.load_button.setEnabled(False)
            self.status.setText('等待执行端就绪并生成计划；运动尚未派发。')
        except Exception as error:
            self.status.setText(str(error))

    def refresh(self):
        try:
            raw=rospy.get_param('/formation_mission_runner/task_state',None)
            if raw is None:return
            self.state=json.loads(raw)
            self.view.setPlainText(json.dumps(self.state,ensure_ascii=False,indent=2))
            status=self.state.get('status','UNKNOWN')
            self.status.setText('任务权威状态：'+status)
            awaiting=(status=='AWAITING_CONFIRMATION' and self.process is not None and self.process.poll() is None)
            self.confirm_button.setEnabled(awaiting and not self.submitted)
            actions=list(self.state.get('current_actions',{}).values())
            if self.state.get('current_action'):actions.append(self.state['current_action'])
            self.cancel_button.setEnabled(status=='RUNNING' and bool(actions) and
                                          all(a.get('goal_id') for a in actions))
            if self.process and self.process.poll() is not None:
                self.status.setText('任务进程已结束，退出码 '+str(self.process.returncode)+'；状态：'+status)
        except Exception as error:
            self.status.setText('任务状态暂不可读：'+str(error))

    def confirm(self):
        if self.submitted or self.state.get('status')!='AWAITING_CONFIRMATION' or not self.process:return
        try:
            self.process.stdin.write(b'yes\n')
            self.process.stdin.flush()
            self.submitted=True
            self.confirm_button.setEnabled(False)
        except (BrokenPipeError,OSError) as error:self.status.setText(str(error))

    def cancel(self):
        actions=list(self.state.get('current_actions',{}).values())
        if self.state.get('current_action'):actions.append(self.state['current_action'])
        targets={(a['endpoint'],a['goal_id']) for a in actions if a.get('endpoint') and a.get('goal_id')}
        def publish():
            for endpoint,goal_id in targets:
                sender=rospy.Publisher(endpoint.rstrip('/')+'/cancel',GoalID,queue_size=1)
                deadline=time.monotonic()+2.
                while sender.get_num_connections()==0 and time.monotonic()<deadline:time.sleep(.02)
                sender.publish(GoalID(id=goal_id))
        threading.Thread(target=publish,daemon=True).start()
        self.status.setText('已请求取消当前端点；等待任务层报告实际处置与资源状态。')

    def closeEvent(self,event):
        # Before confirmation EOF means NOT_CONFIRMED. Afterwards the runner no
        # longer consumes stdin and continues independently of this window.
        if self.process and self.process.stdin:self.process.stdin.close()
        if self.log:self.log.close()
        event.accept()


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--request',type=Path,required=True)
    p.add_argument('--output',type=Path)
    p.add_argument('--serial',choices=['true','false'],default='true')
    p.add_argument('--smoke-image',type=Path)
    p.add_argument('--select-circle',action='store_true')
    p.add_argument('--joint-panel',action='store_true')
    p.add_argument('--select-region',choices=('true','false'),default='true')
    p.add_argument('--scene',type=Path)
    p.add_argument('--output-request',type=Path)
    p.add_argument('--output-scene',type=Path)
    p.add_argument('--circle',nargs=3,type=float,metavar=('X','Y','R'))
    args=p.parse_args()
    if args.circle is not None:
        if args.scene is None or args.output_request is None or args.output_scene is None:
            p.error('--circle requires --scene, --output-request and --output-scene')
        import yaml
        request,scene=circle_joint_mission_mappings(yaml.safe_load(args.request.read_text()),
            yaml.safe_load(args.scene.read_text()),args.circle[:2],args.circle[2])
        args.output_request.write_text(yaml.safe_dump(request,allow_unicode=True,sort_keys=False))
        args.output_scene.write_text(yaml.safe_dump(scene,allow_unicode=True,sort_keys=False))
        load_request(args.output_request)
        return 0
    if not args.smoke_image and not args.select_circle and not args.joint_panel:
        rospy.init_node('mission_console',disable_signals=True)
    app=QApplication(sys.argv[:1])
    cjk=Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    if cjk.is_file():
        font_id=QFontDatabase.addApplicationFont(str(cjk))
        families=QFontDatabase.applicationFontFamilies(font_id)
        if families:app.setFont(QFont(families[0],10))
    if args.joint_panel:
        if args.scene is None or args.output is None:p.error('--joint-panel requires --scene and --output')
        window=JointMissionPanel(args.request,args.scene,args.output,args.select_region=='true')
    elif args.select_circle:
        if args.scene is None or args.output_request is None or args.output_scene is None:
            p.error('--select-circle requires --scene, --output-request and --output-scene')
        window=CircleMissionSelector(args.request,args.scene,args.output_request,args.output_scene,
                                     bool(args.smoke_image))
    else:
        if args.output is None:p.error('--output is required for the mission console')
        window=MissionConsole(args.request,args.output,args.serial=='true',bool(args.smoke_image))
    window.show()
    if args.smoke_image:
        app.processEvents()
        window.grab().save(str(args.smoke_image))
        window.close()
        return 0
    result=app.exec_()
    return (0 if not args.select_circle or window.accepted else 2) if result==0 else result


if __name__=='__main__':
    raise SystemExit(main())
