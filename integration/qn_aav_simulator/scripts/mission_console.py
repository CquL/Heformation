#!/usr/bin/env python3
"""Qt operator entry for the existing runner and its authoritative task state.

The runner performs expansion/planning. Confirmation writes to its existing CLI
gate; the GUI never dispatches motion goals or infers business completion.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import threading
import time

import rospy
from actionlib_msgs.msg import GoalID
from python_qt_binding.QtCore import QTimer,Qt,QPointF,QRectF
from python_qt_binding.QtGui import QFont,QFontDatabase,QPainter,QPen,QBrush,QColor
from python_qt_binding.QtWidgets import (QApplication,QFileDialog,QHBoxLayout,QLabel,
    QMessageBox,QPlainTextEdit,QPushButton,QSplitter,QVBoxLayout,QWidget)
from qn_aav_simulator.task_line import load_request
from qn_aav_simulator.monitoring_request import circle_joint_mission_mappings


class CircleMap(QWidget):
    """One direct world-coordinate circle gesture; no task state duplication."""
    def __init__(self,scene,center,radius,on_change):
        super().__init__();self.scene=scene;self.center=tuple(center);self.radius=float(radius)
        self.dragging=False;self.on_change=on_change;self.setMinimumSize(800,560)
        xs=[];ys=[]
        for item in scene.get('objects',()):
            xs.extend((item['center'][0]-item['size'][0]/2,item['center'][0]+item['size'][0]/2))
            ys.extend((item['center'][1]-item['size'][1]/2,item['center'][1]+item['size'][1]/2))
        for item in scene.get('return_sites',{}).values():xs.append(item['position'][0]);ys.append(item['position'][1])
        xs.append(scene['mother_ship_position'][0]);ys.append(scene['mother_ship_position'][1])
        self.bounds=(min(xs)-4,max(xs)+12,min(ys)-5,max(ys)+5)

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
        if event.button()==Qt.LeftButton:
            self.center=self.to_world(event.pos());self.radius=0.;self.dragging=True;self.update()

    def mouseMoveEvent(self,event):
        if self.dragging:
            point=self.to_world(event.pos());self.radius=math.hypot(point[0]-self.center[0],point[1]-self.center[1])
            self.on_change(self.center,self.radius);self.update()

    def mouseReleaseEvent(self,event):
        if self.dragging and event.button()==Qt.LeftButton:
            self.dragging=False;self.mouseMoveEvent(event);self.on_change(self.center,self.radius)

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
            cx,cy,_=item['center'];sx,sy,_=item['size'];a=self.to_screen((cx-sx/2,cy+sy/2))
            painter.setPen(QPen(QColor('#c8c9cf') if item['kind']=='SOLID' else QColor('#ee5963'),2))
            painter.setBrush(QBrush(QColor(160,165,175,170) if item['kind']=='SOLID' else QColor(220,55,70,90)))
            painter.drawRect(QRectF(a.x(),a.y(),sx*self._scale(),sy*self._scale()))
        for member,item in self.scene.get('return_sites',{}).items():
            p=self.to_screen(item['position']);painter.setBrush(QBrush(QColor('#48d597')))
            painter.setPen(Qt.NoPen);painter.drawEllipse(p,4,4)
        mother=self.to_screen(self.scene['mother_ship_position']);painter.setBrush(QBrush(QColor('#f1d38b')))
        painter.drawEllipse(mother,7,7)
        if self.radius>0:
            c=self.to_screen(self.center);r=self.radius*self._scale()
            painter.setPen(QPen(QColor('#ffd447'),3));painter.setBrush(QBrush(QColor(255,212,71,45)))
            painter.drawEllipse(c,r,r);painter.setBrush(QBrush(QColor('#ffd447')));painter.drawEllipse(c,4,4)


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
    if not args.smoke_image and not args.select_circle:rospy.init_node('mission_console',disable_signals=True)
    app=QApplication(sys.argv[:1])
    cjk=Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    if cjk.is_file():
        font_id=QFontDatabase.addApplicationFont(str(cjk))
        families=QFontDatabase.applicationFontFamilies(font_id)
        if families:app.setFont(QFont(families[0],10))
    if args.select_circle:
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
