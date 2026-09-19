#!/usr/bin/env python3
"""Qt operator entry for the existing runner and its authoritative task state.

The runner performs expansion/planning. Confirmation writes to its existing CLI
gate; the GUI never dispatches motion goals or infers business completion.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import threading
import time

import rospy
from actionlib_msgs.msg import GoalID
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtGui import QFont,QFontDatabase
from python_qt_binding.QtWidgets import (QApplication,QFileDialog,QHBoxLayout,QLabel,
    QPlainTextEdit,QPushButton,QSplitter,QVBoxLayout,QWidget)
from qn_aav_simulator.task_line import load_request


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
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--serial',choices=['true','false'],default='true')
    p.add_argument('--smoke-image',type=Path)
    args=p.parse_args()
    if not args.smoke_image:rospy.init_node('mission_console',disable_signals=True)
    app=QApplication(sys.argv[:1])
    cjk=Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    if cjk.is_file():
        font_id=QFontDatabase.addApplicationFont(str(cjk))
        families=QFontDatabase.applicationFontFamilies(font_id)
        if families:app.setFont(QFont(families[0],10))
    window=MissionConsole(args.request,args.output,args.serial=='true',bool(args.smoke_image))
    window.show()
    if args.smoke_image:
        app.processEvents()
        window.grab().save(str(args.smoke_image))
        window.close()
        return 0
    return app.exec_()


if __name__=='__main__':
    raise SystemExit(main())
