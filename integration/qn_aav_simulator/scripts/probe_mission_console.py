#!/usr/bin/env python3
"""Exercise actual Qt clicks, the runner confirmation gate and window closure."""
import argparse
import json
from pathlib import Path
import time
import rospy
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QFont,QFontDatabase
from python_qt_binding.QtTest import QTest
from python_qt_binding.QtWidgets import QApplication
from qn_aav_simulator.msg import FormationActionGoal
from mission_console import MissionConsole


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--request',type=Path,required=True)
    args=parser.parse_args()
    rospy.init_node('probe_mission_console',disable_signals=True)
    app=QApplication([])
    font=Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    if font.exists():
        ident=QFontDatabase.addApplicationFont(str(font))
        app.setFont(QFont(QFontDatabase.applicationFontFamilies(ident)[0],10))
    goals=[]
    subscriptions=[rospy.Subscriber('/'+name+'/formation_action/goal',FormationActionGoal,
        lambda msg:goals.append(msg.goal.task_id),queue_size=20) for name in ('aav_1','aav_2','aav_3','aav_formation')]
    window=MissionConsole(args.request,args.output,serial=True)
    window.show()
    app.processEvents()
    QTest.mouseClick(window.preview_button,Qt.LeftButton)
    deadline=time.monotonic()+180
    while time.monotonic()<deadline:
        app.processEvents()
        if window.confirm_button.isEnabled():break
        time.sleep(.03)
    else:raise RuntimeError('UI preview never became confirmable')
    assert not goals,'motion dispatched before UI confirmation'
    window.grab().save(str(args.output/'ui-authoritative-preview.png'))
    print('PREVIEW READY; native goals before confirmation:',len(goals),flush=True)
    QTest.mouseClick(window.confirm_button,Qt.LeftButton)
    deadline=time.monotonic()+30
    while not goals and time.monotonic()<deadline:
        app.processEvents()
        time.sleep(.03)
    assert goals,'confirmation did not dispatch a native goal'
    window.close()
    app.processEvents()
    assert window.process.poll() is None,'closing UI terminated runner'
    print('UI CLOSED; confirmed runner remains alive',flush=True)
    deadline=time.monotonic()+300
    previous=None
    final={}
    while time.monotonic()<deadline:
        final=json.loads(rospy.get_param('/formation_mission_runner/task_state','{}'))
        phase=(final.get('status'),(final.get('current_action') or {}).get('task_id'))
        if phase!=previous:
            print('TASK',phase,flush=True)
            previous=phase
        if final.get('status') in ('PASS_GEOMETRIC_PROXY','UNKNOWN_LOCKED','FAIL','FAIL_COVERAGE','FAIL_DEADLINE'):break
        time.sleep(.2)
    checks={'no_dispatch_before_confirmation':True,'confirmed_native_dispatch':bool(goals),
            'runner_survives_ui_close':True,'request_passed':final.get('status')=='PASS_GEOMETRIC_PROXY',
            'resources_released':final.get('resource_locks')==[]}
    result={'scope':'real Qt/ROS three-AAV serial request; not five-platform UI acceptance',
            'checks':checks,'passed':all(checks.values()),'final':final,'native_goals':goals}
    (args.output/'ui-probe.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(checks),flush=True)
    return 0 if result['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
