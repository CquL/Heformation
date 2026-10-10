import argparse
import csv
import hashlib
import json
import math
import re
from pathlib import Path
from statistics import median

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch


REPORT_ROOT = Path(__file__).resolve().parent
REPOSITORY_ROOT = REPORT_ROOT.parents[2]
DATA_ROOT = REPORT_ROOT / 'data'
FIGURE_ROOT = REPORT_ROOT / 'figures'
RUNS = {
    'A': 'experiments/20261005-abc-16x-videos/A',
    'B': 'experiments/20261004-server715-b-complete-r75',
    'C': 'experiments/20261004-server715-c-complete-r78',
}
PALETTE = {
    'air': '#0F4D92', 'water': '#42949E', 'support': '#8BCF8B',
    'assembly': '#9A4D8E', 'return': '#CFCECE', 'generated': '#B64342',
    'received': '#0F4D92', 'text': '#272727', 'muted': '#767676',
}
MEMBERS = ['AAV1', 'AAV2', 'AAV3', 'USV', 'UUV', 'AAV编队']
PRODUCERS = {'drone_0': 'AAV1', 'drone_1': 'AAV2', 'drone_2': 'AAV3',
             'usv': 'USV', 'uuv': 'UUV'}


def member_label(executor):
    if executor == 'aav_formation':
        return 'AAV编队'
    if executor.startswith('aav_'):
        return 'AAV' + executor.split('_')[1]
    return executor.upper()


def write_csv(filename, rows):
    if not rows:
        raise ValueError('Empty data table: ' + filename)
    with (DATA_ROOT / filename).open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def phase_of(activity, target, endpoint):
    if 'formation-assembly:' in activity:
        return 'assembly'
    if 'formation-return' in activity or target.startswith('return:'):
        return 'return'
    if 'support' in target:
        return 'support'
    return 'water' if endpoint.endswith('/platform_task') else 'air'


def refresh_data():
    summaries, steps, receipts, works, support = [], [], [], [], []
    geometries, source_files = {}, []
    for business, folder in RUNS.items():
        metrics_path = REPOSITORY_ROOT / folder / 'metrics.json'
        raw_bytes = metrics_path.read_bytes()
        metrics = json.loads(raw_bytes)
        source_files.append({'path': str(metrics_path.relative_to(REPOSITORY_ROOT)),
                             'sha256': hashlib.sha256(raw_bytes).hexdigest()})
        plan_items = {item['execution_id']: item for item in metrics['plan']['items']}
        origins = [execution['result_received_at'] - plan_items[execution['execution_id']]['actual_finish']
                   for execution in metrics['executions']
                   if execution['execution_id'] in plan_items
                   and plan_items[execution['execution_id']].get('actual_finish') is not None]
        origin = median(origins)
        origin_spread = max(origins) - min(origins)
        if origin_spread > 1e-5:
            raise ValueError('Inconsistent model-time origin: ' + business)
        business_steps = []
        for result in metrics['step_results']:
            if result['native_state'] != 3 or not result.get('verified'):
                raise ValueError('Unverified terminal in selected completed run')
            item = plan_items[result['activity_id']]
            step_number = int(result['execution_id'].rsplit(':step:', 1)[1]) if ':step:' in result['execution_id'] else 0
            definition = item['execution_steps'][step_number]
            goal_match = re.search(r'-(\d+\.\d+)$', result['goal_id'])
            if goal_match is None:
                raise ValueError('Goal timestamp absent')
            dispatch = float(goal_match.group(1)) - origin
            terminal = result['result_received_at'] - origin
            if terminal < dispatch or dispatch < -0.01:
                raise ValueError('Invalid Goal event interval')
            row = {'business': business, 'request_id': metrics['request_id'],
                   'activity_id': result['activity_id'], 'execution_id': result['execution_id'],
                   'goal_id': result['goal_id'], 'member': member_label(item['executor_id']),
                   'endpoint': result['endpoint'], 'target_ref': definition['target_ref'],
                   'phase': phase_of(result['activity_id'], definition['target_ref'], result['endpoint']),
                   'dispatch_s': dispatch, 'verified_terminal_s': terminal,
                   'native_state': result['native_state'], 'verified': True}
            business_steps.append(row)
        steps.extend(business_steps)
        business_receipts = []
        for product in metrics['received_products'].values():
            generated = product['generated_at'] - origin
            received = product['received_at'] - origin
            if product['request_id'] != metrics['request_id'] or received < generated:
                raise ValueError('Report identity or event order mismatch')
            row = {'business': business, 'request_id': metrics['request_id'],
                   'product_id': product['product_id'], 'work_id': product.get('work_id', ''),
                   'work_version': product.get('work_version', ''), 'goal_id': product['goal_id'],
                   'member': PRODUCERS[product['producer']], 'generated_s': generated,
                   'received_s': received, 'task_event_wait_s': received - generated,
                   'event_type': product.get('event_type', 'MAPPING_PROXY'),
                   'required_bytes': product.get('required_bytes', '')}
            business_receipts.append(row)
        receipts.extend(business_receipts)
        source_work = metrics.get('inspection_work') or []
        geometries[business] = source_work
        for work in source_work:
            product = next(product for product in metrics['received_products'].values()
                           if product.get('work_id') == work['work_id'])
            result = product['result']
            moving_sections = [section for section in work['sections'] if section['kind'] != 'LOCAL']
            declared_length = sum(math.dist(first, second) for section in moving_sections
                                  for first, second in zip(section['points'], section['points'][1:]))
            declared_hold = sum(section['duration_s'] for section in work['sections'] if section['kind'] == 'LOCAL')
            required_count = sum(len(section['points']) - 1 if section['kind'] != 'LOCAL' else 1
                                 for section in work['sections'])
            intervals = result['qualified_intervals']
            if result['fraction'] != 1.0 or len(intervals) != required_count:
                raise ValueError('Incomplete selected facility work')
            works.append({'business': business, 'request_id': metrics['request_id'],
                          'work_id': work['work_id'], 'work_version': work['version'],
                          'object_id': work['object_id'], 'domain': work['domain'],
                          'member': PRODUCERS[product['producer']], 'required_segments': required_count,
                          'qualified_segment_keys': len(intervals), 'fraction': result['fraction'],
                          'declared_movement_length_m': declared_length,
                          'qualified_movement_length_m': result['measured_length_m'],
                          'declared_local_hold_s': declared_hold,
                          'qualified_local_hold_s': result['qualified_hold_s']})
        summary = {'business': business, 'request_id': metrics['request_id'],
                   'planning_wall_s': metrics['planning_wall_s'], 'planning_budget_wall_s': metrics['planning_budget_s'],
                   'report_count': len(business_receipts), 'verified_step_count': len(business_steps),
                   'model_epoch_ros_s': origin, 'epoch_consistency_spread_s': origin_spread,
                   'first_goal_dispatch_s': min(row['dispatch_s'] for row in business_steps),
                   'last_required_report_received_s': max(row['received_s'] for row in business_receipts),
                   'return_authorized_s': metrics['return_authorization']['at_ros_s'] - origin,
                   'last_verified_terminal_s': max(row['verified_terminal_s'] for row in business_steps),
                   'final_resource_locks': len(metrics['resource_locks']),
                   'final_status': metrics['status'], 'final_phase': metrics['session_phase'],
                   'audit_member_proxy_clearance_m': '', 'audit_scene_proxy_clearance_m': '',
                   'audit_strict_time_alignment': '', 'audit_missing_position_samples': ''}
        summary['authorized_return_to_last_terminal_s'] = summary['last_verified_terminal_s'] - summary['return_authorized_s']
        if business in ['B', 'C']:
            audit_name = 'b-audit-summary.json' if business == 'B' else 'c-full-audit-summary.json'
            audit_path = REPOSITORY_ROOT / folder / audit_name
            audit_bytes = audit_path.read_bytes()
            audit = json.loads(audit_bytes)
            source_files.append({'path': str(audit_path.relative_to(REPOSITORY_ROOT)),
                                 'sha256': hashlib.sha256(audit_bytes).hexdigest()})
            if not audit['passed']:
                raise ValueError('Selected audit did not pass')
            summary.update(audit_member_proxy_clearance_m=audit['fleet_proxy_minimum_clearance_m'],
                           audit_scene_proxy_clearance_m=audit['minimum_scene_net_clearance_m'][1],
                           audit_strict_time_alignment=audit.get('strict_task_window_alignment', audit.get('strict_alignment_passed')),
                           audit_missing_position_samples=audit['missing_position_samples'])
        summaries.append(summary)
        if business == 'B':
            for history in metrics['plan_history']:
                for item in history['plan']['items']:
                    if '::support:' in item['execution_id']:
                        matching_steps = [row for row in business_steps if row['activity_id'] == item['execution_id']]
                        support.append({'revision': history['revision'], 'activity_id': item['execution_id'],
                                        'planned_start_s': item['planned_start'], 'planned_finish_s': item['planned_finish'],
                                        'actual_finish_s': item.get('actual_finish'),
                                        'first_goal_dispatch_s': min(row['dispatch_s'] for row in matching_steps)})
    for filename, rows in [('mission_summary.csv', summaries), ('goal_events.csv', steps),
                           ('report_events.csv', receipts), ('facility_work.csv', works),
                           ('b_support_revisions.csv', support)]:
        write_csv(filename, rows)
    (DATA_ROOT / 'work_geometry.json').write_text(json.dumps(geometries, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    scene_path = REPOSITORY_ROOT / 'docs/images/three-business-scene-overview.png'
    source_files.append({'path': str(scene_path.relative_to(REPOSITORY_ROOT)),
                         'sha256': hashlib.sha256(scene_path.read_bytes()).hexdigest()})
    manifest = {'scope': 'Three separate completed model-scale control runs; one run per business, not repeated-trial statistics.',
                'time_origin': 'Median(result_received_at - matching plan.actual_finish); relative simulation seconds.',
                'goal_start': 'Timestamp encoded in ROS GoalID; dispatch event, not physical motion onset; about 1 ms resolution.',
                'report_wait': 'Task-level software event wait; not measured acoustic/RF transmission delay.',
                'clearance': 'B/C sampled declared collision proxies, not continuous equipment certification; A has no comparable full audit here.',
                'style_reference': 'https://github.com/ChenLiu-1996/figures4papers',
                'style_commit': '0ba898c8024ed4ce5bd06c8e5500d189fcd7a160',
                'diagram_style_reference': 'https://github.com/Sleepy-Avacado/FigGenie-paper-diagram-skill',
                'diagram_style_commit': '82be26d6f2954c8bfc4089d70d8dcf82fe395864',
                'source_files': source_files}
    (DATA_ROOT / 'provenance.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def read_rows(filename):
    with (DATA_ROOT / filename).open(encoding='utf-8', newline='') as stream:
        return list(csv.DictReader(stream))


def apply_publication_style():
    plt.rcParams.update({'font.family': ['Noto Sans CJK SC', 'DejaVu Sans'],
                         'font.size': 15, 'axes.titlesize': 16, 'axes.labelsize': 15,
                         'axes.spines.top': False, 'axes.spines.right': False,
                         'axes.linewidth': 1, 'legend.frameon': False,
                         'svg.fonttype': 'none', 'pdf.fonttype': 42,
                         'axes.unicode_minus': False, 'savefig.facecolor': 'white'})


def finalize_figure(figure, name):
    for extension in ['png', 'pdf', 'svg']:
        figure.savefig(FIGURE_ROOT / (name + '.' + extension), dpi=300,
                       bbox_inches='tight', pad_inches=0.08)
    plt.close(figure)


def geometry_figure():
    geometry = json.loads((DATA_ROOT / 'work_geometry.json').read_text(encoding='utf-8'))
    figure, axes = plt.subplots(2, 2, figsize=(10, 9))
    panels = [(axes[0, 0], geometry['B'], (0, 1), '(a) B：两台风机的水平作业意图'),
              (axes[0, 1], geometry['B'], (0, 2), '(b) B：水上高度与基础深度'),
              (axes[1, 0], [work for work in geometry['C'] if work['object_id'] == 'production_platform'], (0, 1), '(c) C：平台外侧与水下结构'),
              (axes[1, 1], [work for work in geometry['C'] if work['object_id'] == 'seabed_pipeline'], (0, 1), '(d) C：管路偏置线与局部作业')]
    for axis, works, coordinates, title in panels:
        for work in works:
            color = PALETTE['air'] if work['domain'] == 'AIR' else PALETTE['water']
            for section in work['sections']:
                horizontal = [point[coordinates[0]] for point in section['points']]
                vertical = [point[coordinates[1]] for point in section['points']]
                if section['kind'] == 'LOCAL':
                    axis.scatter(horizontal, vertical, marker='D', s=40, color=PALETTE['generated'], zorder=5)
                else:
                    axis.plot(horizontal, vertical, color=color, linewidth=1.4, linestyle='--', alpha=0.80)
        axis.set_title(title, loc='left', pad=9, fontsize=14)
        axis.set_xlabel('x / m')
        axis.set_ylabel(('y' if coordinates[1] == 1 else 'z') + ' / m')
        axis.set_aspect('equal', adjustable='datalim')
        axis.margins(0.16)
        if coordinates[1] == 2:
            axis.axhline(0, color=PALETTE['muted'], linewidth=0.8)
            axis.annotate('海面 z＝0', xy=(0.97, 0), xycoords=('axes fraction', 'data'),
                          xytext=(0, 4), textcoords='offset points', ha='right', fontsize=12, color=PALETTE['muted'])
    axes[1, 0].text(0.02, 0.02, 'AIR：3.9／4.4／4.9 m\nWATER：−1.2／−3.0／−4.8 m',
                    transform=axes[1, 0].transAxes, fontsize=12, va='bottom',
                    bbox={'facecolor': 'white', 'edgecolor': 'none', 'alpha': 0.92})
    axes[1, 1].text(0.02, 0.03, '水下深度 −4.5 m\n3 段沿线＋3 处 LOCAL，各 4 s', transform=axes[1, 1].transAxes, fontsize=12)
    handles = [Line2D([], [], color=PALETTE['air'], linestyle='--', label='AIR 作业'),
               Line2D([], [], color=PALETTE['water'], linestyle='--', label='WATER 作业'),
               Line2D([], [], color=PALETTE['generated'], marker='D', linestyle='none', label='局部驻留')]
    figure.legend(handles=handles, loc='upper center', ncol=3, bbox_to_anchor=(0.52, 1.0), fontsize=14)
    figure.tight_layout(rect=(0, 0.03, 1, 0.95), pad=1.3)
    figure.text(0.1, 0.006, '按已接纳的作业定义绘制；多层水平投影重合。虚线不是实际轨迹，不含进场、连接与避障。', fontsize=12, color=PALETTE['muted'])
    finalize_figure(figure, 'fig02_work_geometry')


def timeline_figure():
    events = read_rows('goal_events.csv')
    products = read_rows('report_events.csv')
    summary = {row['business']: row for row in read_rows('mission_summary.csv')}
    figure, axes = plt.subplots(3, 1, figsize=(10, 11.2))
    for axis, business, title in zip(axes, RUNS, ['(a) A：区域调查', '(b) B：风机巡检', '(c) C：平台及管路巡检']):
        for row in events:
            if row['business'] != business:
                continue
            lane = MEMBERS.index(row['member'])
            start, end = float(row['dispatch_s']), float(row['verified_terminal_s'])
            axis.broken_barh([(start, end - start)], (lane - 0.26, 0.52),
                             facecolors=PALETTE[row['phase']], edgecolors='white', linewidth=0.45,
                             hatch='//' if row['phase'] == 'return' else None, zorder=2)
        for row in products:
            if row['business'] == business:
                lane = MEMBERS.index(row['member'])
                axis.scatter(float(row['generated_s']), lane - 0.31, marker='^', s=30,
                             color=PALETTE['generated'], zorder=5)
                axis.scatter(float(row['received_s']), lane + 0.31, marker='o', s=22,
                             facecolors='white', edgecolors=PALETTE['received'], linewidths=1.1, zorder=5)
        authorization = float(summary[business]['return_authorized_s'])
        axis.axvline(authorization, color=PALETTE['muted'], linestyle=':', linewidth=1.2)
        axis.set_title(title, loc='left', pad=8)
        axis.text(0.99, 1.045, summary[business]['report_count'] + ' 份实收 · ' + summary[business]['verified_step_count'] + ' 个核实终态',
                  transform=axis.transAxes, ha='right', fontsize=13, color=PALETTE['muted'])
        maximum = float(summary[business]['last_verified_terminal_s'])
        axis.set(xlim=(0, maximum * 1.035), ylim=(5.65, -0.65), yticks=range(len(MEMBERS)), yticklabels=MEMBERS,
                 xlabel='相对仿真时间 / s')
        axis.tick_params(axis='y', length=0)
        axis.grid(axis='x', color='#E8E8E8', linewidth=0.6)
        axis.set_axisbelow(True)
        if business == 'C':
            axis.text(maximum * 0.50, 4, 'UUV 待命，无执行 Goal', color=PALETTE['muted'], fontsize=13, ha='center', va='center')
    handles = [Patch(facecolor=PALETTE[key], label=label, hatch='//' if key == 'return' else None)
               for key, label in [('air', 'AIR 作业／接近'), ('water', 'WATER／跨介质'), ('support', 'USV 支援'),
                                  ('assembly', 'AAV 集结'), ('return', '返回')]]
    handles.extend([Line2D([], [], color=PALETTE['generated'], marker='^', linestyle='none', label='报告产生'),
                    Line2D([], [], color=PALETTE['received'], marker='o', markerfacecolor='white', linestyle='none', label='母船匹配实收'),
                    Line2D([], [], color=PALETTE['muted'], linestyle=':', label='人工返回授权')])
    figure.legend(handles=handles, loc='upper center', ncol=4, bbox_to_anchor=(0.52, 1.0), fontsize=13)
    figure.tight_layout(rect=(0, 0.035, 1, 0.93), h_pad=2, pad=1.0)
    figure.text(0.08, 0.010, '条带＝Goal 派发至终态核实，含准备与等待；三幅为独立任务，时间尺度分别设置。', fontsize=12, color=PALETTE['muted'])
    finalize_figure(figure, 'fig04_actual_timeline')


def support_figure():
    revisions = read_rows('b_support_revisions.csv')
    originals = [row for row in revisions if row['revision'] == revisions[0]['revision']]
    original_second = next(row for row in originals if row['activity_id'].endswith('support:1'))
    final_first = next(row for row in reversed(revisions) if row['activity_id'].endswith('support:0') and row['actual_finish_s'])
    final_second = next(row for row in reversed(revisions) if row['activity_id'].endswith('support:1') and row['actual_finish_s'])
    planned_release = float(original_second['planned_start_s'])
    actual_release = float(final_first['actual_finish_s'])
    second_dispatch = float(final_second['first_goal_dispatch_s'])
    figure, axes = plt.subplots(2, 1, figsize=(9.6, 5.8), gridspec_kw={'height_ratios': [1.7, 1]})
    axis = axes[0]
    for row in originals:
        start, finish = float(row['planned_start_s']), float(row['planned_finish_s'])
        axis.broken_barh([(start, finish - start)], (-0.22, 0.44), facecolors='#DDF3DE', edgecolors=PALETTE['water'], linewidth=1.2)
    for row in [final_first, final_second]:
        start, finish = float(row['first_goal_dispatch_s']), float(row['actual_finish_s'])
        axis.broken_barh([(start, finish - start)], (0.78, 0.44), facecolors=PALETTE['support'], edgecolors='white', linewidth=0.7)
    axis.set(yticks=[0, 1], yticklabels=['原预计安排', '实际执行窗口'], ylim=(1.5, -0.6), xlim=(-10, 1330), xlabel='相对仿真时间 / s')
    axis.axvline(planned_release, linestyle='--', color=PALETTE['muted'], linewidth=1)
    axis.axvline(actual_release, linestyle=':', color=PALETTE['generated'], linewidth=1.5)
    axis.annotate(f'原预计后继开始\n{planned_release:.3f} s', xy=(planned_release, 0), xytext=(540, -0.30),
                  ha='left', va='center', fontsize=14, arrowprops={'arrowstyle': '-', 'color': PALETTE['muted']})
    axis.text(460, 1.0, '服务 0：进场与规定交付', ha='center', va='center', fontsize=14)
    axis.text(1190, 1.0, '服务 1', ha='center', va='center', fontsize=14)
    axis.set_title('(a) 预计时序不作为实际放行时刻', loc='left', pad=10)
    axis.grid(axis='x', color='#E8E8E8', linewidth=0.6)
    axis.set_axisbelow(True)
    detail = axes[1]
    detail.set_axis_off()
    detail.text(0.0, 0.94, '(b) 实际完成 → 更新后继 → 实际派发', fontsize=16, weight='bold', transform=detail.transAxes)
    detail.text(0.0, 0.54, f'服务 0 终态核实：{actual_release:.3f} s', transform=detail.transAxes, color=PALETTE['generated'], fontsize=15)
    detail.text(0.0, 0.24, f'服务 1 更新开始：{float(final_second["planned_start_s"]):.3f} s；Goal 派发：{second_dispatch:.3f} s',
                transform=detail.transAxes, fontsize=14)
    figure.tight_layout(rect=(0, 0.04, 1, 1), pad=1.3)
    figure.text(0.09, 0.012, 'B r75 同请求；来自 Plan 历史和匹配 Goal／终态，不是物理链路传输测量。', fontsize=12, color=PALETTE['muted'])
    finalize_figure(figure, 'fig05_support_feedback')


def main():
    parser = argparse.ArgumentParser(description='Extract recorded A/B/C control-run data and render report figures.')
    parser.add_argument('--refresh-data', action='store_true', help='Read the original local completed experiment records.')
    arguments = parser.parse_args()
    DATA_ROOT.mkdir(exist_ok=True)
    FIGURE_ROOT.mkdir(exist_ok=True)
    if arguments.refresh_data:
        refresh_data()
    apply_publication_style()
    geometry_figure()
    timeline_figure()
    support_figure()
    print('Report figures rendered; original simulation records unchanged.')


if __name__ == '__main__':
    main()
