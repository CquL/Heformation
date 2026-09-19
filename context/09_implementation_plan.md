# 当前实施路线

**更新：2026-09-19**

本文件只保存阶段路线，不复制一份巨大的第二实施方案。最新用户明确决定优先于旧 `plan.md` 中与当前状态冲突的历史 P0–P3 表述。

## 当前已经完成

### A. 三机编队与 Action

完成：

- 3 节点声明式编队图；
- 单机/组级模式复位；
- 单机→组级→再单机；
- 路由、落点、Result、释放判据实跑通过。

### B. 任务计划语义

完成并单测：

- `serial=True`；
- 物理成员预测位置；
- 无在线端点的平台不占在线串行时钟；
- `observed AND received`；
- 全区间 formation / corridor 判据。

当前总计：

```text
240 tests passed
```

## 当前下一步

### Step 1：runner 双模式

同一 runner 显式支持：

```text
fixed_coalition
executor
```

七机回归保持原路径。

三机任务线：

```text
load_request
→ expand
→ build_executor_plan(serial=True)
```

### Step 2：PlanItem 真正选择 Action endpoint

不能只把 `executor_id/action_endpoint` 记录进 evidence。

实际派发必须：

```text
selected Executor
→ selected Action client
→ real Result
```

结果超时或终态未知时继续保持资源锁定语义。

### Step 3：用户确认

目标流程：

```text
启动三机系统待命
→ 加载请求
→ 打印任务、分配和限制
→ 用户确认
→ 才开始派发
```

默认不自动运行。

### Step 4：完整请求实跑

一次运行贯通：

```text
单机巡查
→ 岸线三机编队阶段
→ observation
→ delivery
→ 必要时一次复测
→ mission result
```

### Step 5：实时展示

dashboard 只读任务层权威状态：

```text
Plan
Executor
current action
coverage
delivery
resource occupancy
failure reason
```

不自己重新推导另一套成功判据。

### Step 6：七机 M2

独立完成：

- 原盒子时间线；
- `d_ref / d_actual`；
- 局部扫描中断；
- 可追溯基线元数据；
- `e_budget` 口径。

## 后续阶段

三机任务闭环稳定后：

```text
USV backend
→ UUV backend
→ 五平台在线资源选择
→ 通信窗口 / 中继 / 缓存
→ 能量
→ 跨介质 AAV
```

不在当前三机任务线完成前同时展开这些工作。
