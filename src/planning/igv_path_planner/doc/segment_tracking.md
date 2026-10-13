# 下游 segment 跟踪

`fms_path_planning_node.cpp` 是 CMake 实际编译的节点入口。选择器位于
`route_segment_selector.hpp`，由 `fms_path_planning.hpp` 引入。
VA 和消息定义不变。

## 行为

- `#p` 仅使路径缓存刷新，不推进 segment；基础 route ID 变化才清除匹配历史。
- 首次按折线投影距离和插值后的车身 heading 筛选。多个相近候选时发布零速度路径。
- 后续窗口要求当前段与历史段有至少两个不同位置、顺序一致的共同点，且方向一致。
  数组下标可随裁剪变化。只有一个共同边界点不能证明是同一段。
- 检查当前位姿在历史几何上的进度变化，限制回退和跨分支跳跃。
- 只向相邻下一段推进；必须接近当前段末端且下一段起点连续。
  同向段可在边界附近切换，异向段要求低速持续保持后才切换。
- 定时器也执行匹配，所以同一个 route ID 内可以切段。选段、几何或窗口 ID
  变化，以及从停车保护恢复，都会重建参考路径。
- 只输出当前段，并按最大 0.8 m/s² 的速度上限包络将段末速度降为零。
  该包络不是车辆制动能力或 jerk 可行性的保证；后续速度规划仍需处理运动学约束。
- 匹配失败、空段、未定位、定位/窗口超时、路径生成异常均替换缓存为零速度路径，
  防止定时器或 stop-point 回调恢复旧的行驶速度。暂停保留段历史，取消/完成清除历史。
- 换向停车计时使用里程计接收时间，只反复处理旧里程计不会满足新的停车时长。

## 参数

| 参数 | 默认值 |
|---|---:|
| `segment_tracking.endpoint_distance` | 0.5 m |
| `segment_tracking.stopped_speed` | 0.1 m/s |
| `segment_tracking.stopped_duration` | 1.0 s |
| `segment_tracking.ambiguity_margin` | 0.25（距离与角度误差之和的评分差） |
| `segment_tracking.backtrack_tolerance` | 1.0 m |
| `segment_tracking.progress_slack` | 2.0 m |

位置/朝向匹配阈值沿用 `ego_nearest_dist_threshold` / `ego_nearest_yaw_threshold`。
定位与路线超时沿用 `stale_threshold`。

## 接口限制

没有稳定段号时，完全重复的几何无法唯一恢复执行身份。历史段已经完全滑出窗口、
共同点不足、上游重新采样或初始正反向路线完全重叠时，本实现停车并报告原因，
不会跳到最近的另一段。新任务可重置跟踪；没有添加“任意指定段号”的恢复接口。

当前节点没有实际挡位反馈接口。本实现确认的是“在换向点持续停车”，随后发布
下一段及 `/planning/reverse_state`，不代表已确认底盘完成换挡。底盘换挡联锁仍需
由现有控制接口保障，不能把软件方向状态当作挡位反馈。

同向段仍只生成当前段，段末减速可能造成短暂降速；这里优先保留现有逐段生成
结构，没有跨段合并普通路径或跨段拼接 LC/LOCAL_PLAN 区间。

## 验证

`route_segment_selector_test` 覆盖 #p 更新、窗口下标变化、同向切段、重叠歧义、
换向停车保持、禁止跳到未来段、关联丢失、回退、新路线重置及窗口末端。
