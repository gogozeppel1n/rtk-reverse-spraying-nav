# VERSION_HISTORY：版本演化与工程原因

本文档用于说明本仓库各版本的前因后果：每个功能包做什么、为什么要改、改完解决了什么、还留下什么问题。

## 总体演化主线

项目最初目标是：基于 NovAtel RTK/INS 位姿，让 Ranger/AGV 类底盘在开放场地中完成两点直线倒车或轨迹复现。后来加入甲方喷洒工况：每隔 0.5 m 或 0.3 m 停车一次，停车后上方丝杠/传送带横向运动进行喷洒。

演化过程中发现，原始连续倒车控制本身不是最大问题。真正的问题来自两个方面：

1. **停车状态机破坏连续控制**：频繁停车、起步、继电器动作会打断连续小修正，导致恢复导航瞬间大幅打轮。
2. **上方长横杆/丝杠机械耦合**：横杆展开、电机上电后，底盘轻微纠偏会激发机械反扭矩和偏载，表现为车轮角度变大、车头晃、甚至出现 S 形。

因此后续版本不是简单提高 P 或增加横差增益，而是逐步走向：

```text
连续控制基线
→ 分段停车/继电器
→ 稳定停车状态机
→ 影子停车和低速角速度保护
→ 长横杆随机偏载适配
→ 停车角度保持与角速度变化率限制
→ 喷洒固定作业航向
→ cte_keep：停车前保留部分横差修正
```

---

## 01_rtk_reverse_nav_save_dir_baseline

### 版本定位

基础稳定版，也叫“添加保存目录版”。这是后续所有版本的母版。

### 做了什么

- 支持 RTK/INS 位姿桥接。
- 支持两点模式：记录终点、人工开到起点、再沿两点确定直线倒车。
- 支持多点轨迹记录与反向复现。
- 增加路径保存目录、路径前缀、导入导出等功能。
- 核心控制保持连续输出：

```text
linear_x = -reverse_speed
angular_z = k_heading * heading_error - k_cross_track * cross_track_error
```

### 为什么重要

它证明了“连续倒车跟线”本身可用。后面加入停车、继电器和喷洒机构后出现的问题，不能直接归咎于 RTK 控制算法，而要看停车状态机和机械耦合。

### 工程效果

在没有频繁停车和上方机构强扰动时，两点直线倒车可以比较稳定，轮胎动作小，长距离横差可控。

---

## 02_rtk_reverse_nav_pause_relay_initial_problematic

### 版本定位

初始“停车 + 继电器”版本，也是问题暴露版。

### 做了什么

- 增加 `enable_segment_pause`。
- 每隔 `pause_distance_m` 停车。
- 停车保持 `pause_duration_s`。
- 停车期间触发 USB 继电器闭合，用于喷洒动作。

### 为什么后来要改

这个版本暴露出：原始控制连续性被停车状态机破坏。典型问题包括：

- 停车后恢复导航时大幅打轮；
- 30 cm 一停时，车还没稳定又进入下一次停车；
- 停车、起步过程占用了过多段距，横差没有连续距离收敛；
- 停车保持阶段若 `linear=0` 但 `angular!=0`，角速度型底盘可能原地自转或原地打轮。

### 工程效果

不建议继续作为上车版本。它的价值是保留“错误样本”，解释为什么简单叠加停车继电器会让原来稳定的倒车控制变差。

---

## 03_rtk_reverse_nav_stable_pause_relay

### 版本定位

稳定停车继电器版。它开始把“停车/继电器”看成外层状态机，而不是重启导航或重置控制。

### 做了什么

- 继续使用基础版的两点/多点控制逻辑。
- 把分段停车作为外层管理。
- 停车点到达后进入减速、保持、起步阶段。
- 支持继电器参数：`relay_port`、`relay_pulse_seconds`、`relay_on_hex`、`relay_off_hex`。

### 为什么要改

为了解决 02 里“停车逻辑破坏连续导航”的问题。核心原则变成：停车不重置轨迹、不重置段号、不重新规划，只是临时中断输出。

### 工程效果

相比 02 更稳定，但当停车距离设置成 0.30 m 且起停时间较长时，仍然可能出现连续巡航距离不足的问题。

---

## 04_rtk_reverse_nav_shadow_pause_relay

### 版本定位

影子停车 + 低线速度角速度保护版。

### 做了什么

- 新增 `pause_shadow_steering`：进入停车前保存最后一次稳定小角速度修正。
- 新增 `zero_angular_below_min_linear` 和 `min_linear_for_angular`：线速度过低时强制角速度为 0，避免低速或刚起步阶段原地转。
- 停车保持阶段真实输出仍然必须是：

```text
linear.x = 0
angular.z = 0
```

- 起步时从保存的影子角速度平滑过渡到实时计算角速度。

### 为什么要改

因为 Ranger/AGV 类底盘在 `linear.x = 0` 且 `angular.z != 0` 时可能原地自转。影子模式不是让停车时继续输出角速度，而是“内部记住趋势，真实输出仍为 0”。

### 工程效果

适合解决“停车恢复导航瞬间打轮/原地自转”的问题。但它主要处理停车恢复，不直接解决上方长横杆在运动过程中产生的机械反扭矩。

---

## 05_rtk_reverse_nav_shadow_payload_optimized

### 版本定位

面向长横杆、随机偏载和终点精修的影子优化版。

### 做了什么

- 延长 RTK 超时参数：`position_timeout`、`heading_timeout`、`pose_timeout`，减少现场短时数据卡顿造成的误停。
- 降低速度和角速度上限，减小长横杆激励。
- 增加终点精修：`final_slowdown_distance`、`final_reverse_speed`、`final_cross_track_tolerance`。
- 终点前跳过分段停车，给车辆连续低速修横差。
- 不做固定 left/right 补偿，因为横杆初始位置随机，停车后还会左右切换。

### 为什么要改

现场发现上方丝杠/横杆展开后，车轮纠偏明显变大。固定偏载补偿可能猜错方向，所以更可靠的是降低控制激励、限制角速度、延长连续修正距离。

### 工程效果

对“随机偏载 + 停车恢复”更友好，但控制主线仍偏影子停车，不是最终喷洒固定航向方案。

---

## 06_rtk_reverse_nav_random_payload_shadow_final

### 版本定位

随机偏载影子最终版，与 05 基本同源，是 05 思路的整理版本。

### 做了什么

- 保留影子停车；
- 保留低速角速度保护；
- 保留终点精修；
- 保留“不做固定 left/right 补偿”的策略。

### 工程效果

适合解释“在随机左右负载下不要猜补偿方向”的工程结论。后续主线转向喷洒固定作业航向后，该版本不再作为当前首选。

---

## 07_rtk_reverse_nav_pause_yaw_hold_slew

### 版本定位

停车角度保持 + 角速度变化率限制探索版。本目录中包含两个子版本：

```text
two_point_resume_variant/
yaw_hold_variant/
```

### 做了什么

- 加入全局角速度变化率限制：`enable_global_angular_slew`、`max_angular_accel`。
- 加入速度相关角速度限制：`enable_speed_based_angular_limit`、`min_turn_radius_m`。
- 加入停车角度保持相关参数：`hold_yaw_during_pause`、`pause_hold_yaw_average_s`、`resume_straight_time_s`、`resume_cross_track_ramp_s`。
- 降低 `k_cross_track`，减少横差直接抢方向。

### 为什么要改

为了满足喷洒时“每次停车车头要平行”的要求：不希望车每停一次都为了横差大角度摆头。

### 工程效果

它开始从“追横差”转向“保持作业航向”。但这一版仍是中间探索版，最终被 09/10 的喷洒直线控制取代。

---

## 08_rtk_reverse_nav_yaml_only_tuning

### 版本定位

YAML-only 调参思路，不单独放源码。

### 做了什么

- 思路是把现场可调参数集中到 `config/nav_params.yaml`。
- `reverse_nav.launch` 只负责加载 YAML，不再覆盖关键控制参数。

### 为什么重要

现场调试时不能每次改 Python 或 launch。YAML-only 的思想已经合并进 09 和 10。

---

## 09_rtk_reverse_nav_spray_heading_stable

### 版本定位

喷洒固定作业航向稳定版。

### 做了什么

- 主用 `tracking_mode: line`。
- 新增 `enable_spray_line_control`。
- 两点模式先确定固定作业航向。
- 横差不再直接进入 `angular_z`，而是转成小航向偏置：

```text
cte_yaw_correction = atan2(-cross_track_error, cross_track_lookahead_m)
```

- 通过 `max_cross_track_yaw_correction_deg` 限制横差最多让车头偏几度。
- 通过 `pre_stop_align_distance_m` 在接近停车点时逐渐退出横差修正，让停车时车头更平行。

### 为什么要改

现场核心要求不是“每 50 cm 横差立刻归零”，而是“每次停车喷洒时车头方向一致，不能 S 形”。因此把横差变成小航向偏置，比直接加大 `k_cross_track` 更适合喷洒场景。

### 工程效果

用户反馈：没有出现大幅 S 形乱动，车辆明显更稳；但横差修正力度不够，约 50 m 可能偏离 10 cm 左右。进一步分析发现：横差刚被修小，停车前航向完全回正后横差又变大，导致“修横差像做无用功”。

---

## 10_rtk_reverse_nav_spray_cte_keep_final

### 版本定位

当前推荐主用版本：喷洒直线 cte_keep 优化版。

### 做了什么

在 09 的基础上新增：

```yaml
pre_stop_cte_keep_ratio: 0.35
```

这意味着停车前不再把横差修正完全退到 0，而是保留 35% 的横差小航向修正。这样既保持停车车头基本平行，又避免航向回正把刚修回的横差重新放大。

同时调整默认参数：

```yaml
k_heading: 1.00 -> 0.80
cross_track_lookahead_m: 3.0 -> 2.0
max_cross_track_yaw_correction_deg: 2.0 -> 3.0
pre_stop_align_distance_m: 0.20 -> 0.15
max_angular_speed: 0.06 -> 0.08
max_angular_accel: 0.03 -> 0.04
```

### 为什么要改

用户现场反馈的矛盾是：

```text
横差在修，但航差一回正，横差又变大。
```

所以 10 号版的核心不是暴力增大横差 P，而是在停车前保留一部分横差修正趋势，让车辆在保证喷洒平行性的同时，横差不要被回正动作抵消。

### 工程预期效果

- 比 09 横差修正更有力；
- 比直接加大 `k_cross_track` 更不容易 S 形；
- 停车时车头仍保持基本平行；
- 适合 0.50 m 停车喷洒作为默认测试；
- 若甲方要求 0.30 m 停车，应按调参指南缩短起停时间。

---

## 当前推荐结论

当前项目应以 10 号版本作为主线：

```text
packages/10_rtk_reverse_nav_spray_cte_keep_final/rtk_reverse_nav
```

后续不要再从零重写控制逻辑，也不要回到大 `k_cross_track` 抢方向。优化方向应限制在：

- `cross_track_lookahead_m`
- `max_cross_track_yaw_correction_deg`
- `pre_stop_align_distance_m`
- `pre_stop_cte_keep_ratio`
- `max_angular_speed`
- `max_angular_accel`
- `pause_distance_m`
- `pause_smooth_stop_s`
- `pause_smooth_start_s`
