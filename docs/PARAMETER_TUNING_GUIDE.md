# PARAMETER_TUNING_GUIDE：现场调参指南

本文档针对当前主用版本：

```text
packages/10_rtk_reverse_nav_spray_cte_keep_final/rtk_reverse_nav
```

现场调参只改：

```text
rtk_reverse_nav/config/nav_params.yaml
```

不要现场直接改 `reverse_nav.launch` 或主控 Python 文件。

---

## 1. 当前推荐起始参数

```yaml
tracking_mode: line
enable_spray_line_control: true
spray_line_yaw_source: geometry_reverse

reverse_speed: 0.10
k_heading: 0.80
cross_track_lookahead_m: 2.0
max_cross_track_yaw_correction_deg: 3.0
pre_stop_align_distance_m: 0.15
pre_stop_cte_keep_ratio: 0.35
heading_deadband_deg: 0.4

max_angular_speed: 0.08
enable_global_angular_slew: true
max_angular_accel: 0.04

pause_distance_m: 0.50
pause_duration_s: 3.0
pause_smooth_stop_s: 0.20
pause_smooth_start_s: 0.30
pause_skip_near_goal_m: 1.00
```

这组参数的目标不是最快把横差打到 0，而是在长横杆喷洒工况下实现：

```text
车头停车基本平行
横差缓慢收敛
车轮动作不过大
上方装置不被激发晃动
```

---

## 2. 喷洒直线控制的核心参数

### 2.1 `k_heading`

```yaml
k_heading: 0.80
```

含义：航向误差到角速度的比例。

- 太大：车头很快回正，但可能把横差修正抵消，停车时车轮动作明显。
- 太小：车头回正慢，可能整体斜着走。

当前建议：`0.75 ~ 0.90`。

### 2.2 `cross_track_lookahead_m`

```yaml
cross_track_lookahead_m: 2.0
```

含义：把横向误差转换成航向偏置的“虚拟前视距离”。计算逻辑近似为：

```text
cte_yaw_correction = atan2(-cross_track_error, cross_track_lookahead_m)
```

- 数值越大：修横差越柔，不容易 S 形，但横差收敛慢。
- 数值越小：修横差更有力，但车头偏角更明显。

现场调法：

```yaml
横差修不回来：2.0 -> 1.8
仍修不回来：1.8 -> 1.6
车头开始明显摆：退回 2.0
```

### 2.3 `max_cross_track_yaw_correction_deg`

```yaml
max_cross_track_yaw_correction_deg: 3.0
```

含义：横差最多允许把车头偏离固定作业航向几度。

- 越大：横差修正更有力。
- 越小：停车朝向更统一，但横差收敛慢。

现场调法：

```yaml
横差修不回来：3.0 -> 3.5
车头偏角太明显：3.0 -> 2.5
喷洒均匀性优先：2.0 ~ 3.0
横差控制优先：3.0 ~ 3.5
```

### 2.4 `pre_stop_align_distance_m`

```yaml
pre_stop_align_distance_m: 0.15
```

含义：距离下一个停车点最后多少米开始减小横差航向偏置，让停车时车头回到固定作业航向附近。

- 越大：更早回正，停车车头更平行，但横差可能又变大。
- 越小：更晚回正，横差保留更好，但停车朝向可能稍有偏差。

现场调法：

```yaml
停车时车头不够平行：0.15 -> 0.20
横差一回正就变大：0.20 -> 0.15 或 0.10
```

### 2.5 `pre_stop_cte_keep_ratio`

```yaml
pre_stop_cte_keep_ratio: 0.35
```

含义：停车前横差修正保留比例。

- `0.00`：停车前完全退到固定作业航向，车头最平行，但横差可能重新变大。
- `0.35`：保留 35% 横差小航向偏置，兼顾平行和横差收敛。
- `0.50`：横差修正更强，但停车车头可能不够一致。

现场调法：

```yaml
停车时车头不够平行：0.35 -> 0.20
横差修了又变大：0.35 -> 0.45
车头明显斜着喷：0.35 -> 0.25
```

---

## 3. 角速度限制参数

### 3.1 `max_angular_speed`

```yaml
max_angular_speed: 0.08
```

含义：最大角速度输出上限。它不是车轮转角，但会直接影响底盘转向动作幅度。

现场调法：

```yaml
轮子最大角度仍大：0.08 -> 0.06
横差修不回来且轮子动作可接受：0.08 -> 0.10
```

### 3.2 `max_angular_accel`

```yaml
max_angular_accel: 0.04
```

含义：角速度变化率限制，用来避免轮子突然左右快速动作。

现场调法：

```yaml
轮子变化太快：0.04 -> 0.03
车太钝、修正跟不上：0.04 -> 0.05
```

### 3.3 `heading_deadband_deg`

```yaml
heading_deadband_deg: 0.4
```

含义：航差死区，用于减少车轮细碎摆动。

现场调法：

```yaml
小幅抖动多：0.4 -> 0.6
航向响应迟钝：0.4 -> 0.2
```

---

## 4. 停车喷洒参数

### 4.1 默认 0.50 m 停一次

```yaml
pause_distance_m: 0.50
pause_duration_s: 3.0
pause_smooth_stop_s: 0.20
pause_smooth_start_s: 0.30
pause_skip_near_goal_m: 1.00
```

这组适合稳定演示。0.50 m 能给车辆留出相对足够的连续修正距离。

### 4.2 甲方要求 0.30 m 停一次

```yaml
pause_distance_m: 0.30
pause_duration_s: 3.0
pause_smooth_stop_s: 0.15
pause_smooth_start_s: 0.20
```

或稍保守：

```yaml
pause_distance_m: 0.30
pause_duration_s: 3.0
pause_smooth_stop_s: 0.20
pause_smooth_start_s: 0.30
```

注意：0.30 m 一停时，不能把起停时间设置得太长。否则车几乎一直处于“停—起—停—起”，没有连续巡航距离修横差。

### 4.3 终点附近

```yaml
pause_skip_near_goal_m: 1.00
```

含义：终点前 1 m 不再触发分段停车，给车辆连续低速接近终点，避免最后几厘米还被停车打断。

---

## 5. 典型问题与调参路线

### 问题 A：横差修不回来

优先顺序：

```yaml
cross_track_lookahead_m: 2.0 -> 1.8
```

如果还不够：

```yaml
max_cross_track_yaw_correction_deg: 3.0 -> 3.5
```

仍不够且车轮动作不大：

```yaml
max_angular_speed: 0.08 -> 0.10
```

不建议直接把 `k_cross_track` 拉大，因为当前喷洒 line 模式下横差主要通过小航向偏置处理，不走传统大横差 P 抢方向路线。

### 问题 B：停车时车头不够平行

优先顺序：

```yaml
pre_stop_cte_keep_ratio: 0.35 -> 0.20
```

或者：

```yaml
pre_stop_align_distance_m: 0.15 -> 0.20
```

如果车头仍明显斜：

```yaml
max_cross_track_yaw_correction_deg: 3.0 -> 2.5
```

### 问题 C：轮子变化太快，上方装置晃动

优先顺序：

```yaml
max_angular_accel: 0.04 -> 0.03
```

再看最大角度：

```yaml
max_angular_speed: 0.08 -> 0.06
```

如果横杆晃动明显，宁可让横差慢慢收敛，不要强修。

### 问题 D：航向回正后横差又变大

优先顺序：

```yaml
pre_stop_cte_keep_ratio: 0.35 -> 0.45
```

或：

```yaml
pre_stop_align_distance_m: 0.15 -> 0.10
```

这说明停车前把横差修正退得太早或太多。

### 问题 E：刚起步或低速时像原地转

先确认停车保持阶段真实输出必须为：

```text
linear.x = 0
angular.z = 0
```

然后检查：

```yaml
enable_global_angular_slew: true
max_angular_accel: 0.03 ~ 0.04
```

如果使用含低线速度保护的版本，也应保证：

```yaml
zero_angular_below_min_linear: true
min_linear_for_angular: 0.03
```

---

## 6. 调参原则总结

1. 先保车头平行和机械稳定，再追横差收敛。
2. 横差允许几个停车周期内慢慢回，不要求每 0.5 m 归零。
3. 不要用大 `k_cross_track` 抢控制权。
4. 装置展开和电机上电后，控制参数应比空车更柔。
5. 0.30 m 停车必须缩短起停时间，否则横差没有连续距离收敛。
6. 若东西/西东方向总偏同侧，优先怀疑坡度、机械偏载或 yaw_offset，而不是盲目提高控制增益。
