# rtk_reverse_nav：喷洒直线 cte_keep 优化版

## 版本定位

本包是当前推荐主用版本，适用于：

```text
Ranger/AGV 类底盘
+ NovAtel RTK/INS
+ 两点模式直线倒车
+ 每隔固定距离停车
+ 上方丝杠/传送带横向喷洒
```

本版核心目标不是每 0.5 m 都把横差瞬间修到 0，而是在喷洒作业中实现：

```text
车头停车基本平行
横差不持续累积
车轮动作不过大
避免 S 形摆动
减少上方长横杆晃动
```

## 核心控制思想

传统大横差 P 控制容易出现：横差变小但车头斜、停车朝向不一致、下一段又反向修，最后形成 S 形。

本版改为：

```text
固定作业航向
+ 横差转小航向偏置
+ 停车前保留一部分横差修正
+ 角速度限幅
+ 角速度变化率限制
```

横差修正不再直接靠大 `k_cross_track` 抢方向，而是通过：

```text
cte_yaw_correction = atan2(-cross_track_error, cross_track_lookahead_m)
```

再由 `max_cross_track_yaw_correction_deg` 限制最大偏角。

## 相比上一版的关键变化

相比 `09_rtk_reverse_nav_spray_heading_stable`，本版新增：

```yaml
pre_stop_cte_keep_ratio: 0.35
```

含义：接近停车点时，不把横差航向偏置完全退到 0，而是保留 35%。这样可以避免“横差刚修小，航向一回正，横差又变大”。

默认参数同步调整为：

```yaml
k_heading: 0.80
cross_track_lookahead_m: 2.0
max_cross_track_yaw_correction_deg: 3.0
pre_stop_align_distance_m: 0.15
pre_stop_cte_keep_ratio: 0.35
max_angular_speed: 0.08
max_angular_accel: 0.04
```

## 编译与启动

```bash
cd ~/catkin_ws/src
cp -r /path/to/rtk_reverse_nav ./
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch rtk_reverse_nav reverse_nav.launch
```

## 现场操作顺序

1. 启动底盘、RTK/INS 驱动，确认 RTK 位姿连续。
2. 启动本包：

```bash
roslaunch rtk_reverse_nav reverse_nav.launch
```

3. 将车停在倒车终点，记录终点。
4. 人工正向沿作业直线开到倒车起点，记录起点。
5. 启动自动倒车。
6. 车辆每隔 `pause_distance_m` 停车，停车期间触发喷洒机构。

## 当前推荐参数

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

## 常用调参

- 横差修不回来：`cross_track_lookahead_m: 2.0 -> 1.8`，或 `max_cross_track_yaw_correction_deg: 3.0 -> 3.5`。
- 停车时车头不够平行：`pre_stop_cte_keep_ratio: 0.35 -> 0.20`，或 `pre_stop_align_distance_m: 0.15 -> 0.20`。
- 轮子变化太快：`max_angular_accel: 0.04 -> 0.03`。
- 轮子最大角度仍大：`max_angular_speed: 0.08 -> 0.06`。
- 甲方要求 0.3 m 停一次：`pause_distance_m: 0.30`，`pause_smooth_stop_s: 0.15~0.20`，`pause_smooth_start_s: 0.20~0.30`。

## 底盘安全注意

停车保持阶段必须保证：

```text
linear.x = 0
angular.z = 0
```

该类角速度型底盘在 `linear.x = 0` 且 `angular.z != 0` 时，可能出现原地自转或原地打轮。
