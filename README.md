# RTK Reverse Spraying Nav

这是一个用于 **ROS1 Noetic + Ranger/AGV 类底盘 + NovAtel RTK/INS** 的倒车喷洒作业控制归档仓库。项目核心场景是：两点确定一条直线，小车沿该直线倒车，每隔固定距离停车一次，停车后由上方丝杠/传送带横向运动完成喷洒。

## 1. 项目真实工况

- 底盘：Ranger/AGV 类平台，控制输出为 `/cmd_vel`，消息类型 `geometry_msgs/Twist`。
- 定位：NovAtel RTK/INS，优先使用 RTK/INS 位姿。
- 主用模式：两点模式确定直线，沿该直线倒车。
- 作业节拍：默认每 `0.50 m` 停一次；甲方可能要求每 `0.30 m` 停一次。
- 喷洒机构：停车后上方丝杠/传送带横向运动一次。
- 喷洒要求：每次停车时车头朝向基本一致，与上一段停车时平行，保证喷洒均匀。
- 控制要求：不追求每 50 cm 都把横差瞬间修到 0，但横差不能持续累积；不能为了修横差导致大角度转向、S 形摆动或上方装置明显晃动。

## 2. 当前主线结论

现场已确认：丝杠/横杆未展开时车轮修正角度小；丝杠/横杆展开且电机上电后车轮修正角度明显变大。主要原因不是电气或传感器干扰，而是机械机构展开后带来的反扭矩、偏载和长力臂耦合。

因此当前主线不是继续加大 `k_cross_track` 抢方向，也不是固定 left/right 偏载补偿，而是使用：

```text
固定作业航向
+ 横差转小航向偏置
+ 停车前保留少量横差修正
+ 角速度限幅
+ 角速度变化率限制
```

当前推荐主用版本是：

```text
packages/10_rtk_reverse_nav_spray_cte_keep_final/rtk_reverse_nav
```

## 3. 仓库结构

```text
rtk-reverse-spraying-nav/
├── README.md
├── packages/
│   ├── 01_rtk_reverse_nav_save_dir_baseline/
│   ├── 02_rtk_reverse_nav_pause_relay_initial_problematic/
│   ├── 03_rtk_reverse_nav_stable_pause_relay/
│   ├── 04_rtk_reverse_nav_shadow_pause_relay/
│   ├── 05_rtk_reverse_nav_shadow_payload_optimized/
│   ├── 06_rtk_reverse_nav_random_payload_shadow_final/
│   ├── 07_rtk_reverse_nav_pause_yaw_hold_slew/
│   ├── 08_rtk_reverse_nav_yaml_only_tuning/
│   ├── 09_rtk_reverse_nav_spray_heading_stable/
│   └── 10_rtk_reverse_nav_spray_cte_keep_final/
├── docs/
│   ├── VERSION_HISTORY.md
│   ├── PARAMETER_TUNING_GUIDE.md
│   └── FIELD_TEST_NOTES.md
└── original_archive/
    └── rtk-reverse-spraying-nav.original.rar
```

说明：`support_packages/rtk_trajectory_exporter` 暂未纳入本次归档，后续找到后再单独加入。本次打包保留了原始上传压缩包到 `original_archive/`，用于追溯原始文件。

## 4. 版本选择建议

| 版本 | 定位 | 是否建议上车 |
|---|---|---|
| 01 | 添加保存目录的基础稳定倒车版 | 只作基线追溯 |
| 02 | 初始停车继电器版，暴露了停车状态机问题 | 不建议上车，仅保留问题版本 |
| 03 | 稳定停车继电器版 | 可参考，但不是当前喷洒主线 |
| 04 | 影子停车 + 低线速度角速度保护 | 用于解决停车恢复瞬间打轮问题 |
| 05 | 随机偏载/长横杆适配优化 | 中间工程版本 |
| 06 | 随机偏载影子最终版 | 中间工程版本，与 05 基本同源 |
| 07 | 停车角度保持 + 角速度变化率限制 | 中间探索版本 |
| 08 | YAML-only 调参思路 | 未放源码，仅保留占位说明 |
| 09 | 喷洒固定航向稳定版 | 能抑制 S 形，但横差修正偏弱 |
| 10 | 喷洒 cte_keep 优化版 | **当前推荐主用版本** |

详细演化见：[`docs/VERSION_HISTORY.md`](docs/VERSION_HISTORY.md)。


## 5. 重要编译说明

本仓库是“多版本归档仓库”，`packages/` 下多个历史版本的 ROS 包可能使用相同的 package name，例如 `rtk_reverse_nav`。

因此不要把整个 `rtk-reverse-spraying-nav/` 直接放进 `~/catkin_ws/src` 后整体 `catkin_make`。正确做法是：只复制当前要测试的某一个版本包到 `catkin_ws/src`，例如当前推荐版本：

```bash
cd ~/catkin_ws/src
cp -r /path/to/rtk-reverse-spraying-nav/packages/10_rtk_reverse_nav_spray_cte_keep_final/rtk_reverse_nav ./
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 6. 当前主用版本快速使用

将 10 号包复制到 catkin 工作空间：

```bash
cd ~/catkin_ws/src
cp -r /path/to/rtk-reverse-spraying-nav/packages/10_rtk_reverse_nav_spray_cte_keep_final/rtk_reverse_nav ./
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch rtk_reverse_nav reverse_nav.launch
```

现场调参只改：

```text
rtk_reverse_nav/config/nav_params.yaml
```

不建议现场直接改 `launch` 或主控 Python 文件。

## 7. 当前推荐参数

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

调参方法见：[`docs/PARAMETER_TUNING_GUIDE.md`](docs/PARAMETER_TUNING_GUIDE.md)。

## 8. 上车前必须注意

1. 停车保持阶段必须保证 `linear.x = 0` 且 `angular.z = 0`，否则 Ranger/AGV 类角速度型底盘可能原地自转或原地打轮。
2. 不要用 `/ranger_base_node/odom` 作为主导航坐标，主定位应来自 RTK/INS。
3. 不要为了让横差快速归零而盲目增大 `k_cross_track`，这会导致车头斜、停车朝向不一致、上方机构晃动，甚至形成 S 形。
4. 甲方要求 `0.30 m` 停一次时，必须缩短起停时间，否则车辆几乎一直处于“停—起—停—起”，横差没有连续距离收敛。
5. 当前 10 号版本优先解决“喷洒工况下车头平行、慢慢修横差、不激发上方长横杆”的问题。

## 9. 文档说明

- [`docs/VERSION_HISTORY.md`](docs/VERSION_HISTORY.md)：版本前因后果、每个功能包做了什么、为什么改、效果如何。
- [`docs/PARAMETER_TUNING_GUIDE.md`](docs/PARAMETER_TUNING_GUIDE.md)：现场参数含义、调参路线、0.5 m 和 0.3 m 停车参数建议。
- [`docs/FIELD_TEST_NOTES.md`](docs/FIELD_TEST_NOTES.md)：试车记录模板、测试步骤、故障现象判断表。

## 10. License / 许可证

本项目采用 Apache License 2.0 许可证。具体条款见 [LICENSE](LICENSE) 文件。

Copyright 2026 Hairong Gu, gogozeppel1n, Tianyi Yang, Haonan Li.

版权所有 © 2026 顾海荣、gogozeppel1n、杨天一、李浩南。
