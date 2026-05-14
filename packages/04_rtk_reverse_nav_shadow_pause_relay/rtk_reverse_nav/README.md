# rtk_reverse_nav 段间直线轨迹复现版

这版不是纯跟踪。

目标行为：
- 录制人工正向轨迹时保留原始点和原始车身朝向。
- 自动倒车时，按**回放点序列**逐段复现。
- 当前段只做一件事：**从上一点沿直线走到下一点**。
- 启动前要求车辆已停在回放起点附近，且车头朝向和录制起点姿态接近。
- 如果起步姿态不对，会拒绝启动，而不是自己随意摆车头。

## 和旧版区别
- 去掉按几何切线预瞄的纯跟踪逻辑。
- 不再根据 `path_heading + pi` 自己找倒车方向。
- 多点轨迹模式下直接使用**反序后的录制点**作为回放点。
- 每一段都只跟踪当前 `p0 -> p1` 直线段，同时参考录制时的 yaw 插值。
- 倒车速度固定为 `reverse_speed`。

## 启动前要求
1. 设备位姿已经正确。
2. 车辆停在回放起点附近。
3. 车辆当前车头朝向与录制起点 yaw 接近。

## 启动
```bash
roslaunch rtk_reverse_nav reverse_nav.launch
```

## 关键参数
- `reverse_speed`: 固定倒车速度
- `startup_heading_tolerance_deg`: 起步姿态允许误差
- `waypoint_reach_tolerance`: 段终点切换阈值
- `k_heading`: 姿态误差控制增益
- `k_cross_track`: 当前段横向误差控制增益

## 说明
如果你看到启动时航差仍然接近 90 度，优先检查：
- MS6111 旋转角配置
- 杆臂配置
- `/rtk_reverse_nav/pose` 的 yaw 是否和真实车头一致

## CSV 保存与导入

本版默认把保存的 CSV 放到工控机用户 `agx` 的主目录文件夹：

```bash
/home/agx/rtk_reverse_paths/
```

文件名按保存时间自动生成，例如：

```bash
rtk_reverse_path_20260426_143012.csv
```

保存的 CSV 是**自动倒车执行顺序**，也就是导入后会按 CSV 行顺序复现，不会再次反转。

### 从界面导入

启动 `reverse_nav.launch` 后，在控制面板点击：

```text
选择并导入 CSV 轨迹
```

选择 CSV 文件后，节点会发布导入后的轨迹。把车停到导入轨迹第一个点附近，再点击【启动自动倒车】。

### 从命令行导入

```bash
rosparam set /reverse_track_nav_node/import_path_file /home/agx/rtk_reverse_paths/rtk_reverse_path_20260426_143012.csv
rosservice call /reverse_track_nav_node/import_path
```

随后把车停到导入轨迹第一个点附近，再启动自动倒车：

```bash
rosservice call /reverse_track_nav_node/start_auto_reverse
```

## 稳定停车 + 继电器版本说明

本版以“添加保存目录版”为运动控制基线：两点直线模式、多点轨迹模式、横向误差/航向误差控制公式均保持原逻辑，只在 `/cmd_vel` 输出外层增加分段停车管理。

核心修复：
- 分段停车不再使用旧的全局阈值连续累加；每次停车保持结束并完成柔顺起步后，才以当前位置重新累计下一次停车距离。
- 停车保持阶段 `linear.x = 0` 且 `angular.z = 0`，避免底盘进入原地自转或原地转向。
- 减速阶段线速度和角速度一起衰减；起步阶段限制角速度幅值和变化率，避免起步时轮胎猛打。
- 继电器闭合使用独立线程，不阻塞停车保持计时。

关键参数在 `config/nav_params.yaml` 中：

```yaml
enable_segment_pause: true
pause_distance_m: 0.30
pause_duration_s: 3.0
pause_smooth_stop_s: 0.35
pause_smooth_start_s: 0.50
pause_start_angular_limit: 0.05
max_pause_angular_accel: 0.10

enable_relay: true
relay_port: auto
relay_pulse_seconds: 2.0
```

临时关闭继电器只测停车运动：

```bash
roslaunch rtk_reverse_nav reverse_nav.launch enable_relay:=false
```

临时关闭停车，恢复保存目录版连续运动：

```bash
roslaunch rtk_reverse_nav reverse_nav.launch enable_segment_pause:=false
```

手动测试继电器：

```bash
rosservice call /reverse_track_nav_node/pulse_relay
```

如果提示缺少 serial：

```bash
sudo apt install python3-serial
```

## 影子停车 + 继电器说明

本版以“添加保存目录版”的两点模式和多点轨迹模式为运动控制基线，正常巡航时仍使用原始控制公式：

```text
angular_z = k_heading * heading_error - k_cross_track * cross_track_error
linear_x = -reverse_speed
```

新增的停车逻辑只作为外层状态机，不修改原来的横向误差、航向误差、段号推进和轨迹复现算法。

### 影子模式

由于底盘在 `linear.x=0`、`angular.z!=0` 时可能进入原地旋转/原地转向状态，本版停车保持阶段真实输出始终为：

```text
linear.x = 0
angular.z = 0
```

但程序内部会保存停车前最后一次稳定的转向修正量 `pause_shadow_angular`。停车结束后柔顺起步时，角速度不是从 0 突然恢复，而是从保存的影子修正量平滑过渡到当前实时计算的控制量。这样能尽量保持原“添加保存目录版”的连续修正趋势，同时避免停车阶段原地自转。

### 默认参数建议

```yaml
pause_distance_m: 0.50
pause_duration_s: 3.0
pause_smooth_stop_s: 0.25
pause_smooth_start_s: 0.40
pause_shadow_steering: true
post_start_slew_s: 0.80
pause_start_angular_limit: 0.12
max_pause_angular_accel: 0.12
```

如果演示时横向误差仍有累积，可优先把 `pause_distance_m` 增大到 `0.80`，让车辆获得更长连续修正距离；不要优先修改原始导航控制公式。


## 影子停车模式与角速度型底盘保护

本版停车逻辑采用“影子模式”：停车保持阶段真实输出 `linear=0, angular=0`，避免角速度型底盘在停车时进入原地自转；程序内部保留停车前最后一次轨迹修正角速度，柔顺起步时再平滑接回原来的轨迹控制。

为避免刚恢复导航时出现“线速度太小但角速度已经输出”的情况，增加了线速度死区保护：

```yaml
zero_angular_below_min_linear: true
min_linear_for_angular: 0.03
```

含义：只要输出线速度绝对值小于 `0.03 m/s`，就强制 `angular.z=0`；线速度超过该阈值后才允许逐步恢复角速度修正。正常巡航阶段仍使用原始轨迹控制公式，不改变两点模式和多点轨迹模式的跟踪逻辑。
