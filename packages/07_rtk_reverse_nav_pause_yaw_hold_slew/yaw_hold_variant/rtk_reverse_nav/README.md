# RTK 两点打点倒车功能包：停车角度保持版

## 版本定位

本版以“两点打点直线倒车”为正式主用模式，多点轨迹复现作为备用模式。

针对实机上方 4 m 丝杠/长梁导致底盘大角度旋转后产生反扭矩的问题，本版重点加入：

1. 小角度优先控制；
2. 停车前角度保存；
3. 停车保持期间不重新找方向；
4. 重新起步时先按停车前角度直着倒；
5. 起步后再渐进恢复横向误差修正；
6. 低速时按最小转弯半径限制角速度，避免低速大转角；
7. 横向误差过大进入故障暂停，人工调回轨迹附近后可继续倒车。

## 启动

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch rtk_reverse_nav reverse_nav.launch
```

## 现场操作顺序

1. 将车停在倒车终点，点击“记录终点并开始录轨”；
2. 人工遥控正向开到倒车起点；
3. 点击“记录起点并停止录轨”；
4. 点击“启动自动倒车”；
5. 若横向误差过大停车，人工把车调整回原直线轨迹附近，再点击“人工调整后继续倒车”。

## 本版停车逻辑

分段停车时程序不是简单 `cmd_vel=0`，而是按下面流程执行：

```text
达到停车距离
↓
保存停车前 yaw
↓
减速阶段围绕停车前 yaw 把角速度压小
↓
停车保持，linear=0，angular=0
↓
重新起步，先保持停车前 yaw，不启用横向误差修正
↓
随后逐渐恢复横向纠偏
↓
回到正常两点倒车
```

## 推荐初始参数

默认 `config/nav_params.yaml` 已经设置成保守测试参数：

```yaml
reverse_speed: 0.10
k_heading: 0.75
k_cross_track: 0.18
max_angular_speed: 0.08
max_angular_accel: 0.10
min_turn_radius_m: 1.60
max_cross_track_error: 0.40

pause_distance_m: 0.30
pause_duration_s: 3.0
pause_smooth_stop_s: 0.80
pause_smooth_start_s: 1.20

hold_yaw_during_pause: true
pause_heading_k: 0.35
pause_heading_max_angular_speed: 0.03
resume_straight_time_s: 0.60
resume_cross_track_ramp_s: 1.20
pause_resume_max_angular_speed: 0.05
max_pause_angular_accel: 0.06
```

## 现场主要调哪些参数

### 1. 底盘还是大角度旋转

优先降低：

```yaml
max_angular_speed: 0.06 ~ 0.08
max_angular_accel: 0.06 ~ 0.10
pause_resume_max_angular_speed: 0.03 ~ 0.05
max_pause_angular_accel: 0.04 ~ 0.06
```

并适当增大：

```yaml
min_turn_radius_m: 1.8 ~ 2.2
resume_straight_time_s: 0.8 ~ 1.0
resume_cross_track_ramp_s: 1.5 ~ 2.0
```

### 2. 车很稳，但横向误差修得太慢

逐步增加，不要一次加太多：

```yaml
k_cross_track: 0.20 ~ 0.25
k_heading: 0.85 ~ 0.95
max_angular_speed: 0.09 ~ 0.10
```

### 3. 停车后起步仍然摆头

优先改：

```yaml
resume_straight_time_s: 0.8
resume_cross_track_ramp_s: 1.6
pause_heading_max_angular_speed: 0.02
pause_resume_max_angular_speed: 0.03
```

### 4. 每次停车距离太密，车还没稳定又停了

可以临时放大：

```yaml
pause_distance_m: 0.40 ~ 0.60
```

等停车起步稳定后，再改回 0.30 m。

### 5. 横向误差稍大就停车，实验不方便

可以略微放宽：

```yaml
max_cross_track_error: 0.45 ~ 0.55
resume_max_cross_track_error: 0.30 ~ 0.35
```

但不建议放太大，因为你的上部丝杠不适合靠底盘大角度强行纠偏。

## 参数含义速查

- `reverse_speed`：倒车线速度，越大越快，但结构冲击也越大。
- `k_heading`：航向误差修正系数，越大车头回正越快，也更容易摆。
- `k_cross_track`：横向误差修正系数，越大越主动追线，也更容易大角度打轮。
- `max_angular_speed`：底盘最大角速度，不是车轮转角，越小底盘越不容易大转。
- `max_angular_accel`：角速度变化率限制，越小起步和修正越柔和。
- `min_turn_radius_m`：按车速限制角速度的最小转弯半径，越大越限制大转角。
- `hold_yaw_during_pause`：开启停车前角度保持。
- `resume_straight_time_s`：停车后重新起步时，先按停车前角度倒车的时间。
- `resume_cross_track_ramp_s`：横向误差修正从 0 慢慢恢复到正常的时间。
- `pause_resume_max_angular_speed`：停车起步阶段允许的最大角速度。
- `pause_heading_max_angular_speed`：保持停车前角度时允许的最大角速度。

## 推荐测试顺序

第一轮只看“是否还会大角度旋转”：

```yaml
reverse_speed: 0.10
k_heading: 0.75
k_cross_track: 0.18
max_angular_speed: 0.08
pause_distance_m: 0.30
```

第二轮若稳定，再提高纠偏能力：

```yaml
reverse_speed: 0.12
k_heading: 0.90
k_cross_track: 0.22
max_angular_speed: 0.10
```

第三轮若停车起步仍然不稳，不要继续提高纠偏系数，先加长：

```yaml
resume_straight_time_s
resume_cross_track_ramp_s
pause_smooth_start_s
```

