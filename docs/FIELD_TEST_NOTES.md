# FIELD_TEST_NOTES：试车记录模板与现场判断

本文档用于现场试车时记录现象，避免只凭感觉调参。

---

## 1. 上车前检查清单

### 1.1 ROS 与话题

```bash
rostopic list
rostopic hz /novatel/oem7/inspvax
rostopic hz /novatel/oem7/bestutm
rostopic hz /cmd_vel
```

确认：

- RTK/INS 数据连续；
- `/cmd_vel` 可被底盘接收；
- 没有其他节点同时抢发 `/cmd_vel`；
- RTK 固定解或允许测试浮动解的参数设置符合现场需求。

### 1.2 底盘安全

停车保持阶段必须满足：

```text
linear.x = 0
angular.z = 0
```

因为该类 Ranger/AGV 角速度型底盘在 `linear.x = 0` 且 `angular.z != 0` 时，可能原地自转或原地打轮。

### 1.3 机械状态

每次试车必须记录：

- 横杆/丝杠是否展开；
- 传送带/转盘电机是否上电；
- 喷洒机构是否实际动作；
- 装置初始在左侧、中间还是右侧；
- 场地是否有坡度；
- 测试方向：南北、北南、东西、西东。

---

## 2. 推荐测试顺序

### 第 1 轮：空车或横杆未展开

目的：确认底盘和 RTK 控制链路正常。

建议参数：

```yaml
pause_distance_m: 0.50
reverse_speed: 0.10
max_angular_speed: 0.08
max_angular_accel: 0.04
```

观察：

- 是否能沿两点直线倒回；
- 车轮是否只有小角度修正；
- 停车点是否稳定；
- 停车时车头是否平行。

### 第 2 轮：横杆展开但电机不上电

目的：区分“纯机械展开偏载”和“电机动作/反扭矩”。

观察：

- 车轮动作是否比未展开时变大；
- 横差是否开始累积；
- 停车时是否仍平行。

### 第 3 轮：横杆展开且电机上电

目的：验证真实喷洒工况。

观察：

- 上方装置运动时底盘是否被明显扰动；
- 停车后再次起步是否打轮；
- 横差是缓慢收敛还是持续扩大；
- 是否出现 S 形。

### 第 4 轮：0.30 m 停车节拍

目的：验证甲方要求工况。

建议参数：

```yaml
pause_distance_m: 0.30
pause_smooth_stop_s: 0.15
pause_smooth_start_s: 0.20
```

观察：

- 是否因为停车太密导致车辆一直处于起停状态；
- 横差是否来不及收敛；
- 上方装置是否因频繁起停晃动。

---

## 3. 现场记录表

| 项目 | 记录 |
|---|---|
| 日期/时间 |  |
| 测试人员 |  |
| 场地 |  |
| 路线方向 | 南北 / 北南 / 东西 / 西东 |
| 直线距离 |  m |
| RTK 状态 | 固定解 / 浮动解 / 不稳定 |
| 横杆状态 | 未展开 / 展开未上电 / 展开上电 |
| 喷洒机构动作 | 无 / 左到右 / 右到左 / 左右随机 |
| 停车间隔 | 0.50 m / 0.30 m / 其他 |
| 倒车速度 |  m/s |
| 最大角速度参数 |  |
| 角速度变化率参数 |  |
| 初始横差 |  m |
| 中段最大横差 |  m |
| 终点横差 |  m |
| 是否 S 形 | 是 / 否 |
| 停车时车头是否平行 | 好 / 一般 / 差 |
| 轮子是否大幅动作 | 是 / 否 |
| 上方装置是否晃动 | 是 / 否 |
| 备注 |  |

---

## 4. 关键现象判断表

| 现象 | 更可能原因 | 优先处理 |
|---|---|---|
| 空车稳定，横杆展开后车轮明显变大 | 机械偏载/反扭矩/长力臂耦合 | 降低 `max_angular_accel` 和 `max_angular_speed` |
| 横差能变小，但停车前又变大 | 停车前横差修正退得太多 | 提高 `pre_stop_cte_keep_ratio` 或减小 `pre_stop_align_distance_m` |
| 停车时车头不平行 | 横差航向偏置保留太多 | 降低 `pre_stop_cte_keep_ratio` 或增大 `pre_stop_align_distance_m` |
| 横差一直修不回来 | 横差航向偏置太弱 | 降低 `cross_track_lookahead_m` 或提高 `max_cross_track_yaw_correction_deg` |
| 轮子动作太快 | 角速度变化率限制太松 | 降低 `max_angular_accel` |
| 轮子最大角度太大 | 最大角速度太高 | 降低 `max_angular_speed` |
| 0.30 m 停车时频繁晃 | 起停时间占用段距太多 | 缩短 `pause_smooth_stop_s` 和 `pause_smooth_start_s` |
| 同一方向总偏同一侧 | 坡度、机械偏置或 yaw_offset | 做四方向测试，不要先盲目加 P |
| 东向偏左、西向偏右 | 航向零偏可能性更大 | 检查 `yaw_offset_deg` 和航向源 |

---

## 5. 试车后建议保存的数据

即使暂时没有 `support_packages/rtk_trajectory_exporter`，也建议至少保存：

- `nav_params.yaml`；
- rosbag 或关键 topic 文本记录；
- 每次停车点的横差和航差截图；
- `/cmd_vel` 输出；
- RTK/INS 位姿；
- 现场照片或视频；
- 测试方向和机械状态。

后续加入 `rtk_trajectory_exporter` 后，建议导出：

```text
recorded_path.csv
control_path.csv
driven_path.csv
target_points.csv
all_in_one.csv
status_log.csv
```

---

## 6. 当前主线试车结论模板

可以按下面格式写当天总结：

```text
本次测试使用 10_rtk_reverse_nav_spray_cte_keep_final，tracking_mode=line，pause_distance_m=0.50。
横杆状态为：____。
车辆整体没有/有明显 S 形。
停车时车头平行性：____。
横差变化趋势：____。
轮子动作：____。
上方装置晃动：____。
下一步调参：____。
```

---

## 7. 不建议的现场操作

- 不要一上来把 `k_cross_track` 大幅提高到 1.2 或 1.5。
- 不要让停车保持阶段输出 `angular.z != 0`。
- 不要在 0.30 m 停车工况下使用过长的起停时间。
- 不要用固定 left/right 偏载补偿，因为装置初始位置随机，且停车后会左右切换。
- 不要同时运行多个 `/cmd_vel` 控制节点。
