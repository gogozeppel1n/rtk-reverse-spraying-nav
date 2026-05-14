# rtk_reverse_nav 随机左右装置工况优化版

本版适配你的实际工况：长横杆上的装置会在停车后通过带传动从左侧移动到右侧，再从右侧移动到左侧，且初始左右位置随机。

因此本版**不做左/右固定偏载补偿**，避免软件猜错方向导致越修越偏。控制思路为：

- 保留原来的两点模式和多点轨迹模式控制逻辑；
- 正常巡航仍使用 `k_heading * 航差 - k_cross_track * 横差`；
- 停车保持时真实输出 `linear=0, angular=0`，防止角速度型底盘原地自转；
- 内部保存停车前角速度修正趋势，起步时影子平滑接回；
- 刚恢复导航时，线速度低于阈值不输出 angular；
- 终点前 1 m 不再分段停车，降速连续精修横向误差。

## 推荐默认参数

```yaml
reverse_speed: 0.12
k_cross_track: 0.85
max_angular_speed: 0.22
pause_distance_m: 0.50
pause_smooth_stop_s: 0.20
pause_smooth_start_s: 0.30
pause_skip_near_goal_m: 1.00
final_slowdown_distance: 1.00
final_reverse_speed: 0.06
final_cross_track_tolerance: 0.03
zero_angular_below_min_linear: true
min_linear_for_angular: 0.03
```

如果甲方必须每 0.30 m 停一次，把 `pause_distance_m` 改为 `0.30`，同时建议：

```yaml
pause_smooth_stop_s: 0.15
pause_smooth_start_s: 0.20
```

## 安装

```bash
cd ~/catkin_ws/src
rm -rf rtk_reverse_nav
unzip ~/下载/rtk_reverse_nav_random_payload_shadow_final.zip
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch rtk_reverse_nav reverse_nav.launch
```

## 现场调参优先级

1. 装置晃动明显：先降低 `max_angular_speed`，例如 `0.22 -> 0.18`。
2. 横差修不回来：先加大 `pause_skip_near_goal_m` 到 `1.20`，再考虑 `k_cross_track: 0.85 -> 0.90`。
3. 起步仍有原地转风险：把 `min_linear_for_angular` 调到 `0.05`。
4. 甲方要求停车更密：`pause_distance_m` 改 `0.30`，同时缩短停车/起步柔顺时间。
5. 不要启用左右固定补偿；此工况装置左右随机，固定补偿容易猜错。
