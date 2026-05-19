# RTK 两点喷洒倒车功能包：硬停硬起 + 弱积分消稳态误差版

## 版本定位

本版用于你的喷洒作业工况：

- 两点模式生成一条直线；
- 车辆沿直线低速倒车；
- 每隔固定距离停车一次，触发继电器让上方丝杠/传送带横向作业；
- 倒车过程中不能大角度 S 形乱摆；
- 每次停车喷洒时，车头朝向尽量保持一致；
- 尽量消除长期 2~4 cm 级小稳态横向误差；
- 停车点间距尽量一致。

## 这一版主要修改

1. **去掉柔顺停车/柔顺起步**  
   `pause_smooth_stop_s` 和 `pause_smooth_start_s` 默认设为 `0.00`。车速很低时直接停、直接起，可以让每次喷洒点距离更接近设定的 `pause_distance_m`。

2. **加入弱横差积分**  
   积分项只修长期小稳态偏差，不参与大误差强修；停车、起步、刹车阶段不积分，并且积分有限幅和泄漏，避免越积越猛。

3. **保持喷洒直线控制逻辑**  
   横差不是直接变成角速度，而是生成很小的航向偏置角，让车辆慢慢靠回轨迹线，避免 S 形。

4. **所有现场参数只改 YAML**  
   `launch/reverse_nav.launch` 只加载 `config/nav_params.yaml`，不再覆盖参数。

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
5. 车辆每隔 `pause_distance_m` 停车，继电器闭合 `relay_pulse_seconds` 秒。

## 推荐默认参数

说明：现场测试中 `cross_track_lookahead_m: 1.0` 后横差明显减小；`0.5` m 虽然更有力，但容易诱发 S 形，因此不建议作为默认值。

```yaml
reverse_speed: 0.10
k_heading: 0.80
cross_track_lookahead_m: 1.0
max_cross_track_yaw_correction_deg: 3.0
pre_stop_align_distance_m: 0.15
pre_stop_cte_keep_ratio: 0.35

max_angular_speed: 0.08
max_angular_accel: 0.04

pause_distance_m: 0.50
pause_smooth_stop_s: 0.00
pause_smooth_start_s: 0.00

# 弱积分
k_cross_track_i: 0.02
max_integral_yaw_correction_deg: 0.8
integral_leak_rate: 0.03
```

## 现场调参建议

### 1. 小稳态横差还消不掉

先小幅加积分：

```yaml
k_cross_track_i: 0.02 -> 0.03
```

不要一上来超过 `0.04`。

### 2. 车头/车轮开始有 S 形趋势

降低横差修正角或增强泄漏：

```yaml
max_cross_track_yaw_correction_deg: 3.0 -> 2.5
k_cross_track_i: 0.03 -> 0.02
integral_leak_rate: 0.03 -> 0.05
```

### 3. 停车点距离仍偏大

车速很低时，优先保持硬停硬起：

```yaml
pause_smooth_stop_s: 0.00
pause_smooth_start_s: 0.00
```

如果实际停点仍比设定远，说明底盘有机械惯性/制动延迟，可以把喷洒间距设小一点补偿：

```yaml
pause_distance_m: 0.50 -> 0.47
```

### 4. 横差修正还不够有力

当前现场推荐值为：

```yaml
cross_track_lookahead_m: 1.0
```

如果仍修不回来，优先小幅增大最大横差修正角；不要继续降到 `0.5` m，因为现场已出现 S 形趋势。

```yaml
max_cross_track_yaw_correction_deg: 3.0 -> 3.5
```

如果车头开始摆动，则反向调柔：

```yaml
cross_track_lookahead_m: 1.0 -> 1.2
max_cross_track_yaw_correction_deg: 3.0 -> 2.5
```

### 5. 车轮动作还是快

优先降低角速度变化率：

```yaml
max_angular_accel: 0.04 -> 0.03
```

再降低最大角速度：

```yaml
max_angular_speed: 0.08 -> 0.06
```

## 注意

弱积分只能用于消除小稳态误差，例如 2~4 cm 的长期偏差；它不适合修 10 cm 以上的大误差。大误差还是要靠 `cross_track_lookahead_m`、`max_cross_track_yaw_correction_deg` 和机械结构优化。
