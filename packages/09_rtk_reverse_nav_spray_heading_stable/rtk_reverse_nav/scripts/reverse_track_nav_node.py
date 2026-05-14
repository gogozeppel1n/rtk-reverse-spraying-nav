#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import math
import os
import threading
import time
from datetime import datetime
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import Trigger, TriggerResponse
import tf.transformations as tft

try:
    import serial
    from serial.tools import list_ports
except Exception:
    serial = None
    list_ports = None


def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def clamp(value, low, high):
    return max(low, min(high, value))


def quat_to_yaw(q):
    return tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def interp_angle(a0, a1, ratio):
    return wrap_to_pi(a0 + ratio * wrap_to_pi(a1 - a0))


class PathPoint(object):
    __slots__ = ('x', 'y', 'yaw')

    def __init__(self, x, y, yaw=0.0):
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)


class ReverseTrackNavNode(object):
    def __init__(self):
        rospy.init_node('reverse_track_nav_node', anonymous=False)
        self.lock = threading.Lock()

        self.pose_topic = rospy.get_param('~pose_topic', '/rtk_reverse_nav/pose')
        self.fix_valid_topic = rospy.get_param('~fix_valid_topic', '/rtk_reverse_nav/fix_valid')
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        # CSV 保存目录默认放在 agx 用户主目录下，文件名按时间自动生成。
        # path_file 仍保留兼容旧 launch；如果显式传入非空 path_file，则保存到该固定文件。
        self.path_dir = rospy.get_param('~path_dir', '/home/agx/rtk_reverse_paths')
        self.path_file = rospy.get_param('~path_file', '')
        self.path_prefix = rospy.get_param('~path_prefix', 'rtk_reverse_path')
        self.import_path_file = rospy.get_param('~import_path_file', '')
        self.frame_id = rospy.get_param('~frame_id', 'utm')
        self.tracking_mode = rospy.get_param('~tracking_mode', 'line').strip().lower()

        self.record_point_spacing = rospy.get_param('~record_point_spacing', 0.20)
        self.record_yaw_spacing_deg = rospy.get_param('~record_yaw_spacing_deg', 4.0)
        self.reverse_speed = abs(rospy.get_param('~reverse_speed', 0.15))
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 0.30)
        # 小角度优先控制：限制角速度绝对值、角速度变化率，并在低速起步时按最小转弯半径再次限幅。
        self.enable_global_angular_slew = rospy.get_param('~enable_global_angular_slew', True)
        self.max_angular_accel = rospy.get_param('~max_angular_accel', 0.15)
        self.enable_speed_based_angular_limit = rospy.get_param('~enable_speed_based_angular_limit', True)
        self.min_turn_radius_m = max(0.30, rospy.get_param('~min_turn_radius_m', 1.60))
        self.hold_yaw_during_pause = rospy.get_param('~hold_yaw_during_pause', True)
        self.pause_hold_yaw_average_s = rospy.get_param('~pause_hold_yaw_average_s', 0.30)
        self.pause_heading_k = rospy.get_param('~pause_heading_k', 0.35)
        self.pause_heading_max_angular_speed = rospy.get_param('~pause_heading_max_angular_speed', 0.03)
        self.resume_straight_time_s = rospy.get_param('~resume_straight_time_s', 0.60)
        self.resume_cross_track_ramp_s = rospy.get_param('~resume_cross_track_ramp_s', 1.20)
        self.pause_resume_max_angular_speed = rospy.get_param('~pause_resume_max_angular_speed', 0.05)
        self.resume_disable_cross_track_first = rospy.get_param('~resume_disable_cross_track_first', True)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.20)
        self.goal_yaw_tolerance_deg = rospy.get_param('~goal_yaw_tolerance_deg', 20.0)
        self.require_goal_yaw = rospy.get_param('~require_goal_yaw', False)
        self.control_rate = rospy.get_param('~control_rate', 15.0)
        self.publish_zero_on_idle = rospy.get_param('~publish_zero_on_idle', True)
        self.pose_timeout = rospy.get_param('~pose_timeout', 0.60)
        self.startup_max_distance = rospy.get_param('~startup_max_distance', 0.60)
        self.startup_heading_tolerance_deg = rospy.get_param('~startup_heading_tolerance_deg', 20.0)
        self.runtime_heading_limit_deg = rospy.get_param('~runtime_heading_limit_deg', 75.0)
        self.k_heading = rospy.get_param('~k_heading', 1.5)
        self.k_cross_track = rospy.get_param('~k_cross_track', 0.8)
        # 喷洒两点直线模式：横差不再直接抢方向盘，而是转换成很小的目标航向偏置。
        # 这样每 0.5 m 停车喷洒时，车头尽量保持和上一段平行，避免 S 形。
        self.enable_spray_line_control = rospy.get_param('~enable_spray_line_control', True)
        self.spray_line_yaw_source = str(rospy.get_param('~spray_line_yaw_source', 'geometry_reverse')).strip().lower()
        self.cross_track_lookahead_m = max(0.30, float(rospy.get_param('~cross_track_lookahead_m', 3.0)))
        self.max_cross_track_yaw_correction = math.radians(float(rospy.get_param('~max_cross_track_yaw_correction_deg', 2.0)))
        self.pre_stop_align_distance_m = max(0.0, float(rospy.get_param('~pre_stop_align_distance_m', 0.20)))
        self.heading_deadband = math.radians(float(rospy.get_param('~heading_deadband_deg', 0.4)))
        self.max_cross_track_error = rospy.get_param('~max_cross_track_error', 0.80)
        # 横向误差过大时不清空轨迹，只进入故障暂停；人工调回轨迹附近后可继续自动倒车。
        self.resume_max_cross_track_error = rospy.get_param('~resume_max_cross_track_error', 0.35)
        self.resume_heading_tolerance_deg = rospy.get_param('~resume_heading_tolerance_deg', 30.0)
        self.resume_projection_margin_m = rospy.get_param('~resume_projection_margin_m', 0.30)
        # 到点容差按实车控制能力设置，长距离时不能小于车辆实际横向/定位误差。
        self.waypoint_reach_tolerance = rospy.get_param('~waypoint_reach_tolerance', 0.25)
        # 长距离轨迹复现防漏点：车没压到点，但已经越过当前段时，也允许切到下一段。
        self.segment_advance_ratio = rospy.get_param('~segment_advance_ratio', 0.90)
        self.enable_projection_segment_advance = rospy.get_param('~enable_projection_segment_advance', True)
        # 漏点严重时，只在当前段前方小范围内恢复段号，不改变轨迹复现大逻辑。
        self.enable_nearest_segment_recovery = rospy.get_param('~enable_nearest_segment_recovery', True)
        self.segment_search_ahead = int(rospy.get_param('~segment_search_ahead', 12))
        self.segment_recovery_min_improve = rospy.get_param('~segment_recovery_min_improve', 0.08)
        # 对短段自动缩小到点容差，防止 yaw 触发的短段被一口气跳过。
        self.waypoint_tolerance_segment_ratio = rospy.get_param('~waypoint_tolerance_segment_ratio', 0.60)
        self.waypoint_min_reach_tolerance = rospy.get_param('~waypoint_min_reach_tolerance', 0.08)
        # 分段停车/继电器只作为“外层暂停器”，不改变原来的轨迹控制算法。
        self.enable_segment_pause = rospy.get_param('~enable_segment_pause', True)
        self.pause_distance_m = rospy.get_param('~pause_distance_m', 0.30)
        self.pause_duration_s = rospy.get_param('~pause_duration_s', 3.0)
        self.pause_smooth_stop_s = max(0.01, rospy.get_param('~pause_smooth_stop_s', 0.35))
        self.pause_smooth_start_s = max(0.01, rospy.get_param('~pause_smooth_start_s', 0.50))
        self.pause_skip_near_goal_m = rospy.get_param('~pause_skip_near_goal_m', 0.35)
        self.pause_start_angular_limit = rospy.get_param('~pause_start_angular_limit', 0.05)
        self.enable_angular_slew_in_pause = rospy.get_param('~enable_angular_slew_in_pause', True)
        self.max_pause_angular_accel = rospy.get_param('~max_pause_angular_accel', 0.10)

        # USB 转继电器参数。relay_port=auto 时自动选择 ttyUSB/ttyACM 候选串口。
        self.enable_relay = rospy.get_param('~enable_relay', True)
        self.relay_port = str(rospy.get_param('~relay_port', 'auto')).strip()
        self.relay_baudrate = int(rospy.get_param('~relay_baudrate', 9600))
        self.relay_pulse_seconds = float(rospy.get_param('~relay_pulse_seconds', 2.0))
        self.relay_serial_timeout = float(rospy.get_param('~relay_serial_timeout', 1.0))
        self.relay_on_hex = str(rospy.get_param('~relay_on_hex', 'A0 01 01 A2'))
        self.relay_off_hex = str(rospy.get_param('~relay_off_hex', 'A0 01 00 A1'))
        self.enable_path_dedup = rospy.get_param('~enable_path_dedup', True)
        self.path_dedup_spacing = rospy.get_param('~path_dedup_spacing', 0.05)
        # yaw 变化打点保留；但几乎原地的小 yaw 抖动不额外打点。
        # 如果想完全恢复原版 yaw 打点行为，把 record_yaw_min_distance 设为 0.0。
        self.record_yaw_min_distance = rospy.get_param('~record_yaw_min_distance', 0.10)
        # 去重时，距离近但 yaw 变化明显的点也保留，避免转弯点被删。
        self.path_dedup_yaw_keep_deg = rospy.get_param('~path_dedup_yaw_keep_deg', 2.0)

        self.current_pose = None
        self.current_pose_stamp = None
        self.fix_valid = False
        self.recording = False
        self.auto_reverse = False
        self.fault_paused = False
        self.fault_reason = ''
        self.pause_until = None
        self.next_pause_progress = None
        self.end_point = None
        self.start_point = None
        self.recorded_path = []
        self.replay_path = []
        self.line_path = []
        self.imported_path_active = False
        self.last_saved_csv = ''
        self.last_imported_csv = ''
        self.last_saved_point = None
        self.segment_index = 0
        self.last_status = ''
        self.last_progress_s = 0.0
        self.pause_state = 'cruise'
        self.pause_state_start_time = None
        self.next_pause_progress = None
        self.pause_progress_at_state_start = 0.0
        self.last_cmd_angular = 0.0
        self.last_cmd_time = None
        self.yaw_history = []
        self.pause_hold_yaw = None
        self.pause_resume_start_time = None
        self.relay_lock = threading.Lock()
        self.relay_busy = False
        self.relay_last_port = ''

        self.pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_cb, queue_size=50)
        self.fix_valid_sub = rospy.Subscriber(self.fix_valid_topic, Bool, self.fix_valid_cb, queue_size=20)
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=20)
        self.path_pub = rospy.Publisher('~recorded_path', Path, queue_size=1, latch=True)
        self.replay_pub = rospy.Publisher('~replay_path', Path, queue_size=1, latch=True)
        self.target_pub = rospy.Publisher('~target_point', PointStamped, queue_size=10)
        self.status_pub = rospy.Publisher('~status_text', String, queue_size=10, latch=True)
        self.count_pub = rospy.Publisher('~path_count', Int32, queue_size=10, latch=True)

        self.srv_record_end = rospy.Service('~record_end_and_start', Trigger, self.srv_record_end_and_start)
        self.srv_record_start = rospy.Service('~record_start_and_stop', Trigger, self.srv_record_start_and_stop)
        self.srv_start_auto = rospy.Service('~start_auto_reverse', Trigger, self.srv_start_auto_reverse)
        self.srv_resume_auto = rospy.Service('~resume_auto_reverse', Trigger, self.srv_resume_auto_reverse)
        self.srv_stop_auto = rospy.Service('~stop_auto', Trigger, self.srv_stop_auto)
        self.srv_clear = rospy.Service('~clear_path', Trigger, self.srv_clear_path)
        self.srv_save = rospy.Service('~save_path', Trigger, self.srv_save_path)
        self.srv_import = rospy.Service('~import_path', Trigger, self.srv_import_path)
        self.srv_pulse_relay = rospy.Service('~pulse_relay', Trigger, self.srv_pulse_relay)
        self.srv_set_line = rospy.Service('~set_line_mode', Trigger, self.srv_set_line_mode)
        self.srv_set_path = rospy.Service('~set_path_mode', Trigger, self.srv_set_path_mode)

        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.control_rate, 1.0)), self.control_loop)
        self.publish_status('节点已启动，等待 RTK 位姿。当前模式: {}'.format(self.tracking_mode))

    def publish_status(self, text):
        if text != self.last_status:
            rospy.loginfo(text)
            self.last_status = text
        self.status_pub.publish(String(data=text))
        self.count_pub.publish(Int32(data=len(self.recorded_path)))

    def fix_valid_cb(self, msg):
        with self.lock:
            self.fix_valid = bool(msg.data)

    def pose_cb(self, msg):
        with self.lock:
            yaw = quat_to_yaw(msg.pose.orientation)
            self.current_pose = PathPoint(msg.pose.position.x, msg.pose.position.y, yaw)
            self.current_pose_stamp = msg.header.stamp
            stamp = msg.header.stamp.to_sec() if msg.header.stamp and msg.header.stamp.to_sec() > 0.0 else rospy.Time.now().to_sec()
            self.yaw_history.append((stamp, yaw))
            # 只保留最近 2 秒航向，用于分段停车时提取“停车前角度”，避免静止后航向抖动影响起步。
            keep_after = stamp - 2.0
            self.yaw_history = [(t, y) for (t, y) in self.yaw_history if t >= keep_after]
            # 两点打点模式只记录“终点”和“起点”两个关键点，
            # 不在人工遥控行驶过程中自动追加中间点。
            # 多点轨迹模式作为备用，仍按间距/航向变化连续录轨。
            if self.recording and self.fix_valid and self.tracking_mode != 'line':
                self.try_append_record_point(self.current_pose)

    def pose_is_fresh(self):
        if self.current_pose is None or self.current_pose_stamp is None:
            return False
        age = (rospy.Time.now() - self.current_pose_stamp).to_sec()
        return age <= self.pose_timeout

    def current_pose_ready(self):
        if self.current_pose is None:
            return False, TriggerResponse(False, '当前还没有收到位姿，请先确认 /rtk_reverse_nav/pose 有数据。')
        if not self.pose_is_fresh():
            return False, TriggerResponse(False, '当前位姿已超时，请先检查 RTK 数据是否持续更新。')
        if not self.fix_valid:
            return False, TriggerResponse(False, 'RTK 当前无有效固定/可用解，拒绝进入自动流程。')
        return True, None

    def try_append_record_point(self, pose):
        if self.last_saved_point is None:
            self.recorded_path.append(PathPoint(pose.x, pose.y, pose.yaw))
            self.last_saved_point = PathPoint(pose.x, pose.y, pose.yaw)
            self.publish_recorded_path(self.recorded_path)
            self.count_pub.publish(Int32(data=len(self.recorded_path)))
            return

        dx = pose.x - self.last_saved_point.x
        dy = pose.y - self.last_saved_point.y
        dist = math.hypot(dx, dy)
        dyaw = abs(math.degrees(wrap_to_pi(pose.yaw - self.last_saved_point.yaw)))
        yaw_trigger = (self.record_yaw_spacing_deg > 0.0 and
                       dyaw >= self.record_yaw_spacing_deg and
                       dist >= self.record_yaw_min_distance)
        if dist >= self.record_point_spacing or yaw_trigger:
            self.recorded_path.append(PathPoint(pose.x, pose.y, pose.yaw))
            self.last_saved_point = PathPoint(pose.x, pose.y, pose.yaw)
            self.publish_recorded_path(self.recorded_path)
            self.count_pub.publish(Int32(data=len(self.recorded_path)))

    def build_path_msg(self, points):
        msg = Path()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        for p in points:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = p.x
            ps.pose.position.y = p.y
            q = tft.quaternion_from_euler(0, 0, p.yaw)
            ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = q
            msg.poses.append(ps)
        return msg

    def publish_recorded_path(self, points):
        self.path_pub.publish(self.build_path_msg(points))

    def publish_replay_path(self, points):
        self.replay_pub.publish(self.build_path_msg(points))

    def dedup_points(self, points, min_spacing):
        if not points:
            return []
        out = [PathPoint(points[0].x, points[0].y, points[0].yaw)]
        for p in points[1:]:
            dist = math.hypot(p.x - out[-1].x, p.y - out[-1].y)
            dyaw = abs(math.degrees(wrap_to_pi(p.yaw - out[-1].yaw)))
            if dist >= min_spacing or dyaw >= self.path_dedup_yaw_keep_deg:
                out.append(PathPoint(p.x, p.y, p.yaw))
        if len(points) >= 2:
            tail = points[-1]
            if math.hypot(tail.x - out[-1].x, tail.y - out[-1].y) > 1e-6:
                out.append(PathPoint(tail.x, tail.y, tail.yaw))
        return out

    def compute_line_work_yaw(self):
        """两点喷洒模式使用的固定作业航向。

        用户流程是：先记录终点，再正向开到起点，自动倒车时车辆从起点倒回终点。
        两点几何方向 start->end 是倒车运动方向，车辆车头方向应约等于该方向反向，
        即 path_yaw + pi。也可以通过 spray_line_yaw_source=recorded_start 使用起点记录 yaw。
        """
        if self.start_point is None or self.end_point is None:
            return 0.0
        if self.spray_line_yaw_source in ('recorded', 'recorded_start', 'start'):
            return self.start_point.yaw
        if self.spray_line_yaw_source in ('recorded_end', 'end'):
            return self.end_point.yaw
        path_yaw = math.atan2(self.end_point.y - self.start_point.y, self.end_point.x - self.start_point.x)
        if self.spray_line_yaw_source in ('geometry_forward', 'path'):
            return path_yaw
        # 默认 geometry_reverse：倒车沿 start->end 运动，车头朝向与运动方向相反。
        return wrap_to_pi(path_yaw + math.pi)

    def build_line_path(self):
        if self.start_point is None or self.end_point is None:
            return []
        if self.enable_spray_line_control:
            work_yaw = self.compute_line_work_yaw()
            return [
                PathPoint(self.start_point.x, self.start_point.y, work_yaw),
                PathPoint(self.end_point.x, self.end_point.y, work_yaw),
            ]
        return [
            PathPoint(self.start_point.x, self.start_point.y, self.start_point.yaw),
            PathPoint(self.end_point.x, self.end_point.y, self.end_point.yaw),
        ]

    def is_spray_line_mode(self):
        return self.tracking_mode == 'line' and self.enable_spray_line_control

    def distance_to_next_work_stop(self, segment_progress_s, dist_goal):
        """返回距离下一次停车/终点还有多远，用于停车前退出横差修正。"""
        distances = []
        if self.should_enable_pause() and self.next_pause_progress is not None:
            distances.append(max(0.0, self.next_pause_progress - segment_progress_s))
        if dist_goal is not None:
            distances.append(max(0.0, dist_goal))
        return min(distances) if distances else float('inf')

    def compute_spray_line_control(self, desired_yaw, pose_yaw, cross_track_error, dist_to_stop):
        """喷洒两点模式控制。

        横差只生成一个小的航向偏置角，不再直接作为角速度项。
        距离停车点越近，横差航向偏置越小，保证每次停车喷洒时车头航向基本一致。
        """
        cte_yaw = math.atan2(-cross_track_error, self.cross_track_lookahead_m)
        cte_yaw = clamp(cte_yaw, -self.max_cross_track_yaw_correction, self.max_cross_track_yaw_correction)
        align_ratio = 1.0
        if self.pre_stop_align_distance_m > 1e-6 and dist_to_stop < self.pre_stop_align_distance_m:
            align_ratio = clamp(dist_to_stop / self.pre_stop_align_distance_m, 0.0, 1.0)
            cte_yaw *= align_ratio
        corrected_yaw = wrap_to_pi(desired_yaw + cte_yaw)
        heading_error = wrap_to_pi(corrected_yaw - pose_yaw)
        if abs(heading_error) < self.heading_deadband:
            heading_error = 0.0
        angular_raw = self.k_heading * heading_error
        return angular_raw, heading_error, cte_yaw, align_ratio, corrected_yaw

    def build_path_mode_track(self):
        if len(self.recorded_path) < 2:
            self.replay_path = []
            return
        path_points = self.recorded_path
        if self.enable_path_dedup:
            path_points = self.dedup_points(path_points, max(0.02, self.path_dedup_spacing))
        self.replay_path = [PathPoint(p.x, p.y, p.yaw) for p in reversed(path_points)]

    def get_active_path(self):
        return self.line_path if self.tracking_mode == 'line' else self.replay_path

    def make_timestamped_path_file(self):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        path_dir = os.path.expanduser(self.path_dir)
        os.makedirs(path_dir, exist_ok=True)
        base = os.path.join(path_dir, '{}_{}'.format(self.path_prefix, timestamp))
        candidate = base + '.csv'
        index = 1
        while os.path.exists(candidate):
            candidate = '{}_{:02d}.csv'.format(base, index)
            index += 1
        return candidate

    def resolve_save_path_file(self):
        if self.path_file and str(self.path_file).strip():
            path_file = os.path.expanduser(str(self.path_file).strip())
            path_dir = os.path.dirname(path_file)
            if path_dir:
                os.makedirs(path_dir, exist_ok=True)
            return path_file
        return self.make_timestamped_path_file()

    def save_path_to_csv(self):
        save_file = self.resolve_save_path_file()
        active_path = self.get_active_path()
        with open(save_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['mode', self.tracking_mode])
            writer.writerow(['order', 'auto_reverse_execution_order'])
            writer.writerow(['created_time', datetime.now().strftime('%Y-%m-%d %H:%M:%S')])
            writer.writerow(['index', 'x', 'y', 'yaw_rad'])
            for i, p in enumerate(active_path):
                writer.writerow([i, '{:.6f}'.format(p.x), '{:.6f}'.format(p.y), '{:.6f}'.format(p.yaw)])
        self.last_saved_csv = save_file
        return save_file

    def load_path_from_csv(self, csv_file):
        csv_file = os.path.expanduser(str(csv_file).strip())
        if not csv_file:
            raise ValueError('导入文件路径为空。')
        if not os.path.exists(csv_file):
            raise IOError('导入文件不存在: {}'.format(csv_file))

        points = []
        header_map = None
        with open(csv_file, 'r', newline='') as f:
            reader = csv.reader(f)
            for row in reader:
                if not row:
                    continue
                row = [c.strip() for c in row]
                lower = [c.lower() for c in row]
                if lower[0] in ('mode', 'order', 'created_time'):
                    continue
                if 'x' in lower and 'y' in lower:
                    header_map = {name: idx for idx, name in enumerate(lower)}
                    continue

                try:
                    if header_map is not None:
                        x = float(row[header_map['x']])
                        y = float(row[header_map['y']])
                        yaw_idx = None
                        for key in ('yaw_rad', 'yaw', 'heading_rad'):
                            if key in header_map:
                                yaw_idx = header_map[key]
                                break
                        yaw = float(row[yaw_idx]) if yaw_idx is not None else 0.0
                    else:
                        # 兼容 index,x,y,yaw_rad 或 x,y,yaw_rad 两种格式。
                        if len(row) >= 4:
                            x = float(row[1])
                            y = float(row[2])
                            yaw = float(row[3])
                        elif len(row) >= 3:
                            x = float(row[0])
                            y = float(row[1])
                            yaw = float(row[2])
                        else:
                            continue
                    points.append(PathPoint(x, y, yaw))
                except Exception:
                    # 跳过无法解析的说明行/空行，不中断导入。
                    continue

        if len(points) < 2:
            raise ValueError('CSV 中有效轨迹点不足 2 个，无法导入。')
        return points

    def publish_zero(self):
        self.cmd_pub.publish(Twist())
        self.last_cmd_angular = 0.0
        self.last_cmd_time = rospy.Time.now()

    def publish_cmd(self, linear_x, angular_z):
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)
        self.last_cmd_angular = angular_z
        self.last_cmd_time = rospy.Time.now()

    def reset_pause_state(self):
        self.pause_until = None  # 兼容旧变量，不再作为主状态机使用。
        self.pause_state = 'cruise'
        self.pause_state_start_time = None
        self.next_pause_progress = None
        self.pause_progress_at_state_start = 0.0
        self.last_cmd_angular = 0.0
        self.last_cmd_time = None
        self.pause_hold_yaw = None
        self.pause_resume_start_time = None

    @staticmethod
    def smoothstep(x):
        x = max(0.0, min(1.0, x))
        return x * x * (3.0 - 2.0 * x)

    def compute_path_progress(self, active_path, segment_index, t):
        if len(active_path) < 2:
            return 0.0
        segment_index = max(0, min(segment_index, len(active_path) - 2))
        progress = 0.0
        for i in range(segment_index):
            progress += math.hypot(active_path[i + 1].x - active_path[i].x, active_path[i + 1].y - active_path[i].y)
        p0 = active_path[segment_index]
        p1 = active_path[segment_index + 1]
        progress += max(0.0, min(1.0, t)) * math.hypot(p1.x - p0.x, p1.y - p0.y)
        return progress

    def should_enable_pause(self):
        return (self.enable_segment_pause and
                self.pause_distance_m > 1e-6 and
                self.pause_duration_s >= 0.0)

    def rate_limit_angular(self, desired_angular, now, accel_limit):
        if not self.enable_angular_slew_in_pause or accel_limit <= 0.0:
            return desired_angular
        return self._slew_angular(desired_angular, now, accel_limit)

    def _slew_angular(self, desired_angular, now, accel_limit):
        if accel_limit <= 0.0:
            return desired_angular
        if self.last_cmd_time is None:
            self.last_cmd_time = now
            self.last_cmd_angular = desired_angular
            return desired_angular
        dt = max(1.0 / max(self.control_rate, 1.0), (now - self.last_cmd_time).to_sec())
        max_delta = accel_limit * dt
        delta = desired_angular - self.last_cmd_angular
        if delta > max_delta:
            desired_angular = self.last_cmd_angular + max_delta
        elif delta < -max_delta:
            desired_angular = self.last_cmd_angular - max_delta
        return desired_angular

    def limit_angular_command(self, desired_angular, linear_x, now, abs_limit=None, accel_limit=None, use_slew=True):
        # 角速度绝对限幅。
        limit = self.max_angular_speed
        if abs_limit is not None:
            limit = min(limit, max(0.0, abs_limit))
        # 低速时按最小转弯半径再次限幅，避免停车后刚起步时“低速 + 较大角速度”变成大转角。
        if self.enable_speed_based_angular_limit:
            speed = abs(linear_x)
            speed_limit = speed / max(0.30, self.min_turn_radius_m)
            limit = min(limit, speed_limit)
        desired_angular = clamp(desired_angular, -limit, limit)
        if use_slew and self.enable_global_angular_slew:
            desired_angular = self._slew_angular(desired_angular, now, self.max_angular_accel if accel_limit is None else accel_limit)
        return desired_angular

    def recent_average_yaw(self, window_s):
        if self.current_pose is None:
            return 0.0
        now_s = rospy.Time.now().to_sec()
        window_s = max(0.01, float(window_s))
        samples = [y for (t, y) in self.yaw_history if now_s - t <= window_s]
        if not samples:
            return self.current_pose.yaw
        sx = sum(math.cos(y) for y in samples)
        sy = sum(math.sin(y) for y in samples)
        return math.atan2(sy, sx)

    def hold_yaw_angular(self, pose, now, linear_x):
        if self.pause_hold_yaw is None:
            self.pause_hold_yaw = self.recent_average_yaw(self.pause_hold_yaw_average_s)
        heading_error_hold = wrap_to_pi(self.pause_hold_yaw - pose.yaw)
        desired = self.pause_heading_k * heading_error_hold
        return self.limit_angular_command(
            desired, linear_x, now,
            abs_limit=self.pause_heading_max_angular_speed,
            accel_limit=self.max_pause_angular_accel,
            use_slew=False,
        )

    def relay_bytes(self, text):
        return bytes.fromhex(text.replace('0x', '').replace(',', ' '))

    def score_relay_port(self, port):
        text = ' '.join([
            getattr(port, 'device', '') or '',
            getattr(port, 'description', '') or '',
            getattr(port, 'manufacturer', '') or '',
            getattr(port, 'hwid', '') or '',
        ]).lower()
        score = 0
        for kw in ('relay', 'usb serial', 'ch340', 'cp210', 'ftdi', 'wch', 'ttyusb', 'ttyacm'):
            if kw in text:
                score += 10
        if 'usb' in text:
            score += 5
        dev = (getattr(port, 'device', '') or '').lower()
        if 'ttyusb' in dev or 'ttyacm' in dev:
            score += 3
        return score

    def find_relay_port(self):
        if self.relay_port and self.relay_port.lower() != 'auto':
            return self.relay_port
        if list_ports is None:
            raise RuntimeError('未安装 pyserial，无法自动查找继电器串口。请执行: sudo apt install python3-serial')
        ports = list(list_ports.comports())
        if not ports:
            raise RuntimeError('没有检测到可用串口，请确认 USB 继电器已插入。')
        candidates = []
        for p in ports:
            text = ' '.join([p.device or '', p.description or '', p.manufacturer or '', p.hwid or '']).lower()
            if 'usb' in text or 'ttyusb' in text or 'ttyacm' in text or 'com' in (p.device or '').lower():
                candidates.append(p)
        if not candidates:
            candidates = ports
        candidates.sort(key=self.score_relay_port, reverse=True)
        return candidates[0].device

    def pulse_relay_worker(self, reason='auto'):
        if not self.enable_relay:
            return
        if serial is None:
            rospy.logwarn('继电器功能需要 pyserial，请安装: sudo apt install python3-serial')
            return
        with self.relay_lock:
            if self.relay_busy:
                rospy.logwarn('继电器正在动作，跳过本次触发。')
                return
            self.relay_busy = True
        port_name = ''
        ser = None
        try:
            port_name = self.find_relay_port()
            self.relay_last_port = port_name
            on_cmd = self.relay_bytes(self.relay_on_hex)
            off_cmd = self.relay_bytes(self.relay_off_hex)
            ser = serial.Serial(
                port=port_name,
                baudrate=self.relay_baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.relay_serial_timeout,
                write_timeout=self.relay_serial_timeout,
            )
            time.sleep(0.10)
            ser.write(off_cmd)
            ser.flush()
            time.sleep(0.10)
            ser.write(on_cmd)
            ser.flush()
            rospy.loginfo('继电器已闭合 %.2f s | port=%s | reason=%s', self.relay_pulse_seconds, port_name, reason)
            time.sleep(max(0.0, self.relay_pulse_seconds))
            ser.write(off_cmd)
            ser.flush()
            rospy.loginfo('继电器已断开 | port=%s', port_name)
        except Exception as e:
            rospy.logwarn('继电器动作失败: %s', e)
        finally:
            if ser is not None and ser.is_open:
                ser.close()
            with self.relay_lock:
                self.relay_busy = False

    def trigger_relay_async(self, reason='auto'):
        if not self.enable_relay:
            return
        t = threading.Thread(target=self.pulse_relay_worker, args=(reason,))
        t.daemon = True
        t.start()

    def stop_auto_with_reason(self, reason, fault_pause=False):
        self.auto_reverse = False
        self.fault_paused = bool(fault_pause)
        self.fault_reason = reason if fault_pause else ''
        self.reset_pause_state()
        self.publish_zero()
        if fault_pause:
            self.publish_status(reason + ' 已进入故障暂停：请人工把车调回原轨迹附近，然后点击【人工调整后继续倒车】。')
        else:
            self.publish_status(reason)

    def srv_pulse_relay(self, _req):
        if not self.enable_relay:
            return TriggerResponse(False, '继电器功能未启用。')
        self.trigger_relay_async('manual')
        return TriggerResponse(True, '已触发继电器闭合 {:.2f} s。'.format(self.relay_pulse_seconds))

    def srv_set_line_mode(self, _req):
        with self.lock:
            self.tracking_mode = 'line'
            if self.start_point is not None and self.end_point is not None:
                self.line_path = self.build_line_path()
                self.publish_replay_path(self.line_path)
            self.publish_status('已切换到两点打点直线倒车模式。')
            return TriggerResponse(True, '已切换到两点打点直线倒车模式。')

    def srv_set_path_mode(self, _req):
        with self.lock:
            self.tracking_mode = 'path'
            if self.imported_path_active and len(self.replay_path) >= 2:
                self.publish_replay_path(self.replay_path)
            elif len(self.recorded_path) >= 2:
                self.build_path_mode_track()
                self.publish_replay_path(self.replay_path)
            self.publish_status('已切换到备用多点轨迹复现模式。')
            return TriggerResponse(True, '已切换到备用多点轨迹复现模式。')

    def srv_record_end_and_start(self, _req):
        with self.lock:
            ok, resp = self.current_pose_ready()
            if not ok:
                return resp
            self.auto_reverse = False
            self.fault_paused = False
            self.fault_reason = ''
            self.reset_pause_state()
            self.recording = True
            self.recorded_path = []
            self.replay_path = []
            self.line_path = []
            self.imported_path_active = False
            self.last_imported_csv = ''
            self.start_point = None
            self.end_point = PathPoint(self.current_pose.x, self.current_pose.y, self.current_pose.yaw)
            self.last_saved_point = None
            self.try_append_record_point(self.end_point)
            self.publish_status('已记录终点，并开始录轨。请遥控车辆正向开到起点。当前模式: {}'.format(self.tracking_mode))
            return TriggerResponse(True, '已记录终点，并开始录轨。')

    def srv_record_start_and_stop(self, _req):
        with self.lock:
            ok, resp = self.current_pose_ready()
            if not ok:
                return resp
            if not self.recording:
                return TriggerResponse(False, '当前不在录轨状态，不能记录起点。')
            self.start_point = PathPoint(self.current_pose.x, self.current_pose.y, self.current_pose.yaw)
            if self.tracking_mode == 'line':
                # 两点模式强制只保留：终点 + 起点。
                # 后面 build_line_path 会生成倒车执行顺序：起点 -> 终点。
                self.recorded_path = []
                if self.end_point is not None:
                    self.recorded_path.append(PathPoint(self.end_point.x, self.end_point.y, self.end_point.yaw))
                self.recorded_path.append(PathPoint(self.start_point.x, self.start_point.y, self.start_point.yaw))
                self.last_saved_point = PathPoint(self.start_point.x, self.start_point.y, self.start_point.yaw)
                self.publish_recorded_path(self.recorded_path)
                self.count_pub.publish(Int32(data=len(self.recorded_path)))
            else:
                self.try_append_record_point(self.start_point)
            self.recording = False
            if len(self.recorded_path) < 2:
                return TriggerResponse(False, '录到的轨迹点不足 2 个，无法自动倒车。')
            self.line_path = self.build_line_path()
            self.build_path_mode_track()
            if self.tracking_mode == 'line':
                self.publish_replay_path(self.line_path)
            else:
                self.publish_replay_path(self.replay_path)
            saved_file = self.save_path_to_csv()
            self.publish_status('已记录起点并停止录轨，可启动自动倒车。轨迹已保存到 {}。当前模式: {}'.format(saved_file, self.tracking_mode))
            return TriggerResponse(True, '已记录起点并停止录轨，轨迹已保存到 {}'.format(saved_file))

    def align_start_ok(self, active_path):
        if not active_path or self.current_pose is None:
            return False, '当前没有可用轨迹。'
        start = active_path[0]
        dist = math.hypot(start.x - self.current_pose.x, start.y - self.current_pose.y)
        heading_err = abs(math.degrees(wrap_to_pi(start.yaw - self.current_pose.yaw)))
        if dist > self.startup_max_distance:
            return False, '当前车辆距离起点 {:.2f} m，超过允许启动距离 {:.2f} m。'.format(dist, self.startup_max_distance)
        if heading_err > self.startup_heading_tolerance_deg:
            return False, '当前车头与录制起点姿态相差 {:.1f} deg，请先人工摆正后再启动。'.format(heading_err)
        return True, ''

    def srv_start_auto_reverse(self, _req):
        with self.lock:
            ok, resp = self.current_pose_ready()
            if not ok:
                return resp
            active_path = self.get_active_path()
            if len(active_path) < 2:
                return TriggerResponse(False, '当前模式下可用轨迹点不足，无法开始自动倒车。')
            align_ok, reason = self.align_start_ok(active_path)
            if not align_ok:
                return TriggerResponse(False, reason)
            self.auto_reverse = True
            self.fault_paused = False
            self.fault_reason = ''
            self.recording = False
            self.segment_index = 0
            self.reset_pause_state()
            self.last_progress_s = 0.0
            if self.should_enable_pause():
                self.next_pause_progress = self.pause_distance_m
            self.publish_status('自动倒车已启动。当前模式: {} | 分段停车: {} | 间隔: {:.2f} m'.format(
                self.tracking_mode, '开启' if self.should_enable_pause() else '关闭', self.pause_distance_m))
            return TriggerResponse(True, '自动倒车已启动。')

    def find_best_resume_segment(self, pose, active_path):
        if len(active_path) < 2:
            return 0
        # 两点模式只有 1 段；多点备用模式优先从当前段向前找，避免倒退到已经走过的段。
        if self.tracking_mode == 'line':
            return 0
        return self.find_nearest_forward_segment(pose, active_path)

    def check_resume_ok(self, active_path):
        if len(active_path) < 2 or self.current_pose is None:
            return False, '当前没有可用轨迹，不能继续。', 0, 0.0, 0.0, 0.0
        best_i = self.find_best_resume_segment(self.current_pose, active_path)
        p0 = active_path[best_i]
        p1 = active_path[best_i + 1]
        t, raw_t, cte, _dist_next, _proj_x, _proj_y = self.segment_metrics(self.current_pose, p0, p1)
        seg_len = max(1e-6, math.hypot(p1.x - p0.x, p1.y - p0.y))
        margin_t = max(0.02, self.resume_projection_margin_m / seg_len)
        if raw_t < -margin_t:
            return False, '当前车在轨迹起点之前过多，投影进度 {:.2f}，请再调整到原轨迹范围内。'.format(raw_t), best_i, t, raw_t, cte
        if raw_t > 1.0 + margin_t and best_i >= len(active_path) - 2:
            return False, '当前车已经越过轨迹终点附近，投影进度 {:.2f}，不建议继续自动倒车。'.format(raw_t), best_i, t, raw_t, cte
        if abs(cte) > self.resume_max_cross_track_error:
            return False, '当前横向误差 {:.2f} m，超过继续启动允许值 {:.2f} m，请人工再靠近原轨迹。'.format(cte, self.resume_max_cross_track_error), best_i, t, raw_t, cte
        desired_yaw = interp_angle(p0.yaw, p1.yaw, t)
        heading_err_deg = abs(math.degrees(wrap_to_pi(desired_yaw - self.current_pose.yaw)))
        if heading_err_deg > self.resume_heading_tolerance_deg:
            return False, '当前航向误差 {:.1f} deg，超过继续启动允许值 {:.1f} deg，请人工摆正车头。'.format(heading_err_deg, self.resume_heading_tolerance_deg), best_i, t, raw_t, cte
        return True, '', best_i, t, raw_t, cte

    def srv_resume_auto_reverse(self, _req):
        with self.lock:
            ok, resp = self.current_pose_ready()
            if not ok:
                return resp
            active_path = self.get_active_path()
            if len(active_path) < 2:
                return TriggerResponse(False, '当前模式下可用轨迹点不足，无法继续自动倒车。')
            resume_ok, reason, best_i, t, raw_t, cte = self.check_resume_ok(active_path)
            if not resume_ok:
                return TriggerResponse(False, reason)
            self.segment_index = best_i
            self.auto_reverse = True
            self.fault_paused = False
            self.fault_reason = ''
            self.recording = False
            self.reset_pause_state()
            self.last_progress_s = self.compute_path_progress(active_path, self.segment_index, t)
            if self.should_enable_pause():
                self.next_pause_progress = self.last_progress_s + self.pause_distance_m
            self.publish_status('人工调整后已继续自动倒车 | 模式: {} | 接入段号: {} | 进度: {:.2f} | 横差: {:.2f} m'.format(
                self.tracking_mode, self.segment_index, self.last_progress_s, cte))
            return TriggerResponse(True, '已在原轨迹附近继续自动倒车。')

    def srv_stop_auto(self, _req):
        with self.lock:
            self.auto_reverse = False
            self.fault_paused = False
            self.fault_reason = ''
            self.recording = False
            self.reset_pause_state()
            self.publish_zero()
            self.publish_status('已停止自动倒车。')
            return TriggerResponse(True, '已停止自动倒车。')

    def srv_clear_path(self, _req):
        with self.lock:
            self.auto_reverse = False
            self.fault_paused = False
            self.fault_reason = ''
            self.recording = False
            self.reset_pause_state()
            self.recorded_path = []
            self.replay_path = []
            self.line_path = []
            self.imported_path_active = False
            self.last_imported_csv = ''
            self.end_point = None
            self.start_point = None
            self.last_saved_point = None
            self.segment_index = 0
            self.last_progress_s = 0.0
            self.publish_zero()
            self.publish_recorded_path([])
            self.publish_replay_path([])
            self.publish_status('已清空轨迹。')
            return TriggerResponse(True, '已清空轨迹。')

    def srv_save_path(self, _req):
        with self.lock:
            if len(self.get_active_path()) < 2:
                return TriggerResponse(False, '当前模式下轨迹点不足，无法保存。')
            saved_file = self.save_path_to_csv()
            self.publish_status('轨迹已保存到 {}'.format(saved_file))
            return TriggerResponse(True, '轨迹已保存到 {}'.format(saved_file))

    def srv_import_path(self, _req):
        with self.lock:
            csv_file = rospy.get_param('~import_path_file', self.import_path_file)
            try:
                imported_points = self.load_path_from_csv(csv_file)
            except Exception as e:
                return TriggerResponse(False, '导入轨迹失败: {}'.format(e))

            self.auto_reverse = False
            self.fault_paused = False
            self.fault_reason = ''
            self.recording = False
            self.reset_pause_state()
            self.segment_index = 0
            self.last_progress_s = 0.0
            self.tracking_mode = 'path'
            self.import_path_file = os.path.expanduser(str(csv_file).strip())
            self.last_imported_csv = self.import_path_file

            # 导入 CSV 后，按 CSV 行顺序作为倒车执行轨迹；不再二次反转。
            self.replay_path = [PathPoint(p.x, p.y, p.yaw) for p in imported_points]
            self.recorded_path = [PathPoint(p.x, p.y, p.yaw) for p in imported_points]
            self.line_path = []
            self.start_point = self.replay_path[0]
            self.end_point = self.replay_path[-1]
            self.last_saved_point = None
            self.imported_path_active = True
            self.publish_recorded_path(self.recorded_path)
            self.publish_replay_path(self.replay_path)
            self.publish_zero()
            msg = '已导入 CSV 轨迹: {} | 点数: {}。请把车停到导入轨迹第一个点附近后启动自动倒车。'.format(self.import_path_file, len(self.replay_path))
            self.publish_status(msg)
            return TriggerResponse(True, msg)

    def segment_metrics(self, pose, p0, p1):
        vx = p1.x - p0.x
        vy = p1.y - p0.y
        seg_len = math.hypot(vx, vy)
        if seg_len < 1e-9:
            return 0.0, 0.0, 0.0, 0.0, p0.x, p0.y
        ux = vx / seg_len
        uy = vy / seg_len
        wx = pose.x - p0.x
        wy = pose.y - p0.y
        along = wx * ux + wy * uy
        raw_t = along / seg_len
        t = max(0.0, min(1.0, raw_t))
        proj_x = p0.x + t * vx
        proj_y = p0.y + t * vy
        dx = pose.x - proj_x
        dy = pose.y - proj_y
        cte = -uy * dx + ux * dy
        remain = math.hypot(p1.x - pose.x, p1.y - pose.y)
        return t, raw_t, cte, remain, proj_x, proj_y

    def find_nearest_forward_segment(self, pose, active_path):
        if len(active_path) < 2:
            return self.segment_index
        start_i = max(0, min(self.segment_index, len(active_path) - 2))
        end_i = min(len(active_path) - 2, start_i + max(1, self.segment_search_ahead))
        best_i = start_i
        best_dist = float('inf')
        for i in range(start_i, end_i + 1):
            p0 = active_path[i]
            p1 = active_path[i + 1]
            _t, raw_t, _cte, _remain, proj_x, proj_y = self.segment_metrics(pose, p0, p1)
            if raw_t < -0.25 or raw_t > 1.35:
                continue
            dist = math.hypot(pose.x - proj_x, pose.y - proj_y) + 0.01 * (i - start_i)
            if dist < best_dist:
                best_dist = dist
                best_i = i
        return best_i

    def update_segment_index(self, pose, active_path):
        if len(active_path) < 2:
            return

        while self.segment_index < len(active_path) - 2:
            p0 = active_path[self.segment_index]
            p1 = active_path[self.segment_index + 1]
            _t, raw_t, _cte, dist_to_next, _proj_x, _proj_y = self.segment_metrics(pose, p0, p1)
            dist_to_current = math.hypot(p0.x - pose.x, p0.y - pose.y)
            seg_len = math.hypot(p1.x - p0.x, p1.y - p0.y)
            effective_reach_tol = min(
                self.waypoint_reach_tolerance,
                max(self.waypoint_min_reach_tolerance, seg_len * self.waypoint_tolerance_segment_ratio)
            )

            reached_next = dist_to_next <= effective_reach_tol
            passed_segment = (self.enable_projection_segment_advance and
                              raw_t >= self.segment_advance_ratio and
                              dist_to_next <= dist_to_current + effective_reach_tol)

            if reached_next or passed_segment:
                self.segment_index += 1
                continue
            break

        if self.enable_nearest_segment_recovery and self.segment_index < len(active_path) - 2:
            old_i = self.segment_index
            new_i = self.find_nearest_forward_segment(pose, active_path)
            if new_i > old_i:
                p0_old = active_path[old_i]
                p1_old = active_path[old_i + 1]
                _t0, _raw0, _cte0, _rem0, old_proj_x, old_proj_y = self.segment_metrics(pose, p0_old, p1_old)
                old_dist = math.hypot(pose.x - old_proj_x, pose.y - old_proj_y)
                p0_new = active_path[new_i]
                p1_new = active_path[new_i + 1]
                _t1, _raw1, _cte1, _rem1, new_proj_x, new_proj_y = self.segment_metrics(pose, p0_new, p1_new)
                new_dist = math.hypot(pose.x - new_proj_x, pose.y - new_proj_y)
                if old_dist - new_dist >= self.segment_recovery_min_improve:
                    self.segment_index = new_i
                    rospy.loginfo('轨迹段号自动恢复: {} -> {} | old_dist={:.2f} m new_dist={:.2f} m'.format(old_i, new_i, old_dist, new_dist))

    def control_loop(self, _event):
        with self.lock:
            if not self.auto_reverse:
                if self.publish_zero_on_idle:
                    self.publish_zero()
                return
            if not self.pose_is_fresh():
                self.stop_auto_with_reason('位姿已超时，自动倒车已强制停止。')
                return
            if not self.fix_valid:
                self.stop_auto_with_reason('RTK 当前无有效解，自动倒车已强制停止。')
                return

            active_path = self.get_active_path()
            if len(active_path) < 2:
                self.stop_auto_with_reason('当前模式下轨迹不足，自动倒车已停止。')
                return

            goal = active_path[-1]
            pose = self.current_pose
            dist_goal = math.hypot(goal.x - pose.x, goal.y - pose.y)
            goal_yaw_err = abs(math.degrees(wrap_to_pi(goal.yaw - pose.yaw)))
            if dist_goal <= self.goal_tolerance and (not self.require_goal_yaw or goal_yaw_err <= self.goal_yaw_tolerance_deg):
                self.stop_auto_with_reason('已到达终点，自动倒车结束。')
                return

            now = rospy.Time.now()

            # 保持原版轨迹控制：先更新段号、算横差/航差/角速度，不改变两点模式和多点模式的主算法。
            self.update_segment_index(pose, active_path)
            p0 = active_path[self.segment_index]
            p1 = active_path[self.segment_index + 1]
            t, raw_t, cross_track_error, dist_seg_end, proj_x, proj_y = self.segment_metrics(pose, p0, p1)
            if abs(cross_track_error) > self.max_cross_track_error:
                self.stop_auto_with_reason('横向误差过大 {:.2f} m，已停止自动倒车。'.format(cross_track_error), fault_pause=True)
                return

            desired_yaw = interp_angle(p0.yaw, p1.yaw, t)
            segment_progress_s = self.compute_path_progress(active_path, self.segment_index, t)
            self.last_progress_s = segment_progress_s
            dist_to_work_stop = self.distance_to_next_work_stop(segment_progress_s, dist_goal)
            cte_yaw_correction = 0.0
            pre_stop_align_ratio = 1.0
            corrected_yaw = desired_yaw

            if self.is_spray_line_mode():
                base_angular_z_raw, heading_error, cte_yaw_correction, pre_stop_align_ratio, corrected_yaw = self.compute_spray_line_control(
                    desired_yaw, pose.yaw, cross_track_error, dist_to_work_stop)
            else:
                heading_error = wrap_to_pi(desired_yaw - pose.yaw)
                if abs(heading_error) < self.heading_deadband:
                    heading_error = 0.0
                base_angular_z_raw = self.k_heading * heading_error - self.k_cross_track * cross_track_error

            if abs(math.degrees(heading_error)) > self.runtime_heading_limit_deg:
                self.stop_auto_with_reason('航向误差过大 {:.1f} deg，已停止自动倒车。'.format(math.degrees(heading_error)), fault_pause=True)
                return

            base_linear_x = -self.reverse_speed
            base_angular_z = self.limit_angular_command(base_angular_z_raw, base_linear_x, now, use_slew=False)

            tp = PointStamped()
            tp.header.stamp = rospy.Time.now()
            tp.header.frame_id = self.frame_id
            tp.point.x = p1.x
            tp.point.y = p1.y
            self.target_pub.publish(tp)

            # 外层停车状态机：只调制输出，不改原轨迹控制。下一次停车点在“起步完成后”重新累计。
            if not self.should_enable_pause() or dist_goal <= self.pause_skip_near_goal_m:
                self.pause_state = 'cruise'
                cruise_angular_z = self.limit_angular_command(base_angular_z, base_linear_x, now)
                self.publish_cmd(base_linear_x, cruise_angular_z)
                self.publish_status(
                    '自动倒车中 | 模式: {} | 录点: {} | 段号: {} | 段进度: {:.2f} | 段终点距: {:.2f} m | 终点距: {:.2f} m | 横差: {:.2f} m | 航差: {:.1f} deg | 横差航向修正: {:.1f} deg | 停车对齐比例: {:.0f}%'.format(
                        self.tracking_mode, len(self.recorded_path), self.segment_index, raw_t,
                        dist_seg_end, dist_goal, cross_track_error, math.degrees(heading_error),
                        math.degrees(cte_yaw_correction), pre_stop_align_ratio * 100.0))
                return

            if self.pause_state == 'cruise':
                if self.next_pause_progress is None:
                    self.next_pause_progress = segment_progress_s + self.pause_distance_m
                if segment_progress_s >= self.next_pause_progress:
                    self.pause_state = 'brake'
                    self.pause_state_start_time = now
                    self.pause_progress_at_state_start = segment_progress_s
                    # 记录停车前角度。后续刹车、保持、重新起步都围绕这个角度过渡，避免每次停车后重新大角度找轨迹。
                    self.pause_hold_yaw = self.recent_average_yaw(self.pause_hold_yaw_average_s) if self.hold_yaw_during_pause else None
                    self.publish_status('达到分段停车点，保存停车前角度 {:.1f} deg，开始减速停车。当前进度 {:.2f} m。'.format(
                        math.degrees(self.pause_hold_yaw) if self.pause_hold_yaw is not None else math.degrees(pose.yaw), segment_progress_s))
                else:
                    cruise_angular_z = self.limit_angular_command(base_angular_z, base_linear_x, now)
                    self.publish_cmd(base_linear_x, cruise_angular_z)
                    self.publish_status(
                        '自动倒车中 | 模式: {} | 录点: {} | 段号: {} | 段进度: {:.2f} | 段终点距: {:.2f} m | 终点距: {:.2f} m | 横差: {:.2f} m | 航差: {:.1f} deg | 横差航向修正: {:.1f} deg | 停车对齐比例: {:.0f}% | 下次停车距: {:.2f} m'.format(
                            self.tracking_mode, len(self.recorded_path), self.segment_index, raw_t,
                            dist_seg_end, dist_goal, cross_track_error, math.degrees(heading_error),
                            math.degrees(cte_yaw_correction), pre_stop_align_ratio * 100.0,
                            max(0.0, self.next_pause_progress - segment_progress_s)))
                    return

            if self.pause_state == 'brake':
                elapsed = (now - self.pause_state_start_time).to_sec() if self.pause_state_start_time else 0.0
                ratio = 1.0 - self.smoothstep(elapsed / self.pause_smooth_stop_s)
                if elapsed >= self.pause_smooth_stop_s:
                    self.pause_state = 'hold'
                    self.pause_state_start_time = now
                    self.publish_zero()
                    self.trigger_relay_async('pause')
                    self.publish_status('已停稳，保持停车 {:.1f} s，继电器独立闭合 {:.1f} s。'.format(
                        self.pause_duration_s, self.relay_pulse_seconds if self.enable_relay else 0.0))
                    return
                # 减速阶段不再继续追横向误差；优先围绕停车前角度把角速度压小。
                linear_x = base_linear_x * ratio
                if self.hold_yaw_during_pause:
                    angular_z = self.hold_yaw_angular(pose, now, linear_x) * ratio
                else:
                    angular_z = base_angular_z * ratio
                angular_z = self.limit_angular_command(angular_z, linear_x, now,
                                                      abs_limit=self.pause_resume_max_angular_speed,
                                                      accel_limit=self.max_pause_angular_accel,
                                                      use_slew=True)
                self.publish_cmd(linear_x, angular_z)
                hold_err_deg = math.degrees(wrap_to_pi((self.pause_hold_yaw if self.pause_hold_yaw is not None else pose.yaw) - pose.yaw))
                self.publish_status('减速停车中 | 保持停车前角度 | 速度比例 {:.2f} | 横差: {:.2f} m | 停车角误差: {:.1f} deg'.format(
                    ratio, cross_track_error, hold_err_deg))
                return

            if self.pause_state == 'hold':
                elapsed = (now - self.pause_state_start_time).to_sec() if self.pause_state_start_time else 0.0
                if elapsed >= self.pause_duration_s:
                    self.pause_state = 'start'
                    self.pause_state_start_time = now
                    self.pause_resume_start_time = now
                    self.last_cmd_angular = 0.0
                    self.last_cmd_time = now
                    self.publish_status('停车保持结束，开始按停车前角度柔顺起步。')
                    return
                # 停车保持阶段必须 linear=0 且 angular=0，避免底盘进入原地自转/原地转向模式。
                self.publish_zero()
                self.publish_status('分段停车保持中，剩余 {:.1f} s | 停车时不输出角速度'.format(
                    max(0.0, self.pause_duration_s - elapsed)))
                return

            if self.pause_state == 'start':
                elapsed = (now - self.pause_state_start_time).to_sec() if self.pause_state_start_time else 0.0
                # 线速度按 pause_smooth_start_s 柔顺恢复；控制恢复总时间由“先直行 + 横差渐进恢复”决定。
                linear_ratio = self.smoothstep(elapsed / self.pause_smooth_start_s)
                linear_x = base_linear_x * linear_ratio

                straight_time = max(0.0, self.resume_straight_time_s)
                ramp_time = max(0.01, self.resume_cross_track_ramp_s)
                recover_total_s = max(self.pause_smooth_start_s, straight_time + ramp_time)

                hold_angular_z = self.hold_yaw_angular(pose, now, linear_x) if self.hold_yaw_during_pause else 0.0
                if elapsed < straight_time and self.resume_disable_cross_track_first:
                    # 起步初期：只保持停车前角度，不启用横向误差修正。
                    angular_z = hold_angular_z
                    recover_ratio = 0.0
                else:
                    recover_ratio = self.smoothstep((elapsed - straight_time) / ramp_time)
                    # 从“停车前角度保持”平滑过渡到“正常两点/轨迹控制”。
                    angular_z = (1.0 - recover_ratio) * hold_angular_z + recover_ratio * base_angular_z

                angular_z = self.limit_angular_command(
                    angular_z, linear_x, now,
                    abs_limit=self.pause_resume_max_angular_speed,
                    accel_limit=self.max_pause_angular_accel,
                    use_slew=True,
                )

                if elapsed >= recover_total_s:
                    # 起步和横差恢复完成后，以当前实际进度重新累计下一次停车距离。
                    self.pause_state = 'cruise'
                    self.next_pause_progress = segment_progress_s + self.pause_distance_m
                    self.pause_hold_yaw = None
                    base_angular_z = self.limit_angular_command(base_angular_z, base_linear_x, now)
                    self.publish_cmd(base_linear_x, base_angular_z)
                    self.publish_status('柔顺起步完成，横向纠偏已渐进恢复。下一次停车目标进度 {:.2f} m。'.format(self.next_pause_progress))
                    return

                self.publish_cmd(linear_x, angular_z)
                hold_err_deg = math.degrees(wrap_to_pi((self.pause_hold_yaw if self.pause_hold_yaw is not None else pose.yaw) - pose.yaw))
                self.publish_status('按停车前角度柔顺起步中 | 线速 {:.0f}% | 横差恢复 {:.0f}% | 横差: {:.2f} m | 停车角误差: {:.1f} deg'.format(
                    linear_ratio * 100.0, recover_ratio * 100.0, cross_track_error, hold_err_deg))
                return

            # 异常状态兜底：回到巡航，不停车。
            self.pause_state = 'cruise'
            cruise_angular_z = self.limit_angular_command(base_angular_z, base_linear_x, now)
            self.publish_cmd(base_linear_x, cruise_angular_z)


if __name__ == '__main__':
    try:
        ReverseTrackNavNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
