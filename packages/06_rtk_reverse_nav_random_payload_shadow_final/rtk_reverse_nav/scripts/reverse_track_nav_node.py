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
        self.tracking_mode = rospy.get_param('~tracking_mode', 'path').strip().lower()

        self.record_point_spacing = rospy.get_param('~record_point_spacing', 0.20)
        self.record_yaw_spacing_deg = rospy.get_param('~record_yaw_spacing_deg', 4.0)
        self.reverse_speed = abs(rospy.get_param('~reverse_speed', 0.15))
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 0.30)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.20)
        # 终点精修：最后一段降低速度、跳过分段停车，并可要求横差达标后再结束。
        self.final_slowdown_distance = abs(float(rospy.get_param('~final_slowdown_distance', 0.0)))
        self.final_reverse_speed = abs(float(rospy.get_param('~final_reverse_speed', 0.0)))
        self.final_cross_track_tolerance = abs(float(rospy.get_param('~final_cross_track_tolerance', 0.0)))
        self.final_require_cross_track = rospy.get_param('~final_require_cross_track', False)
        self.final_goal_overrun_tolerance = abs(float(rospy.get_param('~final_goal_overrun_tolerance', 0.03)))
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
        # 展开装置/长悬臂补偿：不改变原来的横差/航差控制，只叠加一个很小的前馈角速度。
        # payload_side 可取 center/left/right/manual；装置左右位置变化时不用改代码，只改 yaml。
        self.payload_side = str(rospy.get_param('~payload_side', 'center')).strip().lower()
        self.payload_bias_abs = abs(float(rospy.get_param('~payload_bias_abs', 0.0)))
        self.payload_bias_sign = 1.0 if float(rospy.get_param('~payload_bias_sign', 1.0)) >= 0.0 else -1.0
        self.payload_steering_bias = float(rospy.get_param('~payload_steering_bias', 0.0))
        self.payload_bias_max_cross_track = abs(float(rospy.get_param('~payload_bias_max_cross_track', 0.30)))
        self.payload_bias_max_heading_deg = abs(float(rospy.get_param('~payload_bias_max_heading_deg', 8.0)))
        self.payload_deployed_max_angular_speed = abs(float(rospy.get_param('~payload_deployed_max_angular_speed', 0.0)))
        self.max_cross_track_error = rospy.get_param('~max_cross_track_error', 0.80)
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
        self.pause_distance_m = rospy.get_param('~pause_distance_m', 0.50)
        self.pause_duration_s = rospy.get_param('~pause_duration_s', 3.0)
        self.pause_smooth_stop_s = max(0.01, rospy.get_param('~pause_smooth_stop_s', 0.25))
        self.pause_smooth_start_s = max(0.01, rospy.get_param('~pause_smooth_start_s', 0.40))
        self.pause_skip_near_goal_m = rospy.get_param('~pause_skip_near_goal_m', 0.35)
        self.pause_start_angular_limit = rospy.get_param('~pause_start_angular_limit', 0.12)
        # 起步/减速时的线速度死区保护：线速度过低时强制 angular=0，避免角速度型底盘原地自转。
        self.zero_angular_below_min_linear = rospy.get_param('~zero_angular_below_min_linear', True)
        self.min_linear_for_angular = abs(rospy.get_param('~min_linear_for_angular', 0.03))
        # 影子转向模式：停车时真实输出 angular=0，内部保留停车前的修正角速度，起步时平滑接回。
        self.pause_shadow_steering = rospy.get_param('~pause_shadow_steering', True)
        self.post_start_slew_s = rospy.get_param('~post_start_slew_s', 0.80)
        self.enable_angular_slew_in_pause = rospy.get_param('~enable_angular_slew_in_pause', True)
        self.max_pause_angular_accel = rospy.get_param('~max_pause_angular_accel', 0.12)

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
        self.pause_shadow_angular = 0.0
        self.last_tracking_angular = 0.0
        self.post_start_slew_until = None
        self.last_cmd_angular = 0.0
        self.last_cmd_time = None
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
            if self.recording and self.fix_valid:
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

    def build_line_path(self):
        if self.start_point is None or self.end_point is None:
            return []
        return [
            PathPoint(self.start_point.x, self.start_point.y, self.start_point.yaw),
            PathPoint(self.end_point.x, self.end_point.y, self.end_point.yaw),
        ]

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

    def get_payload_bias(self, cross_track_error, heading_error):
        """
        当前工况下装置会在长横杆左右随机切换，软件不猜左右方向。
        因此关闭固定 left/right 偏载前馈补偿，只依靠原轨迹控制、影子停车、限速限角和终点精修。
        """
        return 0.0

    def active_max_angular_speed(self):
        return self.max_angular_speed

    def publish_zero(self):
        self.cmd_pub.publish(Twist())
        self.last_cmd_angular = 0.0
        self.last_cmd_time = rospy.Time.now()

    def publish_cmd(self, linear_x, angular_z):
        # 对角速度型底盘做安全保护：当线速度仍处在底盘死区附近时，
        # 不向底盘输出 angular，避免恢复导航初期出现 linear≈0 但 angular≠0 的原地自转。
        if self.zero_angular_below_min_linear and abs(linear_x) < self.min_linear_for_angular:
            angular_z = 0.0

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
        self.pause_shadow_angular = 0.0
        self.last_tracking_angular = 0.0
        self.post_start_slew_until = None
        self.last_cmd_angular = 0.0
        self.last_cmd_time = None

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

    def apply_post_start_slew(self, base_angular_z, now):
        """起步完成后的短时间内继续限制角速度变化，避免从起步限制瞬间跳回全量控制。"""
        if self.post_start_slew_until is None:
            return base_angular_z
        if now >= self.post_start_slew_until:
            self.post_start_slew_until = None
            return base_angular_z
        return self.rate_limit_angular(base_angular_z, now, self.max_pause_angular_accel)

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

    def stop_auto_with_reason(self, reason):
        self.auto_reverse = False
        self.reset_pause_state()
        self.publish_zero()
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
            self.publish_status('已切换到两点直线复现模式。')
            return TriggerResponse(True, '已切换到两点直线复现模式。')

    def srv_set_path_mode(self, _req):
        with self.lock:
            self.tracking_mode = 'path'
            if self.imported_path_active and len(self.replay_path) >= 2:
                self.publish_replay_path(self.replay_path)
            elif len(self.recorded_path) >= 2:
                self.build_path_mode_track()
                self.publish_replay_path(self.replay_path)
            self.publish_status('已切换到多点轨迹复现模式。')
            return TriggerResponse(True, '已切换到多点轨迹复现模式。')

    def srv_record_end_and_start(self, _req):
        with self.lock:
            ok, resp = self.current_pose_ready()
            if not ok:
                return resp
            self.auto_reverse = False
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
            self.recording = False
            self.segment_index = 0
            self.reset_pause_state()
            self.last_progress_s = 0.0
            if self.should_enable_pause():
                self.next_pause_progress = self.pause_distance_m
            self.publish_status('自动倒车已启动。当前模式: {} | 分段停车: {} | 间隔: {:.2f} m'.format(
                self.tracking_mode, '开启' if self.should_enable_pause() else '关闭', self.pause_distance_m))
            return TriggerResponse(True, '自动倒车已启动。')

    def srv_stop_auto(self, _req):
        with self.lock:
            self.auto_reverse = False
            self.recording = False
            self.reset_pause_state()
            self.publish_zero()
            self.publish_status('已停止自动倒车。')
            return TriggerResponse(True, '已停止自动倒车。')

    def srv_clear_path(self, _req):
        with self.lock:
            self.auto_reverse = False
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

            now = rospy.Time.now()

            # 保持原版轨迹控制：先更新段号、算横差/航差/角速度，不改变两点模式和多点模式的主算法。
            self.update_segment_index(pose, active_path)
            p0 = active_path[self.segment_index]
            p1 = active_path[self.segment_index + 1]
            t, raw_t, cross_track_error, dist_seg_end, proj_x, proj_y = self.segment_metrics(pose, p0, p1)
            if abs(cross_track_error) > self.max_cross_track_error:
                self.stop_auto_with_reason('横向误差过大 {:.2f} m，已停止自动倒车。'.format(cross_track_error))
                return

            desired_yaw = interp_angle(p0.yaw, p1.yaw, t)
            heading_error = wrap_to_pi(desired_yaw - pose.yaw)
            if abs(math.degrees(heading_error)) > self.runtime_heading_limit_deg:
                self.stop_auto_with_reason('航向误差过大 {:.1f} deg，已停止自动倒车。'.format(math.degrees(heading_error)))
                return

            payload_bias = self.get_payload_bias(cross_track_error, heading_error)
            base_angular_z = self.k_heading * heading_error - self.k_cross_track * cross_track_error + payload_bias
            active_max_w = self.active_max_angular_speed()
            base_angular_z = max(-active_max_w, min(active_max_w, base_angular_z))

            # 终点精修：最后一段降低速度，给展开装置状态下的小横差更多连续修正时间。
            in_final_approach = self.final_slowdown_distance > 1e-6 and dist_goal <= self.final_slowdown_distance
            speed_cmd = self.reverse_speed
            if in_final_approach and self.final_reverse_speed > 1e-6:
                speed_cmd = min(self.reverse_speed, self.final_reverse_speed)
            base_linear_x = -speed_cmd

            # 到达终点判断放在横差计算之后。若启用 final_require_cross_track，则同时要求终点横差达标。
            final_cte_ok = (self.final_cross_track_tolerance <= 1e-9 or
                            abs(cross_track_error) <= self.final_cross_track_tolerance)
            yaw_ok = (not self.require_goal_yaw or goal_yaw_err <= self.goal_yaw_tolerance_deg)
            if dist_goal <= self.goal_tolerance and yaw_ok:
                if (not self.final_require_cross_track) or final_cte_ok:
                    self.stop_auto_with_reason('已到达终点，自动倒车结束。终点横差 {:.2f} m。'.format(cross_track_error))
                    return
                if dist_goal <= max(0.02, self.final_goal_overrun_tolerance):
                    self.stop_auto_with_reason('已到达终点附近，横差 {:.2f} m 未完全满足精修阈值，保护停止。'.format(cross_track_error))
                    return
            # 正常跟踪算法保持原样；这里额外保存影子角速度，停车保持时不发给底盘，起步时用于平滑接回。
            if self.pause_state in ('cruise', 'brake'):
                self.last_tracking_angular = base_angular_z
            segment_progress_s = self.compute_path_progress(active_path, self.segment_index, t)
            self.last_progress_s = segment_progress_s

            tp = PointStamped()
            tp.header.stamp = rospy.Time.now()
            tp.header.frame_id = self.frame_id
            tp.point.x = p1.x
            tp.point.y = p1.y
            self.target_pub.publish(tp)

            # 外层停车状态机：只调制输出，不改原轨迹控制。下一次停车点在“起步完成后”重新累计。
            if not self.should_enable_pause() or dist_goal <= self.pause_skip_near_goal_m or in_final_approach:
                self.pause_state = 'cruise'
                angular_out = self.apply_post_start_slew(base_angular_z, now)
                self.publish_cmd(base_linear_x, angular_out)
                self.publish_status(
                    '自动倒车中 | 模式: {} | 录点: {} | 段号: {} | 段进度: {:.2f} | 段终点距: {:.2f} m | 终点距: {:.2f} m | 横差: {:.2f} m | 航差: {:.1f} deg'.format(
                        self.tracking_mode, len(self.recorded_path), self.segment_index, raw_t,
                        dist_seg_end, dist_goal, cross_track_error, math.degrees(heading_error)))
                return

            if self.pause_state == 'cruise':
                if self.next_pause_progress is None:
                    self.next_pause_progress = segment_progress_s + self.pause_distance_m
                if segment_progress_s >= self.next_pause_progress:
                    self.pause_state = 'brake'
                    self.pause_state_start_time = now
                    self.pause_progress_at_state_start = segment_progress_s
                    self.pause_shadow_angular = self.last_tracking_angular if self.pause_shadow_steering else 0.0
                    self.publish_status('达到分段停车点，开始影子模式减速停车。当前进度 {:.2f} m，保存转向修正 {:.3f} rad/s。'.format(segment_progress_s, self.pause_shadow_angular))
                else:
                    angular_out = self.apply_post_start_slew(base_angular_z, now)
                    self.publish_cmd(base_linear_x, angular_out)
                    self.publish_status(
                        '自动倒车中 | 模式: {} | 录点: {} | 段号: {} | 段进度: {:.2f} | 段终点距: {:.2f} m | 终点距: {:.2f} m | 横差: {:.2f} m | 航差: {:.1f} deg | 下次停车距: {:.2f} m'.format(
                            self.tracking_mode, len(self.recorded_path), self.segment_index, raw_t,
                            dist_seg_end, dist_goal, cross_track_error, math.degrees(heading_error),
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
                # 减速阶段线速度和角速度一起衰减，避免低速大打轮。
                linear_x = base_linear_x * ratio
                angular_z = base_angular_z * ratio
                angular_z = self.rate_limit_angular(angular_z, now, self.max_pause_angular_accel)
                self.publish_cmd(linear_x, angular_z)
                self.publish_status('减速停车中 | 速度比例 {:.2f} | 模式: {} | 横差: {:.2f} m | 航差: {:.1f} deg'.format(
                    ratio, self.tracking_mode, cross_track_error, math.degrees(heading_error)))
                return

            if self.pause_state == 'hold':
                elapsed = (now - self.pause_state_start_time).to_sec() if self.pause_state_start_time else 0.0
                if elapsed >= self.pause_duration_s:
                    self.pause_state = 'start'
                    self.pause_state_start_time = now
                    self.last_cmd_angular = 0.0
                    self.last_cmd_time = now
                    self.publish_status('停车保持结束，开始柔顺起步。')
                    return
                # 停车保持阶段必须 linear=0 且 angular=0，避免底盘进入原地自转/原地转向模式。
                self.publish_zero()
                self.publish_status('分段停车保持中，剩余 {:.1f} s | 停车时不输出角速度'.format(
                    max(0.0, self.pause_duration_s - elapsed)))
                return

            if self.pause_state == 'start':
                elapsed = (now - self.pause_state_start_time).to_sec() if self.pause_state_start_time else 0.0
                ratio = self.smoothstep(elapsed / self.pause_smooth_start_s)
                if elapsed >= self.pause_smooth_start_s:
                    # 起步完成后，以当前实际进度重新累计下一次停车距离；这是防止“刚起步又停车”的核心修复。
                    self.pause_state = 'cruise'
                    self.next_pause_progress = segment_progress_s + self.pause_distance_m
                    self.post_start_slew_until = now + rospy.Duration.from_sec(max(0.0, self.post_start_slew_s))
                    angular_out = self.apply_post_start_slew(base_angular_z, now)
                    self.publish_cmd(base_linear_x, angular_out)
                    self.publish_status('影子起步完成，继续连续倒车。下一次停车目标进度 {:.2f} m。'.format(self.next_pause_progress))
                    return
                linear_x = base_linear_x * ratio
                # 影子起步：真实停车时 angular=0；起步时从停车前保存的修正趋势平滑过渡到当前实时控制量。
                if self.pause_shadow_steering:
                    shadow_target = (1.0 - ratio) * self.pause_shadow_angular + ratio * base_angular_z
                else:
                    shadow_target = base_angular_z
                angular_z = shadow_target * ratio
                # 起步阶段限幅只保护低速大打轮；不改变原始控制方向，起步完成后恢复原算法。
                if self.pause_start_angular_limit > 0.0:
                    dynamic_limit = max(self.pause_start_angular_limit, min(abs(self.pause_shadow_angular), self.max_angular_speed))
                    angular_z = max(-dynamic_limit, min(dynamic_limit, angular_z))
                angular_z = self.rate_limit_angular(angular_z, now, self.max_pause_angular_accel)
                self.publish_cmd(linear_x, angular_z)
                self.publish_status('影子柔顺起步中 {:.0f}% | 模式: {} | 段号: {} | 终点距: {:.2f} m | 横差: {:.2f} m | 航差: {:.1f} deg | shadow_w={:.3f}'.format(
                    ratio * 100.0, self.tracking_mode, self.segment_index, dist_goal, cross_track_error, math.degrees(heading_error), self.pause_shadow_angular))
                return

            # 异常状态兜底：回到巡航，不停车。
            self.pause_state = 'cruise'
            self.publish_cmd(base_linear_x, base_angular_z)


if __name__ == '__main__':
    try:
        ReverseTrackNavNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
