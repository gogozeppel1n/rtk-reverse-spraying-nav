#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from novatel_oem7_msgs.msg import (
    BESTUTM,
    HEADING2,
    INSPVAX,
    InertialSolutionStatus,
    PositionOrVelocityType,
    SolutionStatus,
)
import tf.transformations as tft

WGS84_A = 6378137.0
WGS84_F = 1 / 298.257223563
WGS84_E2 = WGS84_F * (2 - WGS84_F)
WGS84_E_PRIME2 = WGS84_E2 / (1 - WGS84_E2)
K0 = 0.9996


def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def latlon_to_utm(lat_deg, lon_deg):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)

    zone_number = int((lon_deg + 180.0) / 6.0) + 1
    lon_origin_deg = (zone_number - 1) * 6 - 180 + 3
    lon_origin = math.radians(lon_origin_deg)

    n = WGS84_A / math.sqrt(1 - WGS84_E2 * math.sin(lat) ** 2)
    t = math.tan(lat) ** 2
    c = WGS84_E_PRIME2 * math.cos(lat) ** 2
    a = math.cos(lat) * (lon - lon_origin)

    m = WGS84_A * ((1 - WGS84_E2 / 4 - 3 * WGS84_E2**2 / 64 - 5 * WGS84_E2**3 / 256) * lat
                   - (3 * WGS84_E2 / 8 + 3 * WGS84_E2**2 / 32 + 45 * WGS84_E2**3 / 1024) * math.sin(2 * lat)
                   + (15 * WGS84_E2**2 / 256 + 45 * WGS84_E2**3 / 1024) * math.sin(4 * lat)
                   - (35 * WGS84_E2**3 / 3072) * math.sin(6 * lat))

    easting = K0 * n * (a + (1 - t + c) * a**3 / 6
                        + (5 - 18 * t + t**2 + 72 * c - 58 * WGS84_E_PRIME2) * a**5 / 120) + 500000.0

    northing = K0 * (m + n * math.tan(lat) * (a**2 / 2
                  + (5 - t + 9 * c + 4 * c**2) * a**4 / 24
                  + (61 - 58 * t + t**2 + 600 * c - 330 * WGS84_E_PRIME2) * a**6 / 720))

    if lat_deg < 0:
        northing += 10000000.0

    return easting, northing, zone_number


def heading_to_ros_yaw_deg(heading_deg, yaw_offset_deg=0.0):
    return wrap_to_pi(math.pi / 2.0 - math.radians(heading_deg) + math.radians(yaw_offset_deg))


class RtkPoseBridge(object):
    def __init__(self):
        rospy.init_node('inspvax_to_utm_pose', anonymous=False)

        self.bestutm_topic = rospy.get_param('~bestutm_topic', '/novatel/oem7/bestutm')
        self.inspvax_topic = rospy.get_param('~inspvax_topic', '/novatel/oem7/inspvax')
        self.heading2_topic = rospy.get_param('~heading2_topic', '/novatel/oem7/heading2')
        self.pose_topic = rospy.get_param('~pose_topic', '/rtk_reverse_nav/pose')
        self.odom_topic = rospy.get_param('~odom_topic', '/rtk_reverse_nav/utm_odom')
        self.fix_valid_topic = rospy.get_param('~fix_valid_topic', '/rtk_reverse_nav/fix_valid')
        self.source_status_topic = rospy.get_param('~source_status_topic', '/rtk_reverse_nav/source_status')
        self.frame_id = rospy.get_param('~frame_id', 'utm')
        self.child_frame_id = rospy.get_param('~child_frame_id', 'base_link')
        self.yaw_offset_deg = rospy.get_param('~yaw_offset_deg', 0.0)
        self.origin_mode = rospy.get_param('~origin_mode', 'absolute_utm')
        self.prefer_bestutm = rospy.get_param('~prefer_bestutm', True)
        self.allow_inspvax_position_fallback = rospy.get_param('~allow_inspvax_position_fallback', True)
        self.allow_float_solution = rospy.get_param('~allow_float_solution', False)
        self.position_timeout = rospy.get_param('~position_timeout', 0.50)
        self.heading_timeout = rospy.get_param('~heading_timeout', 0.50)
        self.max_position_stddev = rospy.get_param('~max_position_stddev', 0.10)
        self.max_heading_stddev_deg = rospy.get_param('~max_heading_stddev_deg', 2.0)
        self.publish_rate = rospy.get_param('~publish_rate', 20.0)

        self.origin_e = None
        self.origin_n = None
        self.zone = None

        self.bestutm_msg = None
        self.inspvax_msg = None
        self.heading2_msg = None
        self.last_valid = False
        self.last_status_text = ''

        self.pose_pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=20)
        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=20)
        self.fix_valid_pub = rospy.Publisher(self.fix_valid_topic, Bool, queue_size=10, latch=True)
        self.source_status_pub = rospy.Publisher(self.source_status_topic, String, queue_size=10, latch=True)

        self.bestutm_sub = rospy.Subscriber(self.bestutm_topic, BESTUTM, self.bestutm_cb, queue_size=20)
        self.inspvax_sub = rospy.Subscriber(self.inspvax_topic, INSPVAX, self.inspvax_cb, queue_size=20)
        self.heading2_sub = rospy.Subscriber(self.heading2_topic, HEADING2, self.heading2_cb, queue_size=20)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate, 1.0)), self.publish_timer)

        rospy.loginfo(
            'rtk pose bridge ready | bestutm=%s | inspvax=%s | heading2=%s | pose=%s',
            self.bestutm_topic, self.inspvax_topic, self.heading2_topic, self.pose_topic
        )

    def allowed_pos_types(self):
        allowed = {
            PositionOrVelocityType.NARROW_INT,
            PositionOrVelocityType.INS_RTKFIXED,
            PositionOrVelocityType.RTK_DIRECT_INS,
            PositionOrVelocityType.WIDE_INT,
            PositionOrVelocityType.L1_INT,
        }
        if self.allow_float_solution:
            allowed.update({
                PositionOrVelocityType.NARROW_FLOAT,
                PositionOrVelocityType.INS_RTKFLOAT,
                PositionOrVelocityType.L1_FLOAT,
            })
        return allowed

    def bestutm_cb(self, msg):
        self.bestutm_msg = msg

    def inspvax_cb(self, msg):
        self.inspvax_msg = msg

    def heading2_cb(self, msg):
        self.heading2_msg = msg

    def is_fresh(self, header, timeout_sec):
        if header is None:
            return False
        age = (rospy.Time.now() - header.stamp).to_sec()
        return age <= timeout_sec

    def extract_nested_status(self, obj, field_name, default_value=None):
        value = getattr(obj, field_name, default_value)
        if hasattr(value, 'status'):
            return value.status
        if hasattr(value, 'type'):
            return value.type
        return value

    def bestutm_valid(self, msg):
        if msg is None or not self.is_fresh(msg.header, self.position_timeout):
            return False
        if self.extract_nested_status(msg.sol_status, 'status', None) != SolutionStatus.SOL_COMPUTED:
            return False
        if self.extract_nested_status(msg.pos_type, 'type', None) not in self.allowed_pos_types():
            return False
        if max(msg.easting_stddev, msg.northing_stddev) > self.max_position_stddev:
            return False
        return True

    def inspvax_position_valid(self, msg):
        if msg is None or not self.is_fresh(msg.header, self.position_timeout):
            return False
        if self.extract_nested_status(msg.ins_status, 'status', None) not in {
            InertialSolutionStatus.INS_SOLUTION_GOOD,
            InertialSolutionStatus.INS_ALIGNMENT_COMPLETE,
        }:
            return False
        if self.extract_nested_status(msg.pos_type, 'type', None) not in self.allowed_pos_types():
            return False
        if max(msg.longitude_stdev, msg.latitude_stdev) > self.max_position_stddev:
            return False
        return True

    def heading2_valid(self, msg):
        if msg is None or not self.is_fresh(msg.header, self.heading_timeout):
            return False
        if self.extract_nested_status(msg.sol_status, 'status', None) != SolutionStatus.SOL_COMPUTED:
            return False
        if self.extract_nested_status(msg.pos_type, 'type', None) not in self.allowed_pos_types():
            return False
        if msg.heading_stdev > self.max_heading_stddev_deg:
            return False
        return True

    def inspvax_heading_valid(self, msg):
        if msg is None or not self.is_fresh(msg.header, self.heading_timeout):
            return False
        if self.extract_nested_status(msg.ins_status, 'status', None) not in {
            InertialSolutionStatus.INS_SOLUTION_GOOD,
            InertialSolutionStatus.INS_ALIGNMENT_COMPLETE,
        }:
            return False
        if self.extract_nested_status(msg.pos_type, 'type', None) not in self.allowed_pos_types():
            return False
        if msg.azimuth_stdev > self.max_heading_stddev_deg:
            return False
        return True

    def select_position(self):
        if self.prefer_bestutm and self.bestutm_valid(self.bestutm_msg):
            return self.bestutm_msg.easting, self.bestutm_msg.northing, self.bestutm_msg.height, 'bestutm'
        if self.allow_inspvax_position_fallback and self.inspvax_position_valid(self.inspvax_msg):
            easting, northing, zone = latlon_to_utm(self.inspvax_msg.latitude, self.inspvax_msg.longitude)
            if self.zone is None:
                self.zone = zone
            return easting, northing, self.inspvax_msg.height, 'inspvax_utm_fallback'
        if not self.prefer_bestutm and self.inspvax_position_valid(self.inspvax_msg):
            easting, northing, zone = latlon_to_utm(self.inspvax_msg.latitude, self.inspvax_msg.longitude)
            if self.zone is None:
                self.zone = zone
            return easting, northing, self.inspvax_msg.height, 'inspvax_utm_primary'
        if self.bestutm_valid(self.bestutm_msg):
            return self.bestutm_msg.easting, self.bestutm_msg.northing, self.bestutm_msg.height, 'bestutm_fallback'
        return None

    def select_heading(self):
        if self.heading2_valid(self.heading2_msg):
            yaw = heading_to_ros_yaw_deg(self.heading2_msg.heading, self.yaw_offset_deg)
            return yaw, 'heading2'
        if self.inspvax_heading_valid(self.inspvax_msg):
            yaw = heading_to_ros_yaw_deg(self.inspvax_msg.azimuth, self.yaw_offset_deg)
            return yaw, 'inspvax'
        return None

    def localize_position(self, easting, northing):
        if self.origin_mode == 'local_zero':
            if self.origin_e is None:
                self.origin_e = easting
                self.origin_n = northing
                rospy.loginfo('Set local_zero origin: E=%.3f N=%.3f', self.origin_e, self.origin_n)
            return easting - self.origin_e, northing - self.origin_n
        return easting, northing

    def build_velocity(self, yaw):
        if self.inspvax_msg is None or not self.is_fresh(self.inspvax_msg.header, self.position_timeout):
            return 0.0, 0.0, 0.0
        north_v = self.inspvax_msg.north_velocity
        east_v = self.inspvax_msg.east_velocity
        forward_v = math.cos(yaw) * east_v + math.sin(yaw) * north_v
        lateral_v = -math.sin(yaw) * east_v + math.cos(yaw) * north_v
        return forward_v, lateral_v, self.inspvax_msg.up_velocity

    def publish_timer(self, _event):
        position = self.select_position()
        heading = self.select_heading()

        if position is None or heading is None:
            valid = False
            reasons = []
            if position is None:
                reasons.append('位置源无有效数据')
            if heading is None:
                reasons.append('航向源无有效数据')
            status = 'RTK pose invalid | ' + ' / '.join(reasons)
            self.fix_valid_pub.publish(Bool(data=False))
            if status != self.last_status_text:
                rospy.logwarn(status)
                self.last_status_text = status
            self.source_status_pub.publish(String(data=status))
            self.last_valid = False
            return

        easting, northing, height, pos_source = position
        yaw, yaw_source = heading
        x, y = self.localize_position(easting, northing)
        q = tft.quaternion_from_euler(0.0, 0.0, yaw)

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = height
        pose.pose.orientation = Quaternion(*q)
        self.pose_pub.publish(pose)

        odom = Odometry()
        odom.header = pose.header
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose = pose.pose
        forward_v, lateral_v, up_v = self.build_velocity(yaw)
        odom.twist.twist.linear.x = forward_v
        odom.twist.twist.linear.y = lateral_v
        odom.twist.twist.linear.z = up_v
        self.odom_pub.publish(odom)

        status = 'RTK pose valid | pos={} | yaw={} | origin_mode={}'.format(pos_source, yaw_source, self.origin_mode)
        self.fix_valid_pub.publish(Bool(data=True))
        self.source_status_pub.publish(String(data=status))
        if status != self.last_status_text:
            rospy.loginfo(status)
            self.last_status_text = status
        self.last_valid = True


if __name__ == '__main__':
    try:
        RtkPoseBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
