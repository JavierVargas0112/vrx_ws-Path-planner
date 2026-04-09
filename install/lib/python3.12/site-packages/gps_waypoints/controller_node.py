import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage


def quaternion_to_yaw(qx, qy, qz, qw):
    # yaw from quaternion
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class GoalToCmd(Node):
    def __init__(self):
        super().__init__('goal_to_cmd')

        # parameters
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('pose_topic', '/wamv/pose')
        self.declare_parameter('gps_topic', '/wamv/sensors/gps/gps/fix')
        self.declare_parameter('imu_topic', '/wamv/sensors/imu/imu/data')
        self.declare_parameter('state_source', 'gps_imu')
        self.declare_parameter('origin_lat', None)
        self.declare_parameter('origin_lon', None)
        self.declare_parameter('cmd_topic', '/wamv/cmd_vel')
        self.declare_parameter('control_mode', 'thrusters')
        self.declare_parameter('left_thrust_topic', '/wamv/thrusters/left/thrust')
        self.declare_parameter('right_thrust_topic', '/wamv/thrusters/right/thrust')
        self.declare_parameter('left_pos_topic', '/wamv/thrusters/left/pos')
        self.declare_parameter('right_pos_topic', '/wamv/thrusters/right/pos')
        self.declare_parameter('base_link_frame', 'wamv/wamv/base_link')
        self.declare_parameter('max_thrust', 400.0)
        self.declare_parameter('k_thrust_lin', 120.0)
        self.declare_parameter('k_thrust_ang', 80.0)
        self.declare_parameter('max_azimuth', 0.8)
        self.declare_parameter('heading_slowdown_rad', 0.6)
        self.declare_parameter('heading_inplace_rad', 1.0)
        self.declare_parameter('heading_deadband_rad', 0.08)
        self.declare_parameter('min_forward_thrust', 0.0)
        self.declare_parameter('turn_close_dist', 8.0)
        self.declare_parameter('turn_to_forward_ratio', 0.6)
        self.declare_parameter('turn_bias_thrust', 20.0)
        self.declare_parameter('k_lin', 0.8)
        self.declare_parameter('k_ang', 1.2)
        self.declare_parameter('max_lin', 1.0)
        self.declare_parameter('max_ang', 1.0)
        self.declare_parameter('arrive_dist', 1.5)

        self.goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.gps_topic = self.get_parameter('gps_topic').get_parameter_value().string_value
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.state_source = self.get_parameter('state_source').get_parameter_value().string_value.lower()
        self.cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        self.control_mode = self.get_parameter('control_mode').get_parameter_value().string_value.lower()
        self.left_thrust_topic = self.get_parameter('left_thrust_topic').get_parameter_value().string_value
        self.right_thrust_topic = self.get_parameter('right_thrust_topic').get_parameter_value().string_value
        self.left_pos_topic = self.get_parameter('left_pos_topic').get_parameter_value().string_value
        self.right_pos_topic = self.get_parameter('right_pos_topic').get_parameter_value().string_value
        self.base_link_frame = self.get_parameter('base_link_frame').get_parameter_value().string_value
        self.max_thrust = float(self.get_parameter('max_thrust').get_parameter_value().double_value)
        self.k_thrust_lin = float(self.get_parameter('k_thrust_lin').get_parameter_value().double_value)
        self.k_thrust_ang = float(self.get_parameter('k_thrust_ang').get_parameter_value().double_value)
        self.max_azimuth = float(self.get_parameter('max_azimuth').get_parameter_value().double_value)
        self.heading_slowdown_rad = float(self.get_parameter('heading_slowdown_rad').get_parameter_value().double_value)
        self.heading_inplace_rad = float(self.get_parameter('heading_inplace_rad').get_parameter_value().double_value)
        self.heading_deadband_rad = float(self.get_parameter('heading_deadband_rad').get_parameter_value().double_value)
        self.min_forward_thrust = float(self.get_parameter('min_forward_thrust').get_parameter_value().double_value)
        self.turn_close_dist = float(self.get_parameter('turn_close_dist').get_parameter_value().double_value)
        self.turn_to_forward_ratio = float(self.get_parameter('turn_to_forward_ratio').get_parameter_value().double_value)
        self.turn_bias_thrust = float(self.get_parameter('turn_bias_thrust').get_parameter_value().double_value)
        self.k_lin = float(self.get_parameter('k_lin').get_parameter_value().double_value)
        self.k_ang = float(self.get_parameter('k_ang').get_parameter_value().double_value)
        self.max_lin = float(self.get_parameter('max_lin').get_parameter_value().double_value)
        self.max_ang = float(self.get_parameter('max_ang').get_parameter_value().double_value)
        self.arrive_dist = float(self.get_parameter('arrive_dist').get_parameter_value().double_value)

        self.origin_lat: Optional[float] = None
        self.origin_lon: Optional[float] = None
        try:
            pv_lat = self.get_parameter('origin_lat').get_parameter_value()
            if hasattr(pv_lat, 'double_value') and pv_lat.double_value is not None and pv_lat.double_value != 0.0:
                self.origin_lat = float(pv_lat.double_value)
            elif hasattr(pv_lat, 'string_value') and pv_lat.string_value:
                self.origin_lat = float(pv_lat.string_value)
        except Exception:
            self.origin_lat = None
        try:
            pv_lon = self.get_parameter('origin_lon').get_parameter_value()
            if hasattr(pv_lon, 'double_value') and pv_lon.double_value is not None and pv_lon.double_value != 0.0:
                self.origin_lon = float(pv_lon.double_value)
            elif hasattr(pv_lon, 'string_value') and pv_lon.string_value:
                self.origin_lon = float(pv_lon.string_value)
        except Exception:
            self.origin_lon = None

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.have_xy = False
        self.have_yaw = False

        self.goal: Optional[PoseStamped] = None

        self.pose_sub = None
        self.gps_sub = None
        self.imu_sub = None
        if self.state_source == 'tf':
            self.pose_sub = self.create_subscription(TFMessage, self.pose_topic, self.pose_cb, 10)
        else:
            self.gps_sub = self.create_subscription(NavSatFix, self.gps_topic, self.gps_cb, 10)
            self.imu_sub = self.create_subscription(Imu, self.imu_topic, self.imu_cb, 10)
        self.goal_sub = self.create_subscription(PoseStamped, self.goal_topic, self.goal_cb, 10)
        self.cmd_pub = None
        self.left_thrust_pub = None
        self.right_thrust_pub = None
        self.left_pos_pub = None
        self.right_pos_pub = None

        if self.control_mode == 'cmd_vel':
            self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        else:
            self.left_thrust_pub = self.create_publisher(Float64, self.left_thrust_topic, 10)
            self.right_thrust_pub = self.create_publisher(Float64, self.right_thrust_topic, 10)
            self.left_pos_pub = self.create_publisher(Float64, self.left_pos_topic, 10)
            self.right_pos_pub = self.create_publisher(Float64, self.right_pos_topic, 10)
            self.get_logger().info(
                f'Using thrusters mode: {self.left_thrust_topic}, {self.right_thrust_topic}, '
                f'{self.left_pos_topic}, {self.right_pos_topic}'
            )
        self.get_logger().info(f'State source: {self.state_source}')

        self.timer = self.create_timer(0.1, self.timer_cb)

    def pose_cb(self, msg: TFMessage):
        if not msg.transforms:
            self.have_pose = False
            return

        chosen = None
        for tf in msg.transforms:
            if tf.child_frame_id == self.base_link_frame and tf.header.frame_id != self.base_link_frame:
                chosen = tf
                break
        if chosen is None:
            for tf in msg.transforms:
                if tf.child_frame_id.endswith('/base_link') and not tf.header.frame_id.endswith('/base_link'):
                    chosen = tf
                    break
        if chosen is None:
            # Some VRX bridge setups publish only sensor-relative transforms.
            # Fallback to the first transform so control can continue.
            chosen = msg.transforms[0]

        t = chosen.transform
        self.current_x = t.translation.x
        self.current_y = t.translation.y
        q = t.rotation
        self.current_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.have_xy = True
        self.have_yaw = True

    def gps_cb(self, msg: NavSatFix):
        if self.origin_lat is None or self.origin_lon is None:
            self.origin_lat = float(msg.latitude)
            self.origin_lon = float(msg.longitude)
            self.get_logger().info(f'Set controller origin from GPS: {self.origin_lat}, {self.origin_lon}')

        east, north = self.lla_to_enu_2d(msg.latitude, msg.longitude, self.origin_lat, self.origin_lon)
        self.current_x = float(east)
        self.current_y = float(north)
        self.have_xy = True

    def imu_cb(self, msg: Imu):
        q = msg.orientation
        self.current_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.have_yaw = True

    def goal_cb(self, msg: PoseStamped):
        self.goal = msg

    def timer_cb(self):
        if self.goal is None or not (self.have_xy and self.have_yaw):
            return

        gx = self.goal.pose.position.x
        gy = self.goal.pose.position.y

        dx = gx - self.current_x
        dy = gy - self.current_y
        dist = math.hypot(dx, dy)

        # desired heading
        desired = math.atan2(dy, dx)
        # shortest angle diff
        ang_err = desired - self.current_yaw
        ang_err = (ang_err + math.pi) % (2 * math.pi) - math.pi

        if dist <= self.arrive_dist:
            self.publish_stop()
        else:
            if self.control_mode == 'cmd_vel':
                cmd = Twist()
                cmd.linear.x = max(-self.max_lin, min(self.max_lin, self.k_lin * dist))
                cmd.angular.z = max(-self.max_ang, min(self.max_ang, self.k_ang * ang_err))
                self.cmd_pub.publish(cmd)
            else:
                # Keep thrusters facing forward and prioritize heading correction first.
                # This avoids large circular trajectories when heading error is high.
                abs_ang = abs(ang_err)
                if abs_ang >= self.heading_inplace_rad:
                    # For large heading errors, keep only a tiny forward component.
                    forward = 0.5 * self.min_forward_thrust
                else:
                    heading_scale = max(0.0, 1.0 - (abs_ang / max(self.heading_slowdown_rad, 1e-6)))
                    forward_raw = min(self.max_thrust, self.k_thrust_lin * dist)
                    if forward_raw > 0.0:
                        forward_raw = max(self.min_forward_thrust, forward_raw)
                    forward = forward_raw * heading_scale
                ang_for_turn = 0.0 if abs_ang < self.heading_deadband_rad else ang_err
                turn = max(-self.max_thrust, min(self.max_thrust, self.k_thrust_ang * ang_for_turn))
                if dist < self.turn_close_dist:
                    turn *= max(0.2, dist / max(self.turn_close_dist, 1e-6))
                # Allow stronger turning authority when heading error is high.
                turn_ratio = self.turn_to_forward_ratio + 0.35 * min(1.0, abs_ang / max(self.heading_inplace_rad, 1e-6))
                turn_cap = turn_ratio * max(forward + self.turn_bias_thrust, self.turn_bias_thrust)
                turn = max(-turn_cap, min(turn_cap, turn))
                left = max(-self.max_thrust, min(self.max_thrust, forward - turn))
                right = max(-self.max_thrust, min(self.max_thrust, forward + turn))
                azimuth = 0.0

                self.left_thrust_pub.publish(Float64(data=float(left)))
                self.right_thrust_pub.publish(Float64(data=float(right)))
                self.left_pos_pub.publish(Float64(data=float(azimuth)))
                self.right_pos_pub.publish(Float64(data=float(azimuth)))

    @staticmethod
    def lla_to_enu_2d(lat, lon, lat0, lon0):
        r = 6378137.0
        dlat = math.radians(lat - lat0)
        dlon = math.radians(lon - lon0)
        lat0_rad = math.radians(lat0)
        north = r * dlat
        east = r * math.cos(lat0_rad) * dlon
        return east, north

    def publish_stop(self):
        if self.control_mode == 'cmd_vel':
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        self.left_thrust_pub.publish(Float64(data=0.0))
        self.right_thrust_pub.publish(Float64(data=0.0))
        self.left_pos_pub.publish(Float64(data=0.0))
        self.right_pos_pub.publish(Float64(data=0.0))


def main(args=None):
    rclpy.init(args=args)
    node = GoalToCmd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
