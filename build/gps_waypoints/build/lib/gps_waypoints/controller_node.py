import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
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
        self.declare_parameter('cmd_topic', '/wamv/cmd_vel')
        self.declare_parameter('control_mode', 'thrusters')
        self.declare_parameter('left_thrust_topic', '/wamv/thrusters/left/thrust')
        self.declare_parameter('right_thrust_topic', '/wamv/thrusters/right/thrust')
        self.declare_parameter('left_pos_topic', '/wamv/thrusters/left/pos')
        self.declare_parameter('right_pos_topic', '/wamv/thrusters/right/pos')
        self.declare_parameter('max_thrust', 400.0)
        self.declare_parameter('k_thrust_lin', 120.0)
        self.declare_parameter('k_thrust_ang', 80.0)
        self.declare_parameter('max_azimuth', 0.8)
        self.declare_parameter('heading_slowdown_rad', 0.6)
        self.declare_parameter('heading_inplace_rad', 1.0)
        self.declare_parameter('min_forward_thrust', 40.0)
        self.declare_parameter('k_lin', 0.8)
        self.declare_parameter('k_ang', 1.2)
        self.declare_parameter('max_lin', 1.0)
        self.declare_parameter('max_ang', 1.0)
        self.declare_parameter('arrive_dist', 1.5)

        self.goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        self.control_mode = self.get_parameter('control_mode').get_parameter_value().string_value.lower()
        self.left_thrust_topic = self.get_parameter('left_thrust_topic').get_parameter_value().string_value
        self.right_thrust_topic = self.get_parameter('right_thrust_topic').get_parameter_value().string_value
        self.left_pos_topic = self.get_parameter('left_pos_topic').get_parameter_value().string_value
        self.right_pos_topic = self.get_parameter('right_pos_topic').get_parameter_value().string_value
        self.max_thrust = float(self.get_parameter('max_thrust').get_parameter_value().double_value)
        self.k_thrust_lin = float(self.get_parameter('k_thrust_lin').get_parameter_value().double_value)
        self.k_thrust_ang = float(self.get_parameter('k_thrust_ang').get_parameter_value().double_value)
        self.max_azimuth = float(self.get_parameter('max_azimuth').get_parameter_value().double_value)
        self.heading_slowdown_rad = float(self.get_parameter('heading_slowdown_rad').get_parameter_value().double_value)
        self.heading_inplace_rad = float(self.get_parameter('heading_inplace_rad').get_parameter_value().double_value)
        self.min_forward_thrust = float(self.get_parameter('min_forward_thrust').get_parameter_value().double_value)
        self.k_lin = float(self.get_parameter('k_lin').get_parameter_value().double_value)
        self.k_ang = float(self.get_parameter('k_ang').get_parameter_value().double_value)
        self.max_lin = float(self.get_parameter('max_lin').get_parameter_value().double_value)
        self.max_ang = float(self.get_parameter('max_ang').get_parameter_value().double_value)
        self.arrive_dist = float(self.get_parameter('arrive_dist').get_parameter_value().double_value)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.have_pose = False

        self.goal: Optional[PoseStamped] = None

        self.pose_sub = self.create_subscription(TFMessage, self.pose_topic, self.pose_cb, 10)
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

        self.timer = self.create_timer(0.1, self.timer_cb)

    def pose_cb(self, msg: TFMessage):
        if not msg.transforms:
            self.have_pose = False
            return

        t = msg.transforms[0].transform
        self.current_x = t.translation.x
        self.current_y = t.translation.y
        q = t.rotation
        self.current_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.have_pose = True

    def goal_cb(self, msg: PoseStamped):
        self.goal = msg

    def timer_cb(self):
        if self.goal is None or not self.have_pose:
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
                    forward = 0.0
                else:
                    heading_scale = max(0.0, 1.0 - (abs_ang / max(self.heading_slowdown_rad, 1e-6)))
                    forward = min(self.max_thrust, max(self.min_forward_thrust, self.k_thrust_lin * dist)) * heading_scale
                turn = max(-self.max_thrust, min(self.max_thrust, self.k_thrust_ang * ang_err))
                left = max(-self.max_thrust, min(self.max_thrust, forward - turn))
                right = max(-self.max_thrust, min(self.max_thrust, forward + turn))
                azimuth = 0.0

                self.left_thrust_pub.publish(Float64(data=float(left)))
                self.right_thrust_pub.publish(Float64(data=float(right)))
                self.left_pos_pub.publish(Float64(data=float(azimuth)))
                self.right_pos_pub.publish(Float64(data=float(azimuth)))

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
