import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
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
        self.declare_parameter('k_lin', 0.8)
        self.declare_parameter('k_ang', 1.2)
        self.declare_parameter('max_lin', 1.0)
        self.declare_parameter('max_ang', 1.0)
        self.declare_parameter('arrive_dist', 1.5)

        self.goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
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
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)

        self.timer = self.create_timer(0.1, self.timer_cb)

    def pose_cb(self, msg: TFMessage):
        # expect at least one TransformStamped
        try:
            t = msg.transforms[0].transform
            self.current_x = t.translation.x
            self.current_y = t.translation.y
            q = t.rotation
            self.current_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
            self.have_pose = True
        except Exception:
            self.have_pose = False

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

        cmd = Twist()
        if dist <= self.arrive_dist:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            cmd.linear.x = max(-self.max_lin, min(self.max_lin, self.k_lin * dist))
            cmd.angular.z = max(-self.max_ang, min(self.max_ang, self.k_ang * ang_err))

        self.cmd_pub.publish(cmd)


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
