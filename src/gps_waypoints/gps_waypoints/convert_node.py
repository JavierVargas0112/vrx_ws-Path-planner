import math
import os
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header


class NavSatToENU(Node):
    def __init__(self):
        super().__init__('navsat_to_enu')

        # parameters
        self.declare_parameter('gps_topic', '/wamv/sensors/gps/gps/fix')
        self.declare_parameter('goal_topic', '/waypoint_gps')
        self.declare_parameter('publish_topic', '/wamv/goal_pose')
        self.declare_parameter('origin_lat', None)
        self.declare_parameter('origin_lon', None)
        self.declare_parameter('frame_id', 'enu')

        self.gps_topic = self.get_parameter('gps_topic').get_parameter_value().string_value
        self.goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        self.publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # try to read optional origin parameters (may be unset)
        self.origin_lat: Optional[float] = None
        self.origin_lon: Optional[float] = None
        try:
            p_lat = self.get_parameter('origin_lat')
            pv_lat = p_lat.get_parameter_value()
            # prefer double_value, fall back to string
            if hasattr(pv_lat, 'double_value') and pv_lat.double_value is not None and pv_lat.double_value != 0.0:
                self.origin_lat = float(pv_lat.double_value)
            elif hasattr(pv_lat, 'string_value') and pv_lat.string_value:
                self.origin_lat = float(pv_lat.string_value)
        except Exception:
            self.origin_lat = None

        try:
            p_lon = self.get_parameter('origin_lon')
            pv_lon = p_lon.get_parameter_value()
            if hasattr(pv_lon, 'double_value') and pv_lon.double_value is not None and pv_lon.double_value != 0.0:
                self.origin_lon = float(pv_lon.double_value)
            elif hasattr(pv_lon, 'string_value') and pv_lon.string_value:
                self.origin_lon = float(pv_lon.string_value)
        except Exception:
            self.origin_lon = None

        if self.origin_lat is not None and self.origin_lon is not None:
            self.get_logger().info(f'Using configured origin: {self.origin_lat}, {self.origin_lon}')

        # subscribers + publisher
        self.goal_sub = self.create_subscription(NavSatFix, self.goal_topic, self.goal_cb, 10)
        self.gps_sub = self.create_subscription(NavSatFix, self.gps_topic, self.gps_cb, 10)
        self.pose_pub = self.create_publisher(PoseStamped, self.publish_topic, 10)

    def gps_cb(self, msg: NavSatFix):
        # use first GPS fix as origin if not configured
        if self.origin_lat is None or self.origin_lon is None:
            if msg.status.status >= 0:
                self.origin_lat = float(msg.latitude)
                self.origin_lon = float(msg.longitude)
                self.get_logger().info(f'Set origin from GPS: {self.origin_lat}, {self.origin_lon}')

    def goal_cb(self, msg: NavSatFix):
        if self.origin_lat is None or self.origin_lon is None:
            self.get_logger().warning('Origin not set yet — cannot convert to ENU')
            return

        east, north, up = self.lla_to_enu(msg.latitude, msg.longitude, msg.altitude, self.origin_lat, self.origin_lon, 0.0)

        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = float(east)
        pose.pose.position.y = float(north)
        pose.pose.position.z = float(up)
        # identity orientation
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        self.pose_pub.publish(pose)

    @staticmethod
    def lla_to_enu(lat, lon, alt, lat0, lon0, alt0):
        # Simple local tangent plane approximation (meters)
        # East = R * cos(lat0) * dlon
        # North = R * dlat
        R = 6378137.0
        dlat = math.radians(lat - lat0)
        dlon = math.radians(lon - lon0)
        lat0_rad = math.radians(lat0)
        north = R * dlat
        east = R * math.cos(lat0_rad) * dlon
        up = alt - alt0
        return east, north, up


def main(args=None):
    rclpy.init(args=args)
    node = NavSatToENU()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
