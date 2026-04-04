import json
import math
import os
from typing import List, Dict

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header


def haversine_distance(lat1, lon1, lat2, lon2):
    # returns meters
    R = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2.0)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2.0)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('gps_waypoint_follower')

        # Parameters
        self.declare_parameter('gps_topic', '/wamv/sensors/gps/gps/fix')
        self.declare_parameter('goal_topic', '/waypoint_gps')
        self.declare_parameter('checkpoints_file', os.path.join(os.getcwd(), 'checkpoints.json'))
        self.declare_parameter('threshold_m', 5.0)
        self.declare_parameter('publish_rate', 1.0)

        self.gps_topic = self.get_parameter('gps_topic').get_parameter_value().string_value
        self.goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        self.checkpoints_file = self.get_parameter('checkpoints_file').get_parameter_value().string_value
        self.threshold = float(self.get_parameter('threshold_m').get_parameter_value().double_value)
        self.publish_rate = float(self.get_parameter('publish_rate').get_parameter_value().double_value)

        self.get_logger().info(f'GPS topic: {self.gps_topic}')
        self.get_logger().info(f'Goal topic: {self.goal_topic}')
        self.get_logger().info(f'Checkpoints file: {self.checkpoints_file}')

        # state
        self.current_fix: NavSatFix = None
        self.checkpoints: List[Dict] = []
        self.idx = 0

        # ROS interfaces
        self.gps_sub = self.create_subscription(NavSatFix, self.gps_topic, self.gps_cb, 10)
        self.goal_pub = self.create_publisher(NavSatFix, self.goal_topic, 10)

        # load checkpoints
        self.load_checkpoints()

        # timer to publish goal
        period = 1.0 / max(0.01, self.publish_rate)
        self.timer = self.create_timer(period, self.timer_cb)

    def load_checkpoints(self):
        if not os.path.exists(self.checkpoints_file):
            self.get_logger().error(f'Checkpoints file not found: {self.checkpoints_file}')
            return
        try:
            with open(self.checkpoints_file, 'r') as f:
                data = json.load(f)
        except Exception as e:
            self.get_logger().error(f'Failed to read checkpoints file: {e}')
            return

        # Accept either a list or an object with key 'checkpoints'
        if isinstance(data, dict) and 'checkpoints' in data:
            cp = data['checkpoints']
        elif isinstance(data, list):
            cp = data
        else:
            self.get_logger().error('Invalid checkpoints JSON format')
            return

        for entry in cp:
            if 'lat' in entry and 'lon' in entry:
                lat = float(entry['lat'])
                lon = float(entry['lon'])
                alt = float(entry.get('alt', 0.0))
                self.checkpoints.append({'lat': lat, 'lon': lon, 'alt': alt})

        self.get_logger().info(f'Loaded {len(self.checkpoints)} checkpoints')

    def gps_cb(self, msg: NavSatFix):
        self.current_fix = msg

    def publish_goal(self, target: Dict):
        msg = NavSatFix()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        msg.status = self.current_fix.status if self.current_fix is not None else msg.status
        msg.latitude = float(target['lat'])
        msg.longitude = float(target['lon'])
        msg.altitude = float(target.get('alt', 0.0))
        self.goal_pub.publish(msg)

    def timer_cb(self):
        if not self.checkpoints:
            return

        if self.idx >= len(self.checkpoints):
            self.get_logger().info('All checkpoints visited')
            return

        target = self.checkpoints[self.idx]

        # publish current target continuously
        self.publish_goal(target)

        # check distance
        if self.current_fix is None:
            self.get_logger().debug('Waiting for GPS fix...')
            return

        d = haversine_distance(self.current_fix.latitude, self.current_fix.longitude, target['lat'], target['lon'])
        self.get_logger().debug(f'Distance to target {self.idx}: {d:.2f} m')
        if d <= self.threshold:
            self.get_logger().info(f'Reached checkpoint {self.idx} (d={d:.1f} m)')
            self.idx += 1
            if self.idx < len(self.checkpoints):
                self.get_logger().info(f'Moving to checkpoint {self.idx}')
            else:
                self.get_logger().info('Finished all checkpoints')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
