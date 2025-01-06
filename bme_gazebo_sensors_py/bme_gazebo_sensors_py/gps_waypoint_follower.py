#!/usr/bin/env python3
import rclpy
from rclpy.node import Node     # Import ROS2 Node as parent for our own node class
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
import math

def haversine(lat1_deg, lon1_deg, lat2_deg, lon2_deg):
    # 0. Radius of earth in km
    R = 6378.137
    # 1. Convert from degrees to radians
    lat1 = math.radians(lat1_deg)
    lon1 = math.radians(lon1_deg)
    lat2 = math.radians(lat2_deg)
    lon2 = math.radians(lon2_deg)

    # 2. Haversine formula for distance
    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.asin(math.sqrt(a))
    distance_m = R * c * 1000

    # 3. Initial bearing calculation (forward azimuth)
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    bearing_rad = math.atan2(y, x)  # range -π to +π

    return distance_m, bearing_rad

class GPSWaypointFollower(Node):
    def __init__(self):
        super().__init__("gps_waypoint_follower")

        self.latitude = 0
        self.longitude = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.waypoints = [[47.478830, 19.058087],
                         [47.478878, 19.058149],
                         [47.479075, 19.058055],
                         [47.478950, 19.057785]]
        
        self.waypoint_index = 0

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.gps_subscription = self.create_subscription(NavSatFix, '/navsat', self.navsat_callback, 10)
        self.imu_subscription = self.create_subscription(Imu, '/imu', self.imu_callback, 10)

    def navsat_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude

    def imu_callback(self, msg):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)

    def waypoint_follower(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        while rclpy.ok():

            distance, bearing = haversine(self.latitude, self.longitude, self.waypoints[self.waypoint_index][0], self.waypoints[self.waypoint_index][1])

            # Calculate the heading error from robot's yaw angle and bearing from the GPS coordinates
            # In ROS, the default convention for rotations around the z-axis follows the right-hand rule
            # Consequently, rotating clockwise corresponds to a negative yaw change.
            # In traditional navigation, bearing is measured clockwise from North (i.e., 0° = North, 90° = East, etc.).
            # In that system, turning clockwise makes the bearing angle increase.
            # Also robot faces to the east when it starts, so we need to add pi/2 offset to the yaw angle
            heading_error =  -1 * bearing - (self.yaw - math.pi/2)
                    
            if heading_error > math.pi:
                heading_error = heading_error - (2 * math.pi) 
            if heading_error < -math.pi:
                heading_error = heading_error + (2 * math.pi)
        
            self.get_logger().info(f'Distance: {distance:.2f} m, heading error: {heading_error:.3f}')

            # Heading error, threshold is 0.03 rad
            if abs(heading_error) > 0.03:
                # Only rotate in place if there is any heading error
                msg.linear.x = 0.0

                if heading_error < 0:
                    msg.angular.z = -0.3
                else:
                    msg.angular.z = 0.3
            else:
                # Only straight driving, no curves
                msg.angular.z = 0.0
                # Distance error, threshold is 0.3m
                if distance > 0.3:
                    msg.linear.x = 0.5
                else:
                    msg.linear.x = 0.0
                    self.get_logger().info("Target waypoint reached!")
                    self.waypoint_index += 1

            #self.get_logger().info(f'Publishing cmd_vel: linear.x={msg.linear.x}, angular.z={msg.angular.z}')
            self.publisher.publish(msg)

            if self.waypoint_index == len(self.waypoints):
                self.get_logger().info("Last target waypoint reached!")
                break
            else:
                # Spin once to handle ROS 2 callbacks
                rclpy.spin_once(self, timeout_sec=0.05)
        

def main(args=None):
    rclpy.init(args=args)
    node = GPSWaypointFollower()

    try:
        node.waypoint_follower()  # Follow waypoints
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()