#!/usr/bin/env python3
import rclpy
from rclpy.node import Node     # Import ROS2 Node as parent for our own node class
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
import math

def haversine(lat1, lon1, lat2, lon2):

    # Calculate distance
    R = 6378.137 # Radius of earth in km
    dLat = lat2 * math.pi / 180 - lat1 * math.pi / 180
    dLon = lon2 * math.pi / 180 - lon1 * math.pi / 180
    a = math.sin(dLat/2) * math.sin(dLat/2) + math.cos(lat1 * math.pi / 180) * math.cos(lat2 * math.pi / 180) * math.sin(dLon/2) * math.sin(dLon/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    d = R * c * 1000 # in meters

    # Calculate heading
    y = math.sin(dLon) * math.cos(dLon)
    x = math.cos(lat1 * math.pi / 180) * math.sin(lat2 * math.pi / 180) - math.sin(lat1 * math.pi / 180) * math.cos(lat2 * math.pi / 180) * math.cos(dLon)
    bearing = -math.atan2(y,x)

    return d, bearing

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
                         [47.478957, 19.057763]]
        
        self.waypoint_index = 0

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.gps_subscription = self.create_subscription(NavSatFix, '/navsat', self.navsat_callback, 10)
        self.imu_subscription = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def navsat_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude

    def imu_callback(self, msg):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        distance, bearing = haversine(self.latitude, self.longitude, self.waypoints[self.waypoint_index][0], self.waypoints[self.waypoint_index][1])

        # calculate heading error from yaw and bearing
        heading_error = bearing - self.yaw + 1.5707
                
        if heading_error > math.pi:
            heading_error = heading_error - (2 * math.pi) 
        if heading_error < -math.pi:
            heading_error = heading_error + (2 * math.pi)
       
        #rospy.loginfo("Distance: %.3f m, heading error: %.3f rad." % (distance, heading_error))
        self.get_logger().info(f'Distance: {distance} m, heading error: {heading_error}')
        #rospy.loginfo("Bearing: %.3f rad, yaw: %.3f rad, error: %.3f rad" % (bearing, yaw, headingError))

        # Heading error, threshold is 0.1 rad
        if abs(heading_error) > 0.05:
            # Only rotate in place if there is any heading error
            msg.linear.x = 0.0

            if heading_error < 0:
                msg.angular.z = -0.2
            else:
                msg.angular.z = 0.2
        else:
            # Only straight driving, no curves
            msg.angular.z = 0.0
            # Distance error, threshold is 0.2m
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
            rclpy.shutdown()
        

def main(args=None):
    rclpy.init(args=args)
    node = GPSWaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()