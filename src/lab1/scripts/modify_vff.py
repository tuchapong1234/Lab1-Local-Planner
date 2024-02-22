#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np


class ModifyVFF_Avoidance(Node):
    def __init__(self):
        super().__init__('modify_vff_avoidance')
        self.create_subscription(LaserScan, "/scan", self.laser_scan_callback, 10)
        self.create_subscription(Vector3, "/attractive_vector", self.vector_callback, 10)
        self.create_timer(0.05, self.vff_controller)

        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "marker_debug", 10)

        self.laser_scan = None
        # Constants for visualization colors
        self.RED = 0
        self.GREEN = 1
        self.BLUE = 2

        self.max_omega = 0.7
        self.max_vel = 0.4

        self.vff_vector = {'attractive': [0.0, 0.0],  # Goal-directed vector
                      'repulsive': [0.0, 0.0],  # Obstacle avoidance vector
                      'result': [0.0, 0.0]}  # Combined vector

        # Callback for processing laser scan messages
    def laser_scan_callback(self, msg):
        # Store the laser scan data for use in the controller
        self.laser_scan = msg

    def vector_callback(self, msg : Vector3):
        self.vff_vector['attractive'][0] = msg.x
        self.vff_vector['attractive'][1] = msg.y

        # The main controller logic for VFF-based obstacle avoidance
    def vff_controller(self):
        if self.laser_scan != None:
            if np.hypot(self.vff_vector['attractive'][0], self.vff_vector['attractive'][1]) > 0.4:
                # print("Target: ", self.goal_x, self.goal_y)
                self.compute_vff(self.laser_scan)
                # Extract the resultant vector for calculating velocity commands
                angle = np.arctan2(self.vff_vector['result'][1], self.vff_vector['result'][0])
                module = np.hypot(self.vff_vector['result'][0], self.vff_vector['result'][1])
                # Create the velocity command message
                if np.fabs(angle) <= np.radians(60) or np.fabs(angle) >= np.radians(120):
                    module = max(-self.max_vel / 2.0, min(np.cos(angle) * module / np.abs(np.cos(angle)), self.max_vel)) #Limit linear velocity
                else :
                    module = 0.0
                angle = max(-self.max_omega, min(angle, self.max_omega)) #Limit angular velocity
                # Publish the velocity command
                self.publish_velocity(module, angle)

                # Publish visualization markers
                marker = self.get_debug_vff(self.vff_vector)
                self.marker_pub.publish(marker)
            else:
                self.publish_velocity(0.0, 0.0)

    
    # Calculate the Virtual Force Field based on laser scan data
    def compute_vff(self, scan : LaserScan):
        # Find the nearest obstacle
        t = self.get_clock().now().to_msg().nanosec ** 1.0e-9
        OBSTACLE_DISTANCE = 0.75 * np.sin(2.0 * np.pi * 2.0 * t) + 1.25

        min_idx = np.argmin(scan.ranges)
        distance_min = scan.ranges[min_idx]

        if distance_min < OBSTACLE_DISTANCE:
            angle = scan.angle_min + (scan.angle_increment * min_idx) 
            complementary_dist = OBSTACLE_DISTANCE - distance_min

        #     # Convert to Cartesian coordinates
            self.vff_vector['repulsive'][0] = complementary_dist * np.cos(angle + np.pi)
            self.vff_vector['repulsive'][1] = complementary_dist * np.sin(angle + np.pi)
        
        # Calculate the resultant vector by combining attractive and repulsive vectors
        self.vff_vector['result'][0] = self.vff_vector['attractive'][0] + self.vff_vector['repulsive'][0]
        self.vff_vector['result'][1] = self.vff_vector['attractive'][1] + self.vff_vector['repulsive'][1]

    def publish_velocity(self, linear_vel, angular_vel):
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.vel_pub.publish(twist)

    
    # Generate visualization markers for the VFF vectors
    def get_debug_vff(self, vff_vectors):
        marker_array = MarkerArray()
        # Create and add markers for attractive, repulsive, and resultant vectors
        marker_array.markers.append(self.make_marker(vff_vectors['attractive'], self.BLUE))
        marker_array.markers.append(self.make_marker(vff_vectors['repulsive'], self.RED))
        marker_array.markers.append(self.make_marker(vff_vectors['result'], self.GREEN))
        return marker_array

    # Utility function to create a marker for visualization
    def make_marker(self, vector, vff_color):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        # Define the start and end points of the marker
        start = Point(x=0.0, y=0.0, z=0.0)
        end = Point(x=vector[0], y=vector[1], z=0.0)
        marker.points = [start, end]
        # Set the scale of the marker
        marker.scale.x = 0.05
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        # Set the color of the marker based on the type of vector
        color = ColorRGBA(a=1.0)  # Fully opaque
        if vff_color == self.RED:
            marker.id = 0
            color.r = 1.0
        elif vff_color == self.GREEN:
            marker.id = 1
            color.g = 1.0
        elif vff_color == self.BLUE:
            marker.id = 2
            color.b = 1.0
        marker.color = color
        return marker

def main(args=None):
    rclpy.init(args=args)
    node = ModifyVFF_Avoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
