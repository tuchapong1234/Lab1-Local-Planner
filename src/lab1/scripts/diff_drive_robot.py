#!/usr/bin/python3

from lab1.mobile_kinematic import Diff_Drive_Kinematic
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState
import numpy as np

class DiffDriveRobot(Node):
    def __init__(self):
        super().__init__('diff_drive_robot')

        self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 10)
        self.create_subscription(JointState, "/joint_states", self.wheel_callback, 10)
        self.create_timer(0.1, self.timer_callback)

        self.vel_cont = self.create_publisher(Float64MultiArray, "/velocity_controllers/commands", 10)
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.pub_tf_br = TransformBroadcaster(self)

        self.Kine = Diff_Drive_Kinematic(r=0.075, b=0.4)
        self.quat = Quaternion()

        self.qd_cmd = np.zeros(2)
        self.qd_fb = np.zeros(2)

        self.pose = np.zeros(3)
        self.twist = np.zeros(2)
        self.lasttimestamp = self.get_clock().now()
    
    def timer_callback(self):
        # calculate dt
        currenttimestamp = self.get_clock().now()
        dt = (currenttimestamp - self.lasttimestamp).to_msg().nanosec * 1.0e-9
        self.lasttimestamp = currenttimestamp
        # publish wheel speed
        self.pub_wheelspeed(self.qd_cmd.tolist())
        # update odometry
        self.pose = self.Kine.get_pose(qd=self.qd_fb ,dt=dt)
        self.twist = self.Kine.get_twist(qd=self.qd_fb)
        # calculate quaternion angle
        quat = quaternion_from_euler(0.0, 0.0, self.pose[2])
        self.quat.x = quat[0]
        self.quat.y = quat[1]
        self.quat.z = quat[2]
        self.quat.w = quat[3]
        # publish odometry and transformation
        self.pub_odometry()
        self.pub_transformation()

    def cmd_callback(self, msg : Twist):
        # subscribe cmd_vel and transform to wheel speed
        self.qd_cmd = self.Kine.get_wheelspeed(
            [msg.linear.x,
             msg.angular.z])

    def wheel_callback(self, msg : JointState):
        self.qd_fb = np.array(msg.velocity)
        
    def pub_wheelspeed(self, data : list):
        wheel_cont = Float64MultiArray()
        wheel_cont.data = data
        self.vel_cont.publish(wheel_cont)

    def pub_transformation(self):
        tf_stamp = TransformStamped()
        tf_stamp.header.stamp = self.lasttimestamp.to_msg()
        tf_stamp.header.frame_id = "odom"
        tf_stamp.child_frame_id = "base_link"
        tf_stamp.transform.translation.x = self.pose[0]
        tf_stamp.transform.translation.y = self.pose[1]
        tf_stamp.transform.rotation = self.quat
        self.pub_tf_br.sendTransform(tf_stamp)

    def pub_odometry(self):
        odom = Odometry()
        odom.header.stamp = self.lasttimestamp.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.pose[0]
        odom.pose.pose.position.y = self.pose[1]
        odom.pose.pose.orientation = self.quat
        odom.twist.twist.linear.x = self.twist[0]
        odom.twist.twist.angular.z = self.twist[1]
        self.pub_odom.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
