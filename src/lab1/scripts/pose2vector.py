#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3


class Pose2Vector(Node):
    def __init__(self):
        super().__init__('pose_to_vector')
        self.create_subscription(PoseStamped, "/target_pose", self.goal_pose_callback, 10)
        self.vector_pub = self.create_publisher(Vector3, "/attractive_vector", 10)

    def goal_pose_callback(self, msg : PoseStamped):
        vec = Vector3()
        vec.x = msg.pose.position.x
        vec.y = msg.pose.position.y
        self.vector_pub.publish(vec)

def main(args=None):
    rclpy.init(args=args)
    node = Pose2Vector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()