#!/usr/bin/env python3
from soupsieve import select
import math
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class TurtleControllerNode(Node):  # MODIFY NAME
    def __init__(self):
        super().__init__("turtle_controller")  # MODIFY NAME
        self.x_target = 4.0
        self.y_target = 8.0
        
        self.pose_ = None
        self.pose_subscriber_ = self.create_subscription(
            Pose, "turtle1/pose", self.callback_turtle_pose, 10)
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        self.control_loo_timer_ = self.create_timer(0.01, self.control_loop)

    def callback_turtle_pose(self, msg):
        self.pose_ = msg

    def control_loop(self):
        if self.pose_ == None:
            return
        
        x_dist = self.x_target - self.pose_.x
        y_dist = self.y_target - self.pose_.y
        target_dist = math.sqrt(x_dist * x_dist + y_dist * y_dist)

        msg = Twist()
        if target_dist > 0.5:
            #pose
            msg.linear.x = 2 * target_dist # tuning the distance by multiplying with 2 for  better path 

            #orientation
            target_theta = math.atan2(y_dist, x_dist)
            diff = target_theta - self.pose_.theta

            #normalize the angle
            if diff > math.pi:
                diff -= 2 * math.pi
            elif diff < -math.pi:
                diff += 2 * math.pi

            msg.angular.z = 6 * diff # tuning the angle by multiplying with 6  for  better path 
            

        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.cmd_vel_publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()  # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
