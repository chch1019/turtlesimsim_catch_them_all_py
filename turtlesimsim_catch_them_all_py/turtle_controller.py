#!/usr/bin/env python3
from soupsieve import select
import math
import rclpy
from functools import partial
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle

class TurtleControllerNode(Node):  # MODIFY NAME
    def __init__(self):
        super().__init__("turtle_controller")  # MODIFY NAME
        self.turtle_to_catch = None
        #self.x_target = 4.0
        #self.y_target = 8.0
        
        self.pose_ = None
        self.pose_subscriber_ = self.create_subscription(
            Pose, "turtle1/pose", self.callback_turtle_pose, 10)
        
        self.alive_turtle_subscriber_ = self.create_subscription(
            TurtleArray, "alive_turtles", self.callback_alive_turtles, 10)
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        self.control_loo_timer_ = self.create_timer(0.01, self.control_loop)

    def callback_turtle_pose(self, msg):
        self.pose_ = msg
    
    def callback_alive_turtles(self, msg):
        if len(msg.turtles)>0:
            self.turtle_to_catch = msg.turtles[0]
        

    def control_loop(self):
        if self.pose_ == None or self.turtle_to_catch == None:
            return
        
        x_dist = self.turtle_to_catch.x - self.pose_.x
        y_dist = self.turtle_to_catch.y - self.pose_.y
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
            #target reached 
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.call_catch_turtle_server(self.turtle_to_catch.name)
            self.turtle_to_catch = None

        self.cmd_vel_publisher_.publish(msg)

    def call_catch_turtle_server(self, turtle_name):
        client = self.create_client(CatchTurtle, "catch_turtle")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server...")

        request = CatchTurtle.Request()
        request.name = turtle_name

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_catch_turtle, turtle_name=turtle_name))

    def callback_call_catch_turtle(self, future, turtle_name):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error("Turtle" + str(turtle_name) + "could not be caught")

        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))



def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()  # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
