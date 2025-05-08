#!/usr/bin/env python3.10
import math
import rclpy
from rclpy.node import Node
from proj_interfaces.srv import MoveToTurtle
from proj_interfaces.srv import EatTurtle
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class MasterTurtleController(Node):
    def __init__(self):
        super().__init__("master_controller")
        self.master_turtle_pose = Pose()
        self.next_turtle = EatTurtle.Request()
        self.next_turtle_pose = None

        self.create_service(MoveToTurtle, "move_to_turtle", self.move_to_turtle_callback)
        self.cmd_pub = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.eat_turtle = self.create_client(EatTurtle, "eat_turtle")
        self.create_subscription(Pose, "turtle1/pose", self.update_turtle_pose, 10)
        return

    def move_to_turtle_callback(self, request, response):
        self.next_turtle.name= request.to_name
        next_turtle_poseXY = request.to_x, request.to_y
        self.set_next_turtle_pose(next_turtle_poseXY)
        self.control_loop_timer = self.create_timer(0.01, self.control_loop_callback)
        return response
    
    def control_loop_callback(self):
        master_turtle_poseXYTheta = self.get_master_turtle_pose()
        next_turtle_poseXY = self.get_next_turtle_pose()
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        distXY = next_turtle_poseXY[0] - master_turtle_poseXYTheta[0], next_turtle_poseXY[1] - master_turtle_poseXYTheta[1]
        distance_from_turtle = math.sqrt(distXY[0]**2 + distXY[1]**2)

        if distance_from_turtle > 0.5:
            cmd.linear.x = distance_from_turtle*2

            theta_from_turtle = math.atan2(distXY[1], distXY[0])
            theta_diff = theta_from_turtle - master_turtle_poseXYTheta[2]
            if theta_diff > math.pi:
                theta_diff -= 2*math.pi
            elif theta_diff < -math.pi:
                theta_diff += 2*math.pi
            # if not -0.2>=theta_diff<=0.2:
            cmd.angular.z = theta_diff*6
        else:
            self.eat_turtle.call_async(self.next_turtle)
            self.destroy_timer(self.control_loop_timer)

        self.cmd_pub.publish(cmd)
        return
    
    # TODO - Make this intuitive approach work

    # def control_loop_callback(self):
    #     master_turtle_poseXYTheta = self.get_master_turtle_pose()
    #     next_turtle_poseXY = self.get_next_turtle_pose()
    #     cmd = Twist()
    #     cmd.linear.x = 0.0
    #     cmd.angular.z = 0.0
    #     distXY = master_turtle_poseXYTheta[0] - next_turtle_poseXY[0], master_turtle_poseXYTheta[1] - next_turtle_poseXY[1]
    #     distance_from_turtle = math.sqrt(distXY[0]**2 + distXY[1]**2)

    #     if distance_from_turtle > 1:
    #         cmd.linear.x = distance_from_turtle*1

    #         theta_from_origin = math.atan2(*distXY) + self.sign_of(distXY[1]) * math.pi/2
    #         theta_from_master = theta_from_origin - master_turtle_poseXYTheta[2]

    #         if abs(theta_from_master) > math.pi:
    #             theta_from_master = 2*math.pi - abs(theta_from_master)*self.sign_of(theta_from_master)*-1
    #         cmd.angular.z = theta_from_master*6
    #     else:
    #         self.eat_turtle.call_async(self.next_turtle)
    #         self.destroy_timer(self.control_loop_timer)

    #     self.cmd_pub.publish(cmd)

    def sign_of(self, uint):
        sign = uint/abs(uint)
        return sign
    
    def update_turtle_pose(self, pose):
        self.master_turtle_pose.x = pose.x
        self.master_turtle_pose.y = pose.y
        self.master_turtle_pose.theta = pose.theta
        return
    
    def get_master_turtle_pose(self):
        return self.master_turtle_pose.x, self.master_turtle_pose.y, self.master_turtle_pose.theta
    
    def set_next_turtle_pose(self, next_turtle_pose:tuple[float]):
        self.next_turtle_pose = next_turtle_pose[0], next_turtle_pose[1]
        return

    def get_next_turtle_pose(self):
        return self.next_turtle_pose


def main(args = None):
    rclpy.init(args = args)
    master_controller_node = MasterTurtleController()
    rclpy.spin(master_controller_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()