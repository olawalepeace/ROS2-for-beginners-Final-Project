#!/usr/bin/env python3.10
import rclpy
from rclpy.node import Node

from turtlesim.srv import Spawn
from turtlesim.srv import Kill

from proj_interfaces.srv import EatTurtle
from proj_interfaces.srv import MoveToTurtle
from proj_interfaces.msg import MasterTurtlePose

import random
import math

class TurtleManager(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.declare_parameter("spawn_period", 1.0)
        self.declare_parameter("eat_closest_turtle", True)
        self.alive_turtles = []
        self.spawn_request = Spawn.Request()

        self.create_service(EatTurtle, "eat_turtle", self.eat_turtle_callback)
        self.create_subscription(MasterTurtlePose, "master_turtle_pose", self.master_turtle_pose_callback, 10)
        self.turtle_spawner = self.create_client(Spawn, "spawn")
        self.kill_turtle = self.create_client(Kill, "kill")
        self.move_to_turtle = self.create_client(MoveToTurtle, "move_to_turtle")
        spawn_period = self.get_parameter("spawn_period").value
        self.create_timer(spawn_period, self.turtle_spawner_callback)
        self.move_to_next_turtle()
        self.master_turtle_poseXY = 5.544445, 5.544445
        return

    def turtle_spawner_callback(self):
        # poseXYTheta = random.random()*9.134332 + 1, random.random()*9.134332 + 1, random.random()*3.142
        poseXYTheta = random.uniform(0.555555, 10.222222), random.uniform(0.555555, 10.222222), random.random()*3.142
        self.set_spawn_request(*poseXYTheta)
        future = self.turtle_spawner.call_async(self.spawn_request)
        future.add_done_callback(self.turtle_spawned_callback)
        return

    def turtle_spawned_callback(self, future):
        response = future.result()
        self.get_logger().info(f"Spawned: {response.name}")
        turtle_info_dict = {"name": response.name, "pose": (self.get_spawn_pose())}
        self.alive_turtles.append(turtle_info_dict)
        return

    def eat_turtle_callback(self, request, response):
        turtle_name = request.name
        alive_turtles = self.get_alive_turtles()
        turtle_to_kill = Kill.Request()
        for turtle in alive_turtles:
            if turtle_name == turtle["name"]:
                turtle_to_kill.name = turtle_name
                self.kill_turtle.call_async(turtle_to_kill)
                alive_turtles.remove(turtle)
                self.update_alive_turtles(alive_turtles)
                self.move_to_next_turtle()
                break
        return response

    def move_to_next_turtle(self):
        alive_turtles = self.get_alive_turtles()
        eat_closest_turtle = self.get_parameter("eat_closest_turtle").value
        move_turtle = MoveToTurtle.Request()        
        if not alive_turtles:
            self.waiting_timer = self.create_timer(0.1, self.waiting_callback)
            return
        elif eat_closest_turtle:
            turtle_distance_dict = {}
            for turtle in alive_turtles:
                turtle_name, turtle_pose = turtle["name"], turtle["pose"]
                master_turtle_poseXY = self.get_master_turtle_poseXY()
                turtle_distance = self.determine_turtle_distance(master_turtle_poseXY, turtle_pose)
                turtle_distance_dict[turtle_name] = turtle_distance
            closest_turtle_name = min(turtle_distance_dict, key=turtle_distance_dict.get)
            for turtle in alive_turtles:
                if closest_turtle_name == turtle["name"]:
                    move_turtle.to_name = turtle["name"]
                    move_turtle.to_x, move_turtle.to_y = turtle["pose"]
                    break
        else:
            next_turtle = alive_turtles[0]
            move_turtle.to_name= next_turtle["name"]
            move_turtle.to_x, move_turtle.to_y = next_turtle["pose"]
        self.move_to_turtle.call_async(move_turtle)
        return
    
    def waiting_callback(self):
        alive_turtles = self.get_alive_turtles()
        if alive_turtles:
            self.move_to_next_turtle()
            self.destroy_timer(self.waiting_timer)
        return
    
    def set_spawn_request(self, x, y, theta):
        self.spawn_request.x = x
        self.spawn_request.y = y
        self.spawn_request.theta = theta
        return
    
    def get_spawn_pose(self):
        return self.spawn_request.x, self.spawn_request.y
    
    def master_turtle_pose_callback(self, master_turtle_pose):
        master_turtle_poseXY = master_turtle_pose.x, master_turtle_pose.y
        self.set_master_turtle_poseXY(master_turtle_poseXY)
        return
    
    def set_master_turtle_poseXY(self, master_turtle_poseXY):
        self.master_turtle_poseXY = master_turtle_poseXY
        return
    
    def get_master_turtle_poseXY(self):
        return self.master_turtle_poseXY
    
    def update_alive_turtles(self, alive_turtles):
        self.alive_turtles = alive_turtles
        return

    def get_alive_turtles(self):
        return self.alive_turtles
    
    def determine_turtle_distance(self, master_pose:tuple, slave_pose:tuple):
        masterX, masterY = master_pose
        slaveX, slaveY = slave_pose
        distXY = masterX - slaveX, masterY-slaveY
        distance = math.sqrt(distXY[0]**2 + distXY[1]**2)
        return distance


def main(args=None):
    print('Hi from turtlesim_catch_them_all.')
    rclpy.init(args=args)
    turtle_spawner_node = TurtleManager()
    rclpy.spin(turtle_spawner_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
