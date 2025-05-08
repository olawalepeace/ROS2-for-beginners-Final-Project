#!/usr/bin/env python3.10
import rclpy
from rclpy.node import Node

from turtlesim.srv import Spawn
from turtlesim.srv import Kill

from proj_interfaces.srv import EatTurtle
from proj_interfaces.srv import MoveToTurtle

import random

class TurtleManager(Node):
    def __init__(self):
        super().__init__("turtle_spawner")

        self.alive_turtles = []
        self.spawn_request = Spawn.Request()

        self.create_service(EatTurtle, "eat_turtle", self.eat_turtle_callback)
        self.turtle_spawner = self.create_client(Spawn, "spawn")
        self.kill_turtle = self.create_client(Kill, "kill")
        self.move_to_turtle = self.create_client(MoveToTurtle, "move_to_turtle")
        self.create_timer(0.8, self.turtle_spawner_callback)
        self.move_to_next_turtle()
        return

    def turtle_spawner_callback(self):
        poseXYTheta = random.random()*9 + 1, random.random()*9 + 1, random.random()*3.142
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
        if alive_turtles:
            for turtle in alive_turtles:
            # TODO - Add an algorithm to choose closest turtle to master turtle here
                pass
            next_turtle = alive_turtles[0]
            
            move_turtle = MoveToTurtle.Request()
            move_turtle.to_name= next_turtle["name"]
            move_turtle.to_x, move_turtle.to_y = next_turtle["pose"]
            self.move_to_turtle.call_async(move_turtle)
        else:
            self.waiting_timer = self.create_timer(0.1, self.waiting_callback)
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
    
    def update_alive_turtles(self, alive_turtles):
        self.alive_turtles = alive_turtles
        return
    
    def get_alive_turtles(self):
        return self.alive_turtles


def main(args=None):
    print('Hi from turtlesim_catch_them_all.')
    rclpy.init(args=args)
    turtle_spawner_node = TurtleManager()
    rclpy.spin(turtle_spawner_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
