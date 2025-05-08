#!/usr/bin/env python3.10
import math
import rclpy
import random
from rclpy.node import Node

from turtlesim.srv import Kill
from turtlesim.srv import Spawn
from turtlesim.msg import Pose

class TurtleEater(Node):
    def __init__(self):
        super().__init__("Turtle_eater")
        self.create_subscription(Pose, "turtle1/pose", self.master_turtle_pose_callback, 10)
        self.turtle_spawner = self.create_client(Spawn, "spawn")
        self.create_timer(2.0, self.turtle_spawner_callback)
        self.alive_turtles = []
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
        self.alive_turtles.append(response.name)
        return
    
    def master_turtle_pose_update_callback(self, pose):
        alive_turtles_ = self.alive_turtles
        for turtle_ in alive_turtles_:
            focused_turtle_topic = f"{turtle_}/pose"
            turtle_pose = self.create_subscription(Pose, focused_turtle_topic, )


def main(args=None):
    print('Hi from turtlesim_catch_them_all.')
    rclpy.init(args=args)
    turtle_spawner_node = TurtleManager()
    rclpy.spin(turtle_spawner_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()