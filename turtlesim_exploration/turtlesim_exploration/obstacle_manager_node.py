#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import random

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
from turtlesim_exploration_interfaces.msg import Obstacle, ObstacleArray

class ObstacleManagerNode(Node):
    def __init__(self):
        super().__init__("obstacle_manager")
        self.active_obstacles = {}
        self.turtle_count = 2
        # self.active_obstacles[name] = {x: "x", y: "y"} 
        self.timer_ = self.create_timer(2.5, self.spawn_obstacle)
        self.publisher_ = self.create_publisher(ObstacleArray, "/obstacle_turtles", 10)
        self.get_logger().info("Obstacle Manager has started")
    
    def spawn_obstacle(self):
        client = self.create_client(Spawn, "/spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server...")
            
        request = Spawn.Request()
        obstacle_x = random.uniform(0.0, 11.0)
        obstacle_y = random.uniform(0.0, 11.0)
        obstacle_theta = random.uniform(0.0, 360.0)
        
        request.x = obstacle_x
        request.y = obstacle_y
        request.theta = obstacle_theta
        request.name = "turtle" + str(self.turtle_count)
        self.active_obstacles[request.name] = {"x": request.x, "y": request.y}
        self.turtle_count += 1
        
        array = ObstacleArray()
        for key, value in self.active_obstacles.items():
            msg = Obstacle()
            msg.name = key
            msg.x = value["x"]
            msg.y = value["y"]
            array.obstacles.append(msg)
        self.publisher_.publish(array)
        
        future = client.call_async(request)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleManagerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
