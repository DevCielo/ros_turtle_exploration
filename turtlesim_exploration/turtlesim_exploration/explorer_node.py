#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim_exploration_interfaces.msg import ObstacleArray


class TurtleExplorerNode(Node):
    def __init__(self):
        super().__init__("turtle_explorer")
        self.current_x = None
        self.current_y = None
        self.current_theta = None
        self.goal_x = 10.0
        self.goal_y = 10.0
        self.obstacles = []

        self.create_subscription(Pose, "/turtle1/pose", self.callback_monitor_position, 10)
        self.create_subscription(ObstacleArray, "/obstacle_turtles", self.callback_obstacle_positions, 10)
        self.velocity_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.timer_ = self.create_timer(0.1, self.move_turtle)

        self.get_logger().info("Turtle Explorer Node has started")

    def callback_monitor_position(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.theta

    def callback_obstacle_positions(self, msg):
        self.obstacles = [{"x": obs.x, "y": obs.y} for obs in msg.obstacles]

    def move_turtle(self):
        if self.current_x is None or self.current_y is None or self.current_theta is None:
            return

        vel_msg = Twist()

        # Calculate the vector towards the goal
        distance_to_goal = math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)
        if distance_to_goal <= 0.1:
            self.get_logger().info("Goal reached!")
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.velocity_publisher.publish(vel_msg)
            return

        angle_to_goal = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)

        # Obstacle avoidance
        safe_distance = 1.0  # Threshold distance to avoid obstacles
        avoidance_vector_x = 0.0
        avoidance_vector_y = 0.0
        for obstacle in self.obstacles:
            distance = math.sqrt((self.current_x - obstacle["x"])**2 + (self.current_y - obstacle["y"])**2)
            if distance < safe_distance:
                # Calculate repulsion vector away from the obstacle
                repulsion_strength = (safe_distance - distance) / safe_distance
                avoidance_vector_x += repulsion_strength * (self.current_x - obstacle["x"]) / distance
                avoidance_vector_y += repulsion_strength * (self.current_y - obstacle["y"]) / distance

        # Combine the goal vector and avoidance vector
        combined_x = math.cos(angle_to_goal) + avoidance_vector_x
        combined_y = math.sin(angle_to_goal) + avoidance_vector_y

        # Calculate the resulting angle and linear velocity
        resulting_angle = math.atan2(combined_y, combined_x)
        vel_msg.linear.x = 0.5  # Move forward with a moderate speed
        vel_msg.angular.z = 2.0 * (resulting_angle - self.current_theta)

        # Normalize angular velocity to avoid excessive spinning
        if vel_msg.angular.z > math.pi:
            vel_msg.angular.z -= 2 * math.pi
        elif vel_msg.angular.z < -math.pi:
            vel_msg.angular.z += 2 * math.pi

        self.velocity_publisher.publish(vel_msg)

        if self.current_x is None or self.current_y is None or self.current_theta is None:
            return

        vel_msg = Twist()

        # Check for obstacles
        if self.detect_obstacle():
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 2.0  # Rotate to avoid
            self.get_logger().info("Avoiding obstacle")
        else:
            # Navigate towards the goal
            distance_to_goal = math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)
            if distance_to_goal > 0.1:
                angle_to_goal = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
                vel_msg.linear.x = 1.0
                vel_msg.angular.z = 2.0 * (angle_to_goal - self.current_theta)
            else:
                self.get_logger().info("Goal reached!")
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.0

            self.velocity_publisher.publish(vel_msg)

    def detect_obstacle(self):
        safe_distance = 1.0  # Threshold distance to avoid obstacles
        for obstacle in self.obstacles:
            distance = math.sqrt((self.current_x - obstacle["x"])**2 + (self.current_y - obstacle["y"])**2)
            if distance < safe_distance:
                return True
        return False


def main(args=None):
    rclpy.init(args=args)
    node = TurtleExplorerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
