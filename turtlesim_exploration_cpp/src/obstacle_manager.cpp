#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <random>

#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim_exploration_interfaces/msg/obstacle.hpp"
#include "turtlesim_exploration_interfaces/msg/obstacle_array.hpp"

class ObstacleManagerNode : public rclcpp::Node
{
public:
    ObstacleManagerNode() : Node("obstacle_manager"), active_obstacles({}), turtle_count(2)
    {
        publisher_ = this->create_publisher<turtlesim_exploration_interfaces::msg::ObstacleArray>("/obstacle_turtles", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(2),
        std::bind(&ObstacleManagerNode::spawnObstacle, this));
        RCLCPP_INFO(this->get_logger(), "Obstacle Manager has started.");
    }   

private: 
    void spawnObstacle() {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dis(0.0, 11.0);
        std::uniform_real_distribution<float> dis2(0.0, 360.0);
        
        float obstacle_x, obstacle_y, obstacle_theta;
        auto client = this->create_client<turtlesim::srv::Spawn>("/spawn");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for server to be up...");
        }

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        obstacle_x = dis(gen);
        obstacle_y = dis(gen);
        obstacle_theta = dis2(gen);

        request->x = obstacle_x;
        request->y = obstacle_y;
        request->theta = obstacle_theta;
        request->name = "turtle" + std::to_string(turtle_count);

        active_obstacles[request->name] = {request->x, request->y};
        turtle_count += 1;

        auto array = std::make_shared<turtlesim_exploration_interfaces::msg::ObstacleArray>();
        for (const auto& [key, value] : active_obstacles) {
            turtlesim_exploration_interfaces::msg::Obstacle msg;
            msg.name = key;
            msg.x = active_obstacles[key].first;
            msg.y = active_obstacles[key].second;
            array->obstacles.push_back(msg);
        }
        publisher_->publish(*array);

        auto future = client->async_send_request(request);
    }
    rclcpp::Publisher<turtlesim_exploration_interfaces::msg::ObstacleArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::map<std::string, std::pair<float, float>> active_obstacles;
    int turtle_count;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

