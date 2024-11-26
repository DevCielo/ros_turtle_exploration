cmake_minimum_required(VERSION 3.8)
project(turtlesim_exploration_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim_exploration_interfaces REQUIRED)

add_executable(obstacle_manager src/obstacle_manager.cpp)
ament_target_dependencies(obstacle_manager rclcpp turtlesim geometry_msgs turtlesim_exploration_interfaces)

install(TARGETS
  obstacle_manager
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
