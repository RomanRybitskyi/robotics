cmake_minimum_required(VERSION 3.5)
project(joint_state_publisher)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add the executable
add_executable(joint_state_publisher src/joint_state_publisher.cpp)
ament_target_dependencies(joint_state_publisher rclcpp sensor_msgs)

# Install the executable
install(TARGETS joint_state_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Enable testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

