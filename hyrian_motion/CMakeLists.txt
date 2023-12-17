cmake_minimum_required(VERSION 3.5)
project(hyrian_motion)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_srvs REQUIRED)


find_package(rosidl_default_generators REQUIRED)

find_package(dynamixel_sdk REQUIRED)
find_package(hyrian_interfaces REQUIRED)
find_package(dynamixel_sdk_custom_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SetMotorPWM.msg"
  "msg/MotorPWM.msg"
)
include_directories(include)

add_executable(read_write_node src/read_write_node.cpp)
ament_target_dependencies(read_write_node rclcpp std_msgs std_srvs dynamixel_sdk_custom_interfaces hyrian_interfaces dynamixel_sdk)

install(TARGETS
  read_write_node

  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
