cmake_minimum_required(VERSION 3.5)
project(topic_subsub)

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
find_package(std_msgs REQUIRED)

# Add the executable target
add_executable(topic_subsub src/topic_sub.cpp)  # 수정된 부분

# Link the target with the required libraries
ament_target_dependencies(topic_subsub rclcpp std_msgs)  # 수정된 부분

#Install the executable
install(TARGETS
  topic_subsub  # 수정된 부분
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Don't forget to export the dependencies
ament_export_dependencies(rclcpp std_msgs)

ament_package()
