cmake_minimum_required(VERSION 3.5)
project(hyrian_navigation2)

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
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED) 
find_package(tf2_geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(movebase src/movebase.cpp)
ament_target_dependencies(movebase rclcpp rclcpp_action nav2_msgs geometry_msgs std_msgs std_srvs tf2_geometry_msgs)

add_executable(gesture src/gesture.cpp)  
ament_target_dependencies(gesture rclcpp std_srvs)  

install(
  DIRECTORY launch map param rviz src
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  movebase
  gesture
  DESTINATION lib/${PROJECT_NAME}
)
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()