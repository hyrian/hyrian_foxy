################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(hyrian_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(hyrian_interfaces REQUIRED)

# find_package(rosidl_default_generators REQUIRED)

#   rosidl_generate_interfaces(${PROJECT_NAME}
#   "srv/Battery.srv"
#   "srv/Calg.srv"
#   "srv/Color.srv"
#   "srv/Onoff.srv"
#   "srv/ResetOdom.srv"
#   "srv/SaveColor.srv"
#   ) #이거 계속 빌드에러 걸려서 일단 잠시 주석처리

################################################################################
# Install
################################################################################
install(
  DIRECTORY launch param config
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
