cmake_minimum_required(VERSION 3.5)
project(mav_control_interface)
find_package(ament_cmake REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
set(CMAKE_BUILD_TYPE Release) # Usually done by catkin_simple

set(SOURCE_FILES
  src/mav_control_interface.cpp
  src/mav_control_interface_impl.cpp
  src/parameters.cpp
  src/position_controller_interface.cpp
  src/rc_interface.cpp
  src/rc_interface_aci.cpp
  src/state_machine.cpp
  src/mpc_queue.cpp
)

# find_package(catkin REQUIRED) # catkin_simple
 # catkin Manually add deps from package.xml

# find_package(catkin_simple REQUIRED)
# catkin_simple()

add_definitions(-std=c++11)

include_directories(${Eigen_INCLUDE_DIRS})
include_directories(include)

#cs_add_library(${PROJECT_NAME} ${SOURCE_FILES}) # catkin_simple
add_library(${PROJECT_NAME} ${SOURCE_FILES}) # catkin
ament_target_dependencies(${PROJECT_NAME} "rclcpp" "tf2_ros" "tf_conversions" "visualization_msgs" "sensor_msgs" "nav_msgs" "std_srvs" "trajectory_msgs" "std_msgs")

#cs_add_executable(${PROJECT_NAME}_test_node src/test_node.cpp)
add_executable(${PROJECT_NAME}_test_node src/test_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_test_node ${PROJECT_NAME})

#cs_install()
#cs_export()
ament_package() # catkin
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DBOOST_MPL_CFG_NO_PREPROCESSED_HEADERS -DBOOST_MPL_LIMIT_VECTOR_SIZE=30 -DBOOST_MPL_LIMIT_MAP_SIZE=30 ")
