cmake_minimum_required(VERSION 2.8.3)
project(mav_control_interface)

# find_package(catkin REQUIRED) # catkin_simple
find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp mav_msgs nav_msgs std_msgs std_srvs tf tf_conversions visualization_msgs) # catkin Manually add deps from package.xml

# find_package(catkin_simple REQUIRED)
# catkin_simple()

add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(Eigen3 REQUIRED)

catkin_package(

)

include_directories(${Eigen_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS}) # Usually done by catkin_simple

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

#cs_add_library(${PROJECT_NAME} ${SOURCE_FILES}) # catkin_simple
add_library(${PROJECT_NAME} ${SOURCE_FILES}) # catkin
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}) # catkin
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DBOOST_MPL_CFG_NO_PREPROCESSED_HEADERS -DBOOST_MPL_LIMIT_VECTOR_SIZE=30 -DBOOST_MPL_LIMIT_MAP_SIZE=30 ")


#cs_add_executable(${PROJECT_NAME}_test_node src/test_node.cpp)
add_executable(${PROJECT_NAME}_test_node src/test_node.cpp)
target_link_libraries(${PROJECT_NAME}_test_node ${PROJECT_NAME})

#cs_install()
#cs_export()
