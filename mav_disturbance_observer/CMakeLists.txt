cmake_minimum_required(VERSION 3.5)
project(mav_disturbance_observer)

find_package(catkin_simple REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)
set(CMAKE_CXX_STANDARD 1 4)
set(CMAKE_BUILD_TYPE Release)

add_definitions(-std=c++11)
rosidl_generate_interfaces(
    ${PROJECT_NAME}
        msg/ObserverState.msg
    DEPENDENCIES
        
)
include_directories(${Eigen_INCLUDE_DIRS})
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
cs_add_library(KFDisturbanceObserverLibrary
  src/KF_disturbance_observer.cpp
)

############
# BINARIES #
############

#No binaries, only libraries in this package!

##########
# EXPORT #
##########
cs_install()
cs_export()
