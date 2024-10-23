cmake_minimum_required(VERSION 3.0.2)
project(xcon_to_mmacs)

SET(Boost_USE_STATIC_LIBS ON)
find_package(Boost 1.83 COMPONENTS log log_setup filesystem system thread REQUIRED)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
  turtlesim
  tf
  nav_msgs
  sensor_msgs
 
)



catkin_package(
#  INCLUDE_DIRS include
# LIBRARIES xcon_to_mmacs
#  CATKIN_DEPENDS joy roscpp turtlesim
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
  /home/user/catkin_ws/src/xcon_to_mmacs/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../helpers/scripts
  /home/user/catkin_ws/src/xcon_to_mmacs/helpers/scripts
)



set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC" )
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC" )


add_executable(xctomm src/xctomm.cpp)
add_executable(fake_odom src/fake_robot.cpp)
add_executable(cmdvel_driver src/cmdvel_driver.cpp)
add_executable(joy src/joy.cpp)



target_link_libraries(xctomm ${catkin_LIBRARIES}
  ${CMAKE_CURRENT_SOURCE_DIR}/../xcon_to_mmacs/lib/libZbCom.so
  ${Boost_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT})
 
target_link_libraries(fake_odom ${catkin_LIBRARIES})

target_link_libraries(joy ${catkin_LIBRARIES}
   ${CMAKE_CURRENT_SOURCE_DIR}/../xcon_to_mmacs/lib/libZbCom.so
   ${Boost_LIBRARIES}
   ${CMAKE_THREAD_LIBS_INIT}
   )
  
target_link_libraries(cmdvel_driver ${catkin_LIBRARIES}
   ${CMAKE_CURRENT_SOURCE_DIR}/../xcon_to_mmacs/lib/libZbCom.so
   ${Boost_LIBRARIES}
   ${CMAKE_THREAD_LIBS_INIT}
   )
  

