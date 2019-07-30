cmake_minimum_required(VERSION 2.8.3)
project(icra20)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(CMAKE_CXX_FLAGS "-std=c++11"
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  message_generation
  pcl_conversions
  pcl_ros
  tf
  unavlib
  sensor_msgs
  cv_bridge
  image_transport
  laser_geometry

)

add_message_files(
  FILES
  synced_node.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mcl_cpp
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
#  ${PCL_INCLUDE_DIRS}
  src
)
set(dataSyncSource
    src/data_synchronizer.cpp
    src/data_synchronizer.h
)

set(dataSaveSource
    src/data_saver.cpp
    src/data_saver.h
)

#add_executable(laser_to_pc srcipts/laser2pc.py)
#target_link_libraries(laser_to_pc ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(pc2_to_pclxyz src/pc2_to_pclxyz.cpp ${dataSyncSource})
target_link_libraries(pc2_to_pclxyz ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(pc2_to_pclxyz unavlib icra20_generate_messages_cpp)

add_executable(pclxyz_to_save src/pclxyz_to_save.cpp ${dataSaveSource})
target_link_libraries(pclxyz_to_save ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(pclxyz_to_save unavlib icra20_generate_messages_cpp)

add_executable(save_image src/save_image.cpp)
target_link_libraries(save_image ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(save_image unavlib icra20_generate_messages_cpp)

#add_executable(laser2PointCloud scripts/laser2pc.py)
#target_link_libraries(laser2PointCloud ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(preprocessor src/data_preprocessor.cpp)
#target_link_libraries(perprocessor ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(cpp_test src/cpp_example.cpp)
#target_link_libraries(cpp_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(cpp_scribble src/cpp_scribble.cpp)
#target_link_libraries(cpp_scribble ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
