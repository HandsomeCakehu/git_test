cmake_minimum_required(VERSION 2.8.3)
project(obstacle_detection)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cv_bridge
  dynamic_reconfigure
  image_transport
  message_generation
  pcl_conversions
  pcl_ros
  roscpp
  std_msgs
  sensor_msgs
  jsk_recognition_msgs
  tf
  mytopic_ob
)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 4.5.4 REQUIRED)

#添加OpenCV头文件
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros std_msgs jsk_recognition_msgs
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

add_executable(${PROJECT_NAME}_node src/Obstacle_detection_node.cpp src/Obstacle_detection_core.cpp  src/fusion.cpp src/yolo.cpp src/socket.cpp)


target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)
add_dependencies(${PROJECT_NAME}_node  mytopic_ob_gencpp)