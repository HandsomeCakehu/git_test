cmake_minimum_required(VERSION 2.8.3)
project(pcl_test)

add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)


#添加OPENCV库
#指定OpenCV版本，代码如下

#如果不需要指定OpenCV版本，代码如下
#set(OpenCV_DIR "/usr/local/opencv4.5.4/share")   
find_package(OpenCV 4.5.4 REQUIRED)

#添加OpenCV头文件
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
pcl_ros
roscpp
sensor_msgs
)

find_package(PCL 1.7 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

add_executable(${PROJECT_NAME}_node src/pcl_test_node.cpp src/pcl_test_core.cpp)


target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)
