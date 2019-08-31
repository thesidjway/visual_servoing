cmake_minimum_required(VERSION 2.8.9)
project(uav_visual_servoing)
set (CMAKE_CXX_STANDARD 11)

find_package (catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  std_msgs 
  )

catkin_package(
#	INCLUDE_DIRS include
	LIBRARIES ugv_visual_servoing
	CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge    
)

include_directories(
	include/
	Eigen
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${ROS_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package (Ceres REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (OpenCV REQUIRED)
find_package (PCL REQUIRED COMPONENTS common io)


add_executable(ugv_visual_servoing src/VisualServoing.cpp)
target_link_libraries(ugv_visual_servoing ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp)
