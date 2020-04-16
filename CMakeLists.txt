cmake_minimum_required(VERSION 2.8.3)
project(mechknownet)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  sensor_msgs
  pcl_ros
  roscpp
  rospy
  std_msgs
  message_generation
  image_transport
  )
find_package(OpenCV 3 REQUIRED PATHS "/home/himlen/opencv3/opencv/build")
#find_package(OpenCV REQUIRED) #may also work for 1.8
find_package(PCL 1.7 REQUIRED) #may also work for 1.8

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})


 add_service_files(
   FILES
   mechknownetdetect.srv
   mechknownetsys.srv
   mechknownetsendtf.srv
   mrcnn.srv
   lookaround.srv
)

 generate_messages(
   DEPENDENCIES
   std_msgs  
   sensor_msgs
   )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES relational_affordance
CATKIN_DEPENDS roscpp message_runtime
)

add_executable(system src/system.cpp)
target_link_libraries(system ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
