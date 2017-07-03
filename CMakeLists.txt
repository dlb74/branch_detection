cmake_minimum_required(VERSION 2.8.3)
project(branch_detection)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
   pcl_conversions
   pcl_ros
)

add_message_files(
  FILES
  Num.msg
)

add_service_files(
  FILES
  AddTwoInts.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES})
add_dependencies(main branch_detection_gencpp)

add_executable(test_client src/test_client.cpp)
target_link_libraries(test_client ${catkin_LIBRARIES})
add_dependencies(test_client branch_detection_gencpp)
