cmake_minimum_required(VERSION 3.0.2)
project(beginner_tutorials)
## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp tf rospy std_msgs genmsg)

## Declare ROS messages and services
add_message_files(FILES Num.msg)
add_service_files(FILES AddTwoInts.srv string.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(
  pub_test
  test/test_pub.launch
  test/pub_test.cpp
  )
  target_link_libraries(
  pub_test
  ${catkin_LIBRARIES}
  )
  add_dependencies(
  pub_test
  talker
  ${catkin_EXPORTED_TARGETS}
  )
endif()
