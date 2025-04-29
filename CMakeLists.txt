cmake_minimum_required(VERSION 3.8)
project(navi_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


include_directories(
  include
  include/common
)


set (
  DEPENDENCY_LIST
  rclcpp
  geometry_msgs
  nav_msgs
  unitree_go
  unitree_api
  std_msgs
  rosbag2_cpp
)


add_executable(simple_navi_node src/simple_navi_node.cpp)
ament_target_dependencies(simple_navi_node rclcpp)

add_executable(simple_path_follower src/simple_path_follower.cpp src/common/ros2_sport_client.cpp)
ament_target_dependencies(simple_path_follower ${DEPENDENCY_LIST})


install(TARGETS
  simple_path_follower
  DESTINATION lib/${PROJECT_NAME})

# install launch files
# 실행파일 설치
install(TARGETS
  simple_navi_node
  DESTINATION lib/${PROJECT_NAME}
)

# 런치파일 설치
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
