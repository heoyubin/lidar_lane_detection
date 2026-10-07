cmake_minimum_required(VERSION 3.8)
project(lidar_lane_detector)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(lidar_lane_detector
  src/lidar_lane_detector.cpp
  src/lidar_lane_detector_node.cpp
)

ament_target_dependencies(lidar_lane_detector
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  pcl_ros
  pcl_conversions
)

target_link_libraries(lidar_lane_detector
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
)
	
target_include_directories(lidar_lane_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS lidar_lane_detector
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

