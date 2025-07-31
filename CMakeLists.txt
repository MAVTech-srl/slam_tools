cmake_minimum_required(VERSION 3.8)
project(slam_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PDAL REQUIRED)
find_package(PROJ REQUIRED CONFIG)
find_package(message_filters REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(livox_interfaces REQUIRED)

message(STATUS "PCL found (headers: ${PCL_INCLUDE_DIRS}, libraries: ${PCL_LIBRARY_DIRS})")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# message(STATUS "PDAL found (headers: ${PDAL_INCLUDE_DIRS}, libraries: ${PDAL_LIBRARY_DIRS})")
# include_directories(${PDAL_INCLUDE_DIRS})
# link_directories(${PDAL_LIBRARY_DIRS})
# add_definitions(${PDAL_DEFINITIONS})

include_directories(include/slam_tools)

add_executable(pointcloud2UTMpcd src/pointcloud2UTMpcd.cpp)
ament_target_dependencies(pointcloud2UTMpcd rclcpp sensor_msgs pcl_conversions mavros_msgs nav_msgs geometry_msgs tf2)

add_executable(pointcloud2pcd src/pointcloud2pcd.cpp)
ament_target_dependencies(pointcloud2pcd rclcpp sensor_msgs pcl_conversions message_filters mavros_msgs nav_msgs geometry_msgs tf2)

add_executable(livox2pointcloud src/livox2pointcloud.cpp)
ament_target_dependencies(livox2pointcloud rclcpp sensor_msgs livox_interfaces pcl_conversions mavros_msgs nav_msgs geometry_msgs tf2)

target_link_libraries(pointcloud2UTMpcd ${PCL_LIBRARIES} PROJ::proj)
target_link_libraries(pointcloud2pcd ${PCL_LIBRARIES})
target_link_libraries(livox2pointcloud ${PCL_LIBRARIES})

target_include_directories(pointcloud2UTMpcd PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pointcloud2UTMpcd PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS pointcloud2UTMpcd
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS pointcloud2pcd
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS livox2pointcloud
  DESTINATION lib/${PROJECT_NAME})

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
