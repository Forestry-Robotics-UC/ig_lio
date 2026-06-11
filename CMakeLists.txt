cmake_minimum_required(VERSION 3.8)
project(ig_lio)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS} -Wall")

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_ros REQUIRED)           # optional, keep if needed
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Optional: Livox support. Only enabled if the ROS 2 Livox driver is present in
# the workspace (it is NOT a hard dependency, so rosdep works without it). When
# found, HAVE_LIVOX is defined and the Livox CustomMsg path is compiled in.
find_package(livox_ros_driver2 QUIET)

# System dependencies
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(TBB REQUIRED)

include(cmake/glog.cmake)

# Sophus (header-only third-party)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/sophus)

include_directories(include)

add_executable(${PROJECT_NAME}_node
  src/ig_lio_node.cpp
  src/pointcloud_preprocess.cpp
  src/lio.cpp
  src/voxel_map.cpp
  src/SymmetricEigenSolver.cpp
  src/timer.cpp
  src/faster_voxel_grid.cpp
)

# Bake the source dir in so the node can default its trajectory output to the
# package's own result/ directory (persists across rebuilds, unlike share).
target_compile_definitions(${PROJECT_NAME}_node PRIVATE
  IG_LIO_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# Use ament_target_dependencies for all ROS packages.
# This automatically adds include directories, libraries, and compile definitions.
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  pcl_ros
  pcl_conversions
  sensor_msgs
  nav_msgs
)

# Optional Livox support (see find_package above).
if(livox_ros_driver2_FOUND)
  message(STATUS "ig_lio: livox_ros_driver2 found -- Livox support ENABLED (lidar_type: livox)")
  target_compile_definitions(${PROJECT_NAME}_node PRIVATE HAVE_LIVOX)
  ament_target_dependencies(${PROJECT_NAME}_node livox_ros_driver2)
else()
  message(STATUS "ig_lio: livox_ros_driver2 NOT found -- Livox support DISABLED. "
                 "Install it in your workspace and rebuild to enable lidar_type: livox.")
endif()

# Link remaining non‑ROS dependencies
target_link_libraries(${PROJECT_NAME}_node
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  glog
  gflags
  TBB::tbb
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

ament_package()