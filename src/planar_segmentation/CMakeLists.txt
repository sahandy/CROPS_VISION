cmake_minimum_required(VERSION 2.8.3)
project(planar_segmentation)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Declare the executable
add_executable(segment_planes src/plane_seg.cpp)
add_executable(find_largest_plane src/largest_plane_seg.cpp)
add_executable(remove_planes src/remove_planes.cpp)

# Specify libraries to link a lib or exec. target against
target_link_libraries(segment_planes ${catkin_LIBRARIES})
target_link_libraries(find_largest_plane ${catkin_LIBRARIES})
target_link_libraries(remove_planes ${catkin_LIBRARIES})


