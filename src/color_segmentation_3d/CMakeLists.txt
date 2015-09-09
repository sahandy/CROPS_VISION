cmake_minimum_required(VERSION 2.8.3)
project(color_segmentation_3d)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  cv_bridge
  roscpp
  sensor_msgs
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Declare the executable
add_executable(cloud_color_filter src/color_filter.cpp)

# Specify libraries to link a lib or exec. target against
target_link_libraries(cloud_color_filter ${catkin_LIBRARIES} ${OpenCV_LIBS})
