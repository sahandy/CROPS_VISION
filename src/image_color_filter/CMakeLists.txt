cmake_minimum_required(VERSION 2.8.3)
project(rgb_image_processing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  std_msgs
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
add_executable(hsv_filter src/image_grabber.cpp src/HSVFilter.hpp)
# Specify libraries to link a lib or exec. target against
target_link_libraries(hsv_filter ${catkin_LIBRARIES} ${OpenCV_LIBS})


