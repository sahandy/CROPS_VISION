cmake_minimum_required(VERSION 2.8.3)
project(marker_pose_estimation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Declare the executable
add_executable(estimate_pose src/main.cpp)

# Specify libraries to link a lib or exec. target against
target_link_libraries(estimate_pose ${catkin_LIBRARIES})
