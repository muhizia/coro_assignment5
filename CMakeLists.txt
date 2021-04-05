##################################################################################
#
# Customize this file  by replacing amuhizi with your own student ID:
# there are six replacements to be made.
#
# Make sure you have done the following:
#
# - created a package named assignment4
#   and that you use this make file in that package
#
# - created a subdirectory named data in the package
#   and that you have created the input file assignment4Input.txt there
#
# - created the interface file amuhizi.h in the include/assignment4 subdirectory
#
# - created the application source file amuhiziApplication.cpp
#   and the implementation source file  amuhiziImplementation.cpp
#   in the src subdirectory
#
##################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(assignment5)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

FIND_PACKAGE(OpenCV 3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  roslib
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
  )

# Install data files
install(DIRECTORY data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
  )

add_executable       (${PROJECT_NAME}_amuhizi src/amuhiziImplementation.cpp src/amuhiziApplication.cpp)
set_target_properties(${PROJECT_NAME}_amuhizi PROPERTIES OUTPUT_NAME amuhizi  PREFIX "")
target_link_libraries(${PROJECT_NAME}_amuhizi ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

