cmake_minimum_required(VERSION 2.8.3)
project(jaguar4x4wheel_control)

find_package(catkin REQUIRED COMPONENTS roslaunch)

file(GLOB_RECURSE include_files "${CMAKE_SOURCE_DIR}/config/*")
file(GLOB_RECURSE launch_files "${CMAKE_SOURCE_DIR}/launch/*")
list(APPEND extra_files ${include_files} ${launch_files})

add_custom_target(jaguar4x4wheel_control SOURCES ${extra_files})

catkin_package()

roslaunch_add_file_check(launch)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
