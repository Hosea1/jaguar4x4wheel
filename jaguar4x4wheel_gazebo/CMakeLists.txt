cmake_minimum_required(VERSION 2.8.3)
project(jaguar4x4wheel_gazebo)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()


file(GLOB_RECURSE include_files "${CMAKE_SOURCE_DIR}/urdf/*")
file(GLOB_RECURSE launch_files "${CMAKE_SOURCE_DIR}/launch/*")
file(GLOB_RECURSE worlds_files "${CMAKE_SOURCE_DIR}/worlds/*")
list(APPEND extra_files ${include_files} ${launch_files} ${worlds_files})

add_custom_target(jaguar4x4wheel_description SOURCES ${extra_files})

roslaunch_add_file_check(launch)

install(
  DIRECTORY launch worlds urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
