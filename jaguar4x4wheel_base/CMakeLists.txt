cmake_minimum_required(VERSION 2.8.3)
project(jaguar4x4wheel_base)

find_package(catkin REQUIRED COMPONENTS controller_manager hardware_interface
  roslaunch roslint  roscpp sensor_msgs)
find_package(Boost REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS hardware_interface roscpp sensor_msgs
    DEPENDS Boost
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(jaguar4x4wheel_node
  src/jaguar4x4wheel_base.cpp
  src/jaguar4x4wheel_hardware.cpp
  src/DrRobotMotionSensorDriver.cpp)
target_link_libraries(jaguar4x4wheel_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

roslint_cpp(
  src/jaguar4x4wheel_base.cpp
  src/jaguar4x4wheel_hardware.cpp
  src/DrRobotMotionSensorDriver.cpp
  )

roslaunch_add_file_check(launch)


#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB children ${CMAKE_SOURCE_DIR}/*)
FOREACH(child ${children})
  IF(IS_DIRECTORY ${child})
    file(GLOB_RECURSE dir_files "${child}/*")
    LIST(APPEND extra_files ${dir_files})
  ENDIF()
ENDFOREACH()
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})

install(TARGETS jaguar4x4wheel_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
