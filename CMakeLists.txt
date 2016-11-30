cmake_minimum_required(VERSION 2.8.3)
project(erle_joy_operation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros
  mavros_msgs
 )

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp mavros mavros_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

set(H_FILES
    include/joystick.hh
    include/Node.h
    include/ErleJoyOperationNode.h
)

set(CPP_FILES
    src/joystick.cc
    src/Node.cpp
    src/ErleJoyOperationNode.cpp
)

add_executable(erle_joy_operation_node src/erle_joy_operation_node.cpp ${CPP_FILES})
add_dependencies(erle_joy_operation_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(erle_joy_operation_node ${catkin_LIBRARIES})
