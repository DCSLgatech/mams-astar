cmake_minimum_required(VERSION 2.8.3)
project(mams_astar)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  std_msgs
  roscpp
  cmake_modules
  message_generation
)
add_message_files(
  FILES
  Cuckoo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

#find_package(Eigen REQUIRED)

catkin_package(
#  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
#  LIBRARIES ToDo main
#  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs roscpp
#  DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
#  ${Eigen_INCLUDE_DIRS}
)



set(MULTISCALE_SOURCE_FILES

  src/nodes/agent_node.cpp
)

set(MULTISCALE_HEADER_FILES

  src/nodes/agent_node.h
)

set(KSHORTEST_HEADER_FILES
  include/kshortestpaths/Graph.h
  include/kshortestpaths/GraphElements.h

)

set(KSHORTEST_SOURCE_FILES
  src/kshortestpaths/Graph.cpp

)


# build!
add_executable(multiscale ${MULTISCALE_SOURCE_FILES} ${MULTISCALE_HEADER_FILES}
			  ${KSHORTEST_SOURCE_FILES} ${KSHORTEST_HEADER_FILES})
target_link_libraries(multiscale ${catkin_LIBRARIES})


add_dependencies(multiscale ${catkin_EXPORTED_TARGETS})
