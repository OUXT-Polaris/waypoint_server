cmake_minimum_required(VERSION 2.8.3)
project(waypoint_server)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rostate_machine
  visualization_msgs
  usv_navigation_msgs
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES waypoint_server
  CATKIN_DEPENDS roscpp rostate_machine usv_navigation_msgs visualization_msgs tf2_ros tf2_geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(waypoint_server_node src/waypoint_server_node.cpp src/waypoint_server.cpp src/waypoint_parser.cpp src/waypoint.cpp)
target_link_libraries(waypoint_server_node ${catkin_LIBRARIES})
add_dependencies(waypoint_server_node rostate_machine_generate_messages_cpp usv_navigation_msgs_generate_messages_cpp)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install library/executable
install(TARGETS waypoint_server_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

foreach(dir launch data)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)