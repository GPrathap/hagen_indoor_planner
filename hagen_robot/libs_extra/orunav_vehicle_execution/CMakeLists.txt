cmake_minimum_required(VERSION 2.8.3)
project(orunav_vehicle_execution)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  std_msgs
  std_srvs
  nav_msgs
  orunav_msgs
  orunav_generic
  orunav_geometry
  tf
  laser_geometry
  orunav_node_utils
  orunav_motion_planner
)

find_package(Boost REQUIRED COMPONENTS system thread program_options)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs std_srvs orunav_node_utils  orunav_motion_planner nav_msgs orunav_msgs orunav_generic orunav_geometry tf laser_geometry
  DEPENDS Boost
)

find_package(ACADO REQUIRED)
if(NOT ACADO_FOUND)
  message(FATAL_ERROR " ACADO *not* found")
endif()

# Added own variable containing the full path to the library.
set(ACADO_LFLAGS -L${ACADO_LIBRARY_DIRS} -l${ACADO_SHARED_LIBRARIES})
message("  - ACADO_LFLAGS : " ${ACADO_LFLAGS})


###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${ACADO_INCLUDE_DIRS}
)

add_executable(orunav_vehicle_execution_node src/orunav_vehicle_execution_node.cpp src/path_planner.cpp src/path_smoother_dynamic.cpp)
add_dependencies(orunav_vehicle_execution_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(orunav_vehicle_execution_node
   ${catkin_LIBRARIES}  ${ACADO_LFLAGS}
 )

# add_executable(point_n_click_target_client src/point_n_click_target_client.cpp)
# add_dependencies(point_n_click_target_client ${catkin_EXPORTED_TARGETS})
# target_link_libraries(point_n_click_target_client
#    ${catkin_LIBRARIES}
#    -lboost_program_options
#  )

# add_executable(send_brake_command src/send_brake_command_main.cpp)
# add_dependencies(send_brake_command ${catkin_EXPORTED_TARGETS})
# target_link_libraries(send_brake_command
#    ${catkin_LIBRARIES}
#    -lboost_program_options
#  )

# add_executable(vehicle_execution_debug_client src/vehicle_execution_debug_client.cpp)
# add_dependencies(vehicle_execution_debug_client ${catkin_EXPORTED_TARGETS})
# target_link_libraries(vehicle_execution_debug_client
#    ${catkin_LIBRARIES}
#    -lboost_program_options
#  )

# add_executable(vehicle_execution_client src/vehicle_execution_client.cpp)
# add_dependencies(vehicle_execution_client ${catkin_EXPORTED_TARGETS})
# target_link_libraries(vehicle_execution_client
#    ${catkin_LIBRARIES}
#    -lboost_program_options
#  )
# add_executable(point_n_click_target_publisher src/point_n_click_target_publisher.cpp)
# add_dependencies(point_n_click_target_publisher ${catkin_EXPORTED_TARGETS})
# target_link_libraries(point_n_click_target_publisher
#    ${catkin_LIBRARIES}
#    -lboost_program_options
#  )

# add_executable(send_abort_task src/send_abort_task_main.cpp)
# add_dependencies(send_abort_task ${catkin_EXPORTED_TARGETS})
# target_link_libraries(send_abort_task
#    ${catkin_LIBRARIES}
#    -lboost_program_options
#  )

# add_executable(laser_filter src/laser_filter_main.cpp)
# add_dependencies(laser_filter ${catkin_EXPORTED_TARGETS})
# target_link_libraries(laser_filter
#    ${catkin_LIBRARIES}
#    -lboost_program_options
#  )

# add_executable(generate_random_targets src/generate_random_targets_main.cpp)
# add_dependencies(generate_random_targets ${catkin_EXPORTED_TARGETS})
# target_link_libraries(generate_random_targets
#    ${catkin_LIBRARIES}
#    -lboost_program_options
#  )

#############
## Install ##
#############

install(TARGETS orunav_vehicle_execution_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

install(DIRECTORY
  targets
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
