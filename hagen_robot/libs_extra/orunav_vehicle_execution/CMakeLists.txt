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
  orunav_rviz
  tf
  tf2_ros
  laser_geometry
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
  CATKIN_DEPENDS roscpp std_msgs tf2_ros std_srvs orunav_rviz orunav_motion_planner nav_msgs orunav_msgs orunav_generic orunav_geometry tf laser_geometry
  DEPENDS Boost
)

find_package(ACADO REQUIRED)
if(NOT ACADO_FOUND)
  message(FATAL_ERROR " ACADO *not* found")
endif()

# Added own variable containing the full path to the library.
set(ACADO_LFLAGS -L${ACADO_LIBRARY_DIRS} -l${ACADO_SHARED_LIBRARIES})
message("  - ACADO_LFLAGS : " ${ACADO_LFLAGS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${ACADO_INCLUDE_DIRS}
)

add_executable(hagen_vehicle_execution_node src/hagen_vehicle_execution_node.cpp src/path_planner.cpp src/path_smoother_dynamic.cpp)
add_dependencies(hagen_vehicle_execution_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(hagen_vehicle_execution_node
   ${catkin_LIBRARIES}  ${ACADO_LFLAGS}
 )

install(TARGETS hagen_vehicle_execution_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

install(DIRECTORY
  targets
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
