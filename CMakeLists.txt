cmake_minimum_required(VERSION 2.8.3)
project(moveit_grasps)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(Boost REQUIRED thread system)
find_package(OpenMP)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  eigen_conversions
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_core
  moveit_visual_tools
  rosparam_shortcuts
  tf
  tf_conversions
  trajectory_msgs
  message_generation
  geometry_msgs
  actionlib
  actionlib_msgs
  moveit_msgs
)

add_message_files(DIRECTORY msg FILES
  GraspGeneratorOptions.msg
)

add_action_files(DIRECTORY action FILES
  GenerateGrasps.action
)

generate_messages(DEPENDENCIES
  geometry_msgs
  actionlib_msgs
  std_msgs
  moveit_msgs
)

# Catkin
catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
    ${PROJECT_NAME}_filter
  CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    moveit_msgs
    trajectory_msgs
    std_msgs
    message_runtime
    moveit_visual_tools
    rosparam_shortcuts
  INCLUDE_DIRS
    include
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Grasp Library
add_library(${PROJECT_NAME}
  src/grasp_generator.cpp
  src/grasp_data.cpp
  src/grasp_candidate.cpp
  src/grasp_scorer.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

# Grasp Filter Library
add_library(${PROJECT_NAME}_filter
  src/grasp_filter.cpp
  src/grasp_planner.cpp
)
target_link_libraries(${PROJECT_NAME}_filter
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
set_target_properties(${PROJECT_NAME}_filter PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}") # for threading
set_target_properties(${PROJECT_NAME}_filter PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")

# Test filter executable
add_executable(${PROJECT_NAME}_filter_test src/test/grasp_filter_test.cpp)
target_link_libraries(${PROJECT_NAME}_filter_test
  ${PROJECT_NAME} ${PROJECT_NAME}_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

# Test grasp executable
add_executable(${PROJECT_NAME}_test src/test/grasp_generator_test.cpp)
target_link_libraries(${PROJECT_NAME}_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Test mesh bounding box
add_executable(bounding_box_test src/test/mesh_bounding_box_test.cpp)
target_link_libraries(bounding_box_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Test grasp data settings
add_executable(grasp_poses_visualizer src/grasp_poses_visualizer.cpp)
target_link_libraries(grasp_poses_visualizer
  ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

## Install ---------------------------------------------------------------

# Install libraries
install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_filter
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install shared resources
install(DIRECTORY launch    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY resources DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install executables
install(TARGETS
  ${PROJECT_NAME}_test
  ${PROJECT_NAME}_filter_test
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
