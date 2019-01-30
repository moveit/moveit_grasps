cmake_minimum_required(VERSION 2.8.3)
project(moveit_grasps)

# C++ 11
add_compile_options(-std=c++11)

# Warnings
add_definitions(-W -Wall -Wextra
  -Wwrite-strings -Wunreachable-code -Wpointer-arith
  -Winit-self -Wredundant-decls
  -Wno-unused-parameter -Wno-unused-function)

# System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED thread system)
find_package(OpenMP)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  eigen_conversions
  geometry_msgs
  message_generation
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  roscpp
  roslint
  rosparam_shortcuts
  std_msgs
  tf
  tf_conversions
  trajectory_msgs
)

add_message_files(DIRECTORY msg FILES
  GraspGeneratorOptions.msg
)

add_action_files(DIRECTORY action FILES
  GenerateGrasps.action
)

generate_messages(DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  moveit_msgs
  std_msgs
)

# Catkin
catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
    ${PROJECT_NAME}_filter
  CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    message_runtime
    moveit_msgs
    moveit_visual_tools
    rosparam_shortcuts
    std_msgs
    trajectory_msgs
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
  src/grasp_candidate.cpp
  src/grasp_data.cpp
  src/grasp_generator.cpp
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

# Demo filter executable
add_executable(${PROJECT_NAME}_grasp_filter_demo src/demo/grasp_filter_demo.cpp)
target_link_libraries(${PROJECT_NAME}_grasp_filter_demo
  ${PROJECT_NAME} ${PROJECT_NAME}_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

# Demo grasp executable
add_executable(${PROJECT_NAME}_grasp_generator_demo src/demo/grasp_generator_demo.cpp)
target_link_libraries(${PROJECT_NAME}_grasp_generator_demo
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Demo grasp data settings
add_executable(${PROJECT_NAME}_grasp_poses_visualizer_demo src/demo/grasp_poses_visualizer_demo.cpp)
target_link_libraries(${PROJECT_NAME}_grasp_poses_visualizer_demo
  ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

# Demo mesh bounding box
add_executable(${PROJECT_NAME}_mesh_bounding_box_demo src/demo/mesh_bounding_box_demo.cpp)
target_link_libraries(${PROJECT_NAME}_mesh_bounding_box_demo
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## INSTALL ##
#############

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
  ${PROJECT_NAME}_grasp_filter_demo
  ${PROJECT_NAME}_grasp_generator_demo
  ${PROJECT_NAME}_grasp_poses_visualizer_demo
  ${PROJECT_NAME}_mesh_bounding_box_demo
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############
## Testing ##
#############

find_package(rostest REQUIRED)

add_rostest_gtest(grasp_data_test test/grasp_data_test.test test/grasp_data_test.cpp)
target_link_libraries(grasp_data_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_rostest_gtest(grasp_generator_test test/grasp_generator_test.test test/grasp_generator_test.cpp)
target_link_libraries(grasp_generator_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

## Test for correct C++ source code
roslint_cpp()
