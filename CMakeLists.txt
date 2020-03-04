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

# Catkin
catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
    ${PROJECT_NAME}_filter
  CATKIN_DEPENDS
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
  src/suction_grasp_candidate.cpp
  src/suction_grasp_data.cpp
  src/suction_grasp_generator.cpp
  src/suction_grasp_scorer.cpp
  src/two_finger_grasp_data.cpp
  src/two_finger_grasp_generator.cpp
  src/two_finger_grasp_scorer.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

# Grasp Filter Library
add_library(${PROJECT_NAME}_filter
  src/grasp_filter.cpp
  src/suction_grasp_filter.cpp
  src/two_finger_grasp_filter.cpp
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

# Demo grasp pipeline
add_executable(${PROJECT_NAME}_grasp_pipeline_demo src/demo/grasp_pipeline_demo.cpp)
target_link_libraries(${PROJECT_NAME}_grasp_pipeline_demo
  ${PROJECT_NAME} ${PROJECT_NAME}_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

# Demo suction grasp pipeline
add_executable(${PROJECT_NAME}_suction_grasp_pipeline_demo src/demo/suction_grasp_pipeline_demo.cpp)
target_link_libraries(${PROJECT_NAME}_suction_grasp_pipeline_demo
  ${PROJECT_NAME} ${PROJECT_NAME}_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES}
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
install(DIRECTORY config       DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config_robot DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch       DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY resources    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install executables
install(TARGETS
  ${PROJECT_NAME}_grasp_filter_demo
  ${PROJECT_NAME}_grasp_generator_demo
  ${PROJECT_NAME}_grasp_poses_visualizer_demo
  ${PROJECT_NAME}_grasp_pipeline_demo
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(two_finger_grasp_data_test test/two_finger_grasp_data_test.test test/two_finger_grasp_data_test.cpp)
  target_link_libraries(two_finger_grasp_data_test
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
  )

  add_rostest_gtest(two_finger_grasp_generator_test test/two_finger_grasp_generator_test.test test/two_finger_grasp_generator_test.cpp)
  target_link_libraries(two_finger_grasp_generator_test
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
  )

  add_rostest_gtest(two_finger_grasp_filter_test test/two_finger_grasp_filter_test.test test/two_finger_grasp_filter_test.cpp)
  target_link_libraries(two_finger_grasp_filter_test
    ${PROJECT_NAME}
    ${PROJECT_NAME}_filter
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
  )

  add_rostest_gtest(suction_grasp_pipeline_test test/suction_grasp_pipeline_test.test test/suction_grasp_pipeline_test.cpp)
  target_link_libraries(suction_grasp_pipeline_test
    ${PROJECT_NAME}
    ${PROJECT_NAME}_filter
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
  )

  add_rostest_gtest(suction_grasp_unit_tests test/suction_grasp_unit_tests.test test/suction_grasp_unit_tests.cpp)
  target_link_libraries(suction_grasp_unit_tests
    ${PROJECT_NAME}
    ${PROJECT_NAME}_filter
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
  )

  ## Test for correct C++ source code
  roslint_cpp()
endif()
