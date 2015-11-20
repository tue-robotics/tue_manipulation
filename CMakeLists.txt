cmake_minimum_required(VERSION 2.8.3)
project(tue_manipulation)

find_package(catkin REQUIRED COMPONENTS
    kdl_parser
    urdf
    actionlib
    actionlib_msgs
    tf
    tf_conversions
    control_msgs
    tue_msgs
    diagnostic_msgs
    visualization_msgs
    roslib # For testing
    geolib2 # Temp
    message_generation
    sensor_msgs
    std_msgs
)

add_message_files(
  FILES
    cart_pos.msg
)

add_action_files(
  FILES
    GraspPrecompute.action
    GripperCommand.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    tue_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tue_manipulation
  CATKIN_DEPENDS  
    actionlib_msgs
    control_msgs
    diagnostic_msgs
    message_runtime
    sensor_msgs
    std_msgs
    tue_msgs
    visualization_msgs
#  DEPENDS
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# Constrained IK solver
add_library(constrained_ik_solver
    src/constrained_chainiksolvervel_pinv.cpp   include/tue/manipulation/constrained_chainiksolvervel_pinv.h
    src/constrained_chainiksolverpos_nr_jl.cpp  include/tue/manipulation/constrained_chainiksolverpos_nr_jl.hpp
)
target_link_libraries(constrained_ik_solver ${catkin_LIBRARIES})

# IK solver
add_library(tue_manipulation
    src/ik_solver.cpp    include/tue/manipulation/ik_solver.h
    src/dwa.cpp          include/tue/manipulation/dwa.h
    src/reference_generator.cpp      include/tue/manipulation/reference_generator.h
    src/reference_interpolator.cpp   include/tue/manipulation/reference_interpolator.h
    src/graph_viewer.cpp include/tue/manipulation/graph_viewer.h
)
target_link_libraries(tue_manipulation constrained_ik_solver ${catkin_LIBRARIES})

# Joint trajectory action
add_executable(joint_trajectory_action src/joint_trajectory_action.cpp)
target_link_libraries(joint_trajectory_action tue_manipulation)

# Grasp precompute
add_executable(grasp_precompute_action src/grasp_precompute_action.cpp)
target_link_libraries(grasp_precompute_action tue_manipulation)
add_dependencies(grasp_precompute_action ${PROJECT_NAME}_generate_messages_cpp)

# Gripper server
add_executable(gripper_server src/gripper_server.cpp)
target_link_libraries(gripper_server tue_manipulation)
add_dependencies(gripper_server ${PROJECT_NAME}_generate_messages_cpp)

# Torso server
add_executable(torso_server src/torso_server.cpp)
target_link_libraries(torso_server
  ${catkin_LIBRARIES}
)
add_dependencies(torso_server ${catkin_EXPORTED_TARGETS})

# ------------------------------------------------------------------------------------------------
#                                              TESTS
# ------------------------------------------------------------------------------------------------

add_executable(test_robot_ik test/test_robot_ik.cpp)
target_link_libraries(test_robot_ik tue_manipulation)

add_executable(test_grasp_precompute test/test_grasp_precompute.cpp)
target_link_libraries(test_grasp_precompute tue_manipulation)

add_executable(test_amigo_dwa test/test_amigo_dwa.cpp)
target_link_libraries(test_amigo_dwa tue_manipulation)

add_executable(test_gripper_server test/test_gripper_server.cpp)
target_link_libraries(test_gripper_server tue_manipulation)

add_executable(torso_server_test_client test/test_torso_server.cpp)
target_link_libraries(torso_server_test_client
  ${catkin_LIBRARIES}
)
add_dependencies(torso_server_test_client ${catkin_EXPORTED_TARGETS})

add_executable(test_refgen test/test_refgen.cpp)
target_link_libraries(test_refgen tue_manipulation)
