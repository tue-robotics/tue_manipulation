cmake_minimum_required(VERSION 2.8.3)
project(tue_manipulation)

find_package(catkin REQUIRED COMPONENTS
    kdl_parser
    urdf
    actionlib
    tf
    tf_conversions
    control_msgs

    # For testing
    roslib

    # Temp
    geolib2
)

# find_package(Boost REQUIRED COMPONENTS system program_options)
# find_package(PCL REQUIRED)
# find_package(OpenCV REQUIRED)

# ------------------------------------------------------------------------------------------------
#                                     ROS MESSAGES AND SERVICES
# ------------------------------------------------------------------------------------------------

add_action_files(
  FILES
    GraspPrecompute.action
    GripperCommand.action
)

# Generate messages
add_message_files(
   FILES
   GripperCommand.msg
   GripperMeasurement.msg
   cart_pos.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    actionlib_msgs
)

# Generate services
# add_service_files(
#    FILES
#    service1.srv
#    ...
# )

# generate_messages(
#    DEPENDENCIES
#    geometry_msgs
#    ...
# )

# ------------------------------------------------------------------------------------------------
#                                          CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bla
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------

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

# ------------------------------------------------------------------------------------------------
#                                              TESTS
# ------------------------------------------------------------------------------------------------

add_executable(test_robot_ik test/test_robot_ik)
target_link_libraries(test_robot_ik tue_manipulation)

add_executable(test_grasp_precompute test/test_grasp_precompute.cpp)
target_link_libraries(test_grasp_precompute tue_manipulation)

add_executable(test_amigo_dwa test/test_amigo_dwa.cpp)
target_link_libraries(test_amigo_dwa tue_manipulation)

add_executable(test_gripper_server test/test_gripper_server.cpp)
target_link_libraries(test_gripper_server tue_manipulation)
