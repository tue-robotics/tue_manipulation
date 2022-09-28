cmake_minimum_required(VERSION 3.0.2)
project(tue_manipulation)

add_compile_options(-Wall -Werror=all)
add_compile_options(-Wextra -Werror=extra)

find_package(catkin REQUIRED COMPONENTS
    actionlib
    actionlib_msgs
    control_msgs
    diagnostic_msgs
    geolib2
    kdl_parser
    moveit_msgs
    moveit_ros_planning_interface
    sensor_msgs
    std_msgs
    tf
    tf_conversions
    tue_manipulation_msgs
    tue_msgs
    urdf
    visualization_msgs
)
find_package(OpenCV REQUIRED)
find_package(orocos_kdl REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tue_manipulation
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    control_msgs
    diagnostic_msgs
    kdl_parser
    moveit_msgs
    sensor_msgs
    std_msgs
    tf
    tue_manipulation_msgs
    tue_msgs
    urdf
    visualization_msgs
  DEPENDS orocos_kdl
)

include_directories(
    include
    SYSTEM
    ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
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
target_link_libraries(tue_manipulation constrained_ik_solver ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Joint trajectory action
add_executable(joint_trajectory_action src/joint_trajectory_action.cpp)
target_link_libraries(joint_trajectory_action tue_manipulation)

# Grasp precompute
add_executable(grasp_precompute_action src/grasp_precompute_action.cpp
                                       src/grasp_precompute.cpp)
target_link_libraries(grasp_precompute_action tue_manipulation)

# Gripper server
add_executable(gripper_server src/gripper_server.cpp)
target_link_libraries(gripper_server tue_manipulation)

# Torso server
add_executable(torso_server src/torso_server.cpp)
target_link_libraries(torso_server ${catkin_LIBRARIES})
add_dependencies(torso_server ${catkin_EXPORTED_TARGETS})

# ------------------------------------------------------------------------------------------------
#                                              TESTS
# ------------------------------------------------------------------------------------------------

add_executable(test_refgen test/test_refgen.cpp)
target_link_libraries(test_refgen tue_manipulation)

add_executable(test_multi_refgen test/test_multi_refgen.cpp)
target_link_libraries(test_multi_refgen tue_manipulation)

add_executable(test_robot_ik test/test_robot_ik.cpp)
target_link_libraries(test_robot_ik tue_manipulation)

add_executable(test_grasp_precompute test/test_grasp_precompute.cpp)
target_link_libraries(test_grasp_precompute tue_manipulation)

add_executable(test_moveit test/test_moveit.cpp)
target_link_libraries(test_moveit tue_manipulation)

add_executable(test_amigo_dwa test/test_amigo_dwa.cpp)
target_link_libraries(test_amigo_dwa tue_manipulation)

add_executable(test_gripper_server test/test_gripper_server.cpp)
target_link_libraries(test_gripper_server tue_manipulation)

add_executable(torso_server_test_client test/test_torso_server.cpp)
target_link_libraries(torso_server_test_client ${catkin_LIBRARIES})
add_dependencies(torso_server_test_client ${catkin_EXPORTED_TARGETS})

add_executable(move_group_interface_tutorial src/move_group_interface_tutorial.cpp)
target_link_libraries(move_group_interface_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_interface_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
