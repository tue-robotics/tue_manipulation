cmake_minimum_required(VERSION 2.8.3)
project(tue_manipulation)

find_package(catkin REQUIRED COMPONENTS
    kdl_parser
    urdf
    actionlib
    amigo_msgs
    tf
    tf_conversions

    # For testing
    roslib
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
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    actionlib_msgs
    amigo_msgs
)


# Generate messages
# add_message_files(
#    FILES
#    message1.msg
#    ...
# )

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

add_library(tue_manipulation
    src/ik_solver.cpp    include/tue/manipulation/ik_solver.h
)
target_link_libraries(tue_manipulation ${catkin_LIBRARIES})

# Joint trajectory action
add_executable(joint_trajectory_action src/joint_trajectory_action.cpp)
target_link_libraries(joint_trajectory_action tue_manipulation)

add_executable(grasp_precompute_action src/grasp_precompute_action.cpp)
target_link_libraries(grasp_precompute_action tue_manipulation)

# ------------------------------------------------------------------------------------------------
#                                              TESTS
# ------------------------------------------------------------------------------------------------

add_executable(test_amigo_ik
    test/test_amigo_ik
)
target_link_libraries(test_amigo_ik tue_manipulation)




