#ifndef GRASP_PRECOMPUTE_
#define GRASP_PRECOMPUTE_

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <tue_manipulation/GraspPrecomputeAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <tf/transform_listener.h>

#include <moveit/move_group_interface/move_group.h>

class GraspPrecompute
{

public:

    /** Constructor */
    GraspPrecompute();

    /** Destructor */
    virtual ~GraspPrecompute();

private:

    /** Action server */
    actionlib::SimpleActionServer<tue_manipulation::GraspPrecomputeAction>* as_;

    /** Goal callback function */
    void execute(const tue_manipulation::GraspPrecomputeGoalConstPtr& goal);

    /** TF listener */
    tf::TransformListener* listener_;

    /** Root and tip link */
    std::string root_link_, tip_link_;

    /** Offset for pre-grasping in cartesian x-direction [m] */
    double pre_grasp_delta_;

    /** Step-size for yaw sampling [rad] */
    double yaw_sampling_step_;

    /** Maximum offset from desired yaw [rad] */
    double max_yaw_;
    /*
    nh_private.param("yaw_sampling_step", YAW_SAMPLING_STEP, 0.2); //
    nh_private.param("pre_grasp_delta", PRE_GRASP_DELTA, 0.05); // Offset for pre-grasping in cartesian x-direction [m]
    nh_private.param("pre_grasp_inbetween_sampling_steps", PRE_GRASP_INBETWEEN_SAMPLING_STEPS, 0); // offset for pre-grasping in cartesian x-direction [m]
    */

    /** MoveIt group */
    moveit::planning_interface::MoveGroup* moveit_group_;

};

#endif
