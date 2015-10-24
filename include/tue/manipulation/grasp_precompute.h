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

    /** Cartesian goal Action server */
    actionlib::SimpleActionServer<tue_manipulation::GraspPrecomputeAction>* as_;

    /** Cartesian goal callback function */
    void execute(const tue_manipulation::GraspPrecomputeGoalConstPtr& goal);

    /** Joint goal action server */
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>* jas_;

    /** Joint goal callback function */
    void joint_execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);

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

    /** MoveIt group */
    moveit::planning_interface::MoveGroup* moveit_group_;

};

#endif
