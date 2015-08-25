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

    /** Pre grasp offset */
    double pre_grasp_delta_;

    /** MoveIt group */
    moveit::planning_interface::MoveGroup* moveit_group_;

};

#endif
