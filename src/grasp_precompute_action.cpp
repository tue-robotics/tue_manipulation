// Author: Rob Janssen & the two stooges
// Modifications: Janno Lunenburg

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <tue_manipulation/GraspPrecomputeAction.h>

#include <control_msgs/FollowJointTrajectoryAction.h>

#include <sensor_msgs/JointState.h>

#include <tue/manipulation/ik_solver.h>

#include <tf_conversions/tf_kdl.h>

#include <visualization_msgs/MarkerArray.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

const double EPS = 1e-6;

using namespace std;

double MAX_YAW_DELTA,YAW_SAMPLING_STEP,PRE_GRASP_DELTA;
//string SIDE;
std::string ROOT_LINK, TIP_LINK;

int NUM_GRASP_POINTS = 0, PRE_GRASP_INBETWEEN_SAMPLING_STEPS = 0;

tue::IKSolver ik_solver;

typedef actionlib::SimpleActionServer<tue_manipulation::GraspPrecomputeAction> Server;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;

ros::Publisher *IKpospub;

tf::TransformListener* TF_LISTENER;

sensor_msgs::JointState arm_joints;
double spindle_position;

void armcontrollerCB(const sensor_msgs::JointState& joint_meas)
{
    arm_joints = joint_meas;
}

void spindlecontrollerCB(const sensor_msgs::JointState& spindle_meas)
{
    spindle_position = spindle_meas.position[0];
}

void execute(const tue_manipulation::GraspPrecomputeGoalConstPtr& goal_in, Server* as, Client* ac)
{
    // Check for absolute or delta (and ambiqious goals)
    bool absolute_requested=false, delta_requested=false;
    if(fabs(goal_in->goal.x)>EPS || fabs(goal_in->goal.y)>EPS || fabs(goal_in->goal.z)>EPS || fabs(goal_in->goal.roll)>EPS || fabs(goal_in->goal.pitch)>EPS || fabs(goal_in->goal.yaw)>EPS)
        absolute_requested = true;
    if(fabs(goal_in->delta.x)>EPS || fabs(goal_in->delta.y)>EPS || fabs(goal_in->delta.z)>EPS || fabs(goal_in->delta.roll)>EPS || fabs(goal_in->delta.pitch)>EPS || fabs(goal_in->delta.yaw)>EPS)
        delta_requested = true;
    if(absolute_requested && delta_requested)
    {
        as->setAborted();
        ROS_WARN("grasp_precompute_action: goal consists out of both absolute AND delta values. Choose only one!");
        return;
    }
    ///ROS_INFO("FIRST_JOINT_POS_ONLY = %d",goal_in->FIRST_JOINT_POS_ONLY);
    // Create input variable which we will work with
    geometry_msgs::PoseStamped stamped_in;
    stamped_in.header = goal_in->goal.header;
    stamped_in.pose.position.x = goal_in->goal.x;
    stamped_in.pose.position.y = goal_in->goal.y;
    stamped_in.pose.position.z = goal_in->goal.z;
    stamped_in.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(goal_in->goal.roll, goal_in->goal.pitch, goal_in->goal.yaw);

    // Check if a delta is requested
    if(delta_requested)
    {
        ROS_DEBUG("Delta requested of x=%f \t y=%f \t z=%f \t roll=%f \t pitch=%f \t yaw=%f",goal_in->delta.x,goal_in->delta.y,goal_in->delta.z,goal_in->delta.roll,goal_in->delta.pitch,goal_in->delta.yaw);
        // Create tmp variable
        tf::StampedTransform tmp;

        // Lookup the current vector of the desired TF to the grippoint
        if(TF_LISTENER->waitForTransform(goal_in->delta.header.frame_id, TIP_LINK, goal_in->delta.header.stamp, ros::Duration(1.0)))
        {
            try
            {TF_LISTENER->lookupTransform(goal_in->delta.header.frame_id, TIP_LINK, goal_in->delta.header.stamp, tmp);
            }
            catch (tf::TransformException ex){
                as->setAborted();
                ROS_ERROR("%s",ex.what());
                return;}
        }else{
            as->setAborted();
            ROS_ERROR("grasp_precompute_action: TF_LISTENER could not find transform from gripper to %s:",goal_in->delta.header.frame_id.c_str());
            return;

        }

        // Print the transform
        ROS_DEBUG("tmp.x=%f \t tmp.y=%f \t tmp.z=%f tmp.X=%f \t tmp.Y=%f \t tmp.Z=%f \t tmp.W=%f",tmp.getOrigin().getX(),tmp.getOrigin().getY(),tmp.getOrigin().getZ(),tmp.getRotation().getX(),tmp.getRotation().getY(),tmp.getRotation().getZ(),tmp.getRotation().getW());

        // Add the delta values and overwrite input variable
        ROS_INFO("Child frame id=%s", tmp.frame_id_.c_str());
        stamped_in.header.frame_id  = tmp.frame_id_;
        stamped_in.header.stamp     = tmp.stamp_;
        stamped_in.pose.position.x  = tmp.getOrigin().getX() + goal_in->delta.x;
        stamped_in.pose.position.y  = tmp.getOrigin().getY() + goal_in->delta.y;
        stamped_in.pose.position.z  = tmp.getOrigin().getZ() + goal_in->delta.z;
        double roll, pitch, yaw;
        tmp.getBasis().getRPY(roll, pitch, yaw);
        stamped_in.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll + goal_in->delta.roll, pitch + goal_in->delta.pitch, yaw + goal_in->delta.yaw);
    }


    // If input frame is not /base_link, convert to base_link so that sampling over the yaw indicates the z-axes pointing up (whereas it wouldn't with e.g. '/torso')
    if(stamped_in.header.frame_id.compare("/amigo/base_link"))
    {
        ROS_DEBUG("Frame id was not BASE_LINK");
        // Create tmp variable
        geometry_msgs::PoseStamped tmp;

        // Perform the transformation to /base_link
        if(TF_LISTENER->waitForTransform("/amigo/base_link", stamped_in.header.frame_id, stamped_in.header.stamp, ros::Duration(1.0)))
        {
            try
            {TF_LISTENER->transformPose("/amigo/base_link", stamped_in, tmp);}
            catch (tf::TransformException ex){
                as->setAborted();
                ROS_ERROR("%s",ex.what());
                return;}
        }else{
            as->setAborted();
            ROS_ERROR("grasp_precompute_action: TF_LISTENER could not find transform from /amigo/base_link to %s:",stamped_in.header.frame_id.c_str());
            return;

        }

        // Overwrite input values
        stamped_in = tmp;

        // EXPERIMENTAL: WHEN FRAME_ID IS NOT BASE LINK, DEFINE ANGLES IN BASE LINK FRAME TO BE ZERO TO ALLOW PROPER SAMPLING
        stamped_in.pose.orientation.x = 0;
        stamped_in.pose.orientation.y = 0;
        stamped_in.pose.orientation.z = 0;
        stamped_in.pose.orientation.w = 1;
    }

    ROS_DEBUG("Frame=%s \t X=%f \t Y=%f \t Z=%f",stamped_in.header.frame_id.c_str(),stamped_in.pose.position.x,stamped_in.pose.position.y,stamped_in.pose.position.z);

    // Define general joint trajectory action info
    control_msgs::FollowJointTrajectoryGoal jtagoal;

    // Define general visualization markers for feasible IK positions
    visualization_msgs::MarkerArray IKPosMarkerArray;

    // Define stringstream for IK pos ID
    ostringstream ss;

    // Store the solutions
    KDL::JntArray grasp_solution;
    vector<KDL::JntArray> pre_grasp_solution;

    // Define joint names and seed positions
    KDL::JntArray joint_seeds(ik_solver.numJoints());
    for(unsigned int i = 0; i < ik_solver.numJoints(); ++i)
    {
        const std::string& joint_name = ik_solver.jointNames()[i];

        if (i == 0) {
            joint_seeds(i) = spindle_position;
        } else {
            for(unsigned int j = 0; j < arm_joints.name.size(); ++j) {
                if (joint_name == arm_joints.name[j]) {
                    joint_seeds(i) = arm_joints.position[j];
                }
            }
        }

        jtagoal.trajectory.joint_names.push_back(joint_name);
    }

    // Check if a pre-grasp is required, resize Joint Trajectory Action
    if (!goal_in->PERFORM_PRE_GRASP)
    {
        NUM_GRASP_POINTS = 1;
    }
    else
    {
        NUM_GRASP_POINTS = PRE_GRASP_INBETWEEN_SAMPLING_STEPS + 2; // Inbetween sample points + Pre-grasp + Grasp point
    }

    // Resize the Joint Trajectory Action goal according to the number of grasp points or to one if FIRST_JOINT_POS_ONLY is TRUE
    int num_jta_points;
    if (goal_in->FIRST_JOINT_POS_ONLY)
        num_jta_points = 1;
    else
        num_jta_points = NUM_GRASP_POINTS;

    jtagoal.trajectory.points.resize(num_jta_points);
    IKPosMarkerArray.markers.resize(NUM_GRASP_POINTS);
    pre_grasp_solution.resize(NUM_GRASP_POINTS);
    for(int i=0;i<(num_jta_points);++i)
    {
        if (i < num_jta_points)
        {
            jtagoal.trajectory.points[i].positions.resize(ik_solver.numJoints());
            jtagoal.trajectory.points[i].velocities.resize(ik_solver.numJoints());
            jtagoal.trajectory.points[i].accelerations.resize(ik_solver.numJoints());
        }

        IKPosMarkerArray.markers[i].type = 2; // 2=SPHERE
        IKPosMarkerArray.markers[i].scale.x = 0.01;
        IKPosMarkerArray.markers[i].scale.y = 0.01;
        IKPosMarkerArray.markers[i].scale.z = 0.01;
        IKPosMarkerArray.markers[i].color.r = 1.0f;
        IKPosMarkerArray.markers[i].color.g = 0.0f;
        IKPosMarkerArray.markers[i].color.b = 0.0f;
        IKPosMarkerArray.markers[i].color.a = 1.0;
        IKPosMarkerArray.markers[i].header.frame_id = stamped_in.header.frame_id;
    }

    ///////////////////////////////////////////////////////////
    /// EVALUATE IF GRASP IS FEASIBLE AND CREATE JOINT ARRAY///
    ///////////////////////////////////////////////////////////
    tf::Transform grasp_pose;
    tf::poseMsgToTF(stamped_in.pose,grasp_pose);

    tf::Transform new_grasp_pose;
    tf::Transform new_pre_grasp_pose;
    bool GRASP_FEASIBLE = false, SAMPLING_BOUNDARIES_REACHED=false;
    double YAW_DELTA = 0.0;
    int YAW_SAMPLING_DIRECTION = 1;

    ROS_INFO("Starting sampling...");

    while(ros::ok() && !GRASP_FEASIBLE && !SAMPLING_BOUNDARIES_REACHED )
    {
        // Define new_grasp_pose
        tf::Transform yaw_offset(tf::createQuaternionFromYaw(YAW_SAMPLING_DIRECTION * YAW_DELTA),tf::Point(0,0,0));
        new_grasp_pose = grasp_pose * yaw_offset;

        // Check if all grasp points are feasible
        GRASP_FEASIBLE = true;
        int k = 0;
        for(int i = NUM_GRASP_POINTS - 1; i >= 0; --i)
        {
            //cout << i << " offset = " << -PRE_GRASP_DELTA*i/(PRE_GRASP_INBETWEEN_SAMPLING_STEPS+1.0) << endl;
            tf::Transform pre_grasp_offset(tf::Quaternion(0,0,0,1),tf::Point(-PRE_GRASP_DELTA*i/(PRE_GRASP_INBETWEEN_SAMPLING_STEPS+1.0),0,0));
            new_pre_grasp_pose = new_grasp_pose * pre_grasp_offset;

            KDL::Frame new_pre_grasp_pose_kdl;
            tf::poseTFToKDL(new_pre_grasp_pose, new_pre_grasp_pose_kdl);

            KDL::JntArray joint_solution;
            if (ik_solver.cartesianToJoints(new_pre_grasp_pose_kdl, joint_solution, joint_seeds))
            {
                pre_grasp_solution[k] = joint_solution;
                ++k;

                // Set the seed state for the next point to the solution of the previous point
                joint_seeds = joint_solution;

                // Fill the IK markers
                IKPosMarkerArray.markers[i].id = i;
                tf::poseTFToMsg(grasp_pose * yaw_offset * pre_grasp_offset, IKPosMarkerArray.markers[i].pose);
            }
            else
            {
                GRASP_FEASIBLE = false;
                break;
            }
        }

        //ROS_INFO("Yaw delta %f ",YAW_SAMPLING_DIRECTION * YAW_DELTA);
        // If the grasp is not feasible, change the yaw and the sampling direction
        if (!GRASP_FEASIBLE)
        {
            ROS_DEBUG("Not all grasp points feasible: resampling yaw");
            YAW_DELTA = YAW_DELTA + YAW_SAMPLING_STEP;
            YAW_SAMPLING_DIRECTION = -1 * YAW_SAMPLING_DIRECTION;

            if(YAW_DELTA > MAX_YAW_DELTA)
            {
                ROS_WARN("Sampling boundaries reached. No feasible sample found\n");
                SAMPLING_BOUNDARIES_REACHED = true;
                break;

            }
        }
    }

    //////////////////////////////////////////////////////////
    //////////////// EXECUTING JOINT ARRAY ///////////////////
    //////////////////////////////////////////////////////////
    // SetAborted when grasp not feasible, else start actuating the robot when all grasp points are feasible
    bool GRASP_SUCCESS = true;
    if(!GRASP_FEASIBLE)
    {
        ROS_INFO("Grasp not feasible\n");
        GRASP_SUCCESS = false;
    }
    else
    {
        ROS_INFO("Solution found: moving arms");
        // Publish the IK markers
        IKpospub->publish(IKPosMarkerArray);

        // Continue if GRASP_SUCCESS is still true
        if(GRASP_SUCCESS)
        {
            // Fill in all the solutions to the JTA goal
            if (goal_in->FIRST_JOINT_POS_ONLY)
            {
                for(unsigned int i = 0; i < ik_solver.numJoints(); ++i)
                {
                    jtagoal.trajectory.points[0].positions[i] = pre_grasp_solution[0](i);
                    //jtagoal.trajectory.points[0].positions[i]   = pre_grasp_solution[0].joint_state.position[i];
                    //jtagoal.trajectory.points[0].velocities[i] = pre_grasp_solution.joint_state.velocity[i];
                    //jtagoal.trajectory.points[0].accelerations[i] = 0.0;
                }
            }
            else if (!goal_in->FIRST_JOINT_POS_ONLY)
            {
                for(int j = 0; j < NUM_GRASP_POINTS; ++j)
                {
                    for(unsigned int i = 0; i < ik_solver.numJoints(); ++i)
                    {
                        jtagoal.trajectory.points[j].positions[i] = pre_grasp_solution[j](i);
                        //jtagoal.trajectory.points[j].positions[i]   = pre_grasp_solution[j].joint_state.position[i];
                        //jtagoal.trajectory.points[0].velocities[i] = pre_grasp_solution.joint_state.velocity[i];
                        //jtagoal.trajectory.points[0].accelerations[i] = 0.0;
                    }
                }
            }
            ac->sendGoal(jtagoal);
            ac->waitForResult();
            if (ac->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Execution of all grasp points succeeded\n");
                GRASP_SUCCESS = false;
            }
        }
    }

    //////////////////////////////////////////////////////////
    ///////////////// FEEDBACK TO CLIENT /////////////////////
    //////////////////////////////////////////////////////////
    if(GRASP_SUCCESS)
    {
        as->setSucceeded();
    }else
    {
        as->setAborted();
    }
}

int main(int argc, char** argv)
{

    ROS_INFO("Starting grasp precompute node");

    ros::init(argc, argv, "grasp_precompute_server");
    ros::NodeHandle n("~");

    TF_LISTENER = new tf::TransformListener();

    // Get the parameters
    std::string side;
    n.param<string>("side", side, ""); //determine for which side this node operates
    if (side.empty()){
        ROS_ERROR("Missing parameter 'side'.");
        return 1;
    }

    n.param("max_yaw_delta", MAX_YAW_DELTA, 2.0); // maximum sampling offset from desired yaw [rad]
    n.param("yaw_sampling_step", YAW_SAMPLING_STEP, 0.2); // step-size for yaw sampling [rad]
    n.param("pre_grasp_delta", PRE_GRASP_DELTA, 0.05); // offset for pre-grasping in cartesian x-direction [m]
    n.param("pre_grasp_inbetween_sampling_steps", PRE_GRASP_INBETWEEN_SAMPLING_STEPS, 0); // offset for pre-grasping in cartesian x-direction [m]

    int ik_max_iterations;
    n.param("ik_max_iterations", ik_max_iterations, 500);

    n.param<std::string>("root_link", ROOT_LINK, "");
    if (ROOT_LINK.empty()){
        ROS_ERROR("Missing parameter 'root_link'.");
        return 1;
    }

    n.param<std::string>("tip_link", TIP_LINK, "");
    if (TIP_LINK.empty()){
        ROS_ERROR("Missing parameter 'tip_link'.");
        return 1;
    }

    ROS_INFO("Waiting for joint trajectory action");

    // Wait for the joint trajectory action server
    Client client("joint_trajectory_action_" + side, true);
    client.waitForServer();

    ROS_INFO("Initialize grasp precompute server");

    // Initialize the grasp_precompute server
    Server server(n, "/grasp_precompute_" + side, boost::bind(&execute, _1, &server, &client), false);
    server.start();

    ROS_INFO("Initialize IK clients");

    // Initialize the IK solver
    std::string urdf_description;
    n.param<std::string>("robot_description", urdf_description, "");
    bool use_constrained_solver;
    n.param("use_constrained_solver", use_constrained_solver, false); // Indicates whether to use the constrained IK solver developed for SERGIO

    std::string error;
    if (!ik_solver.initFromURDF(urdf_description, ROOT_LINK, TIP_LINK, ik_max_iterations, error, use_constrained_solver))
    {
        ROS_ERROR_STREAM("Could not initialize IK solver:\n\n    " << error);
        return 1;
    }

    // Start listening to the current joint measurements
    ros::Subscriber armsub = n.subscribe("/joint_measurements", 1, armcontrollerCB);

    // Start listening to the current spindle measurement
    ros::Subscriber spindlesub = n.subscribe("/spindle_measurement", 1, spindlecontrollerCB);

    // IK marker publisher
    IKpospub = new ros::Publisher(n.advertise<visualization_msgs::MarkerArray>("/IK_Position_Markers", 1));

    ROS_INFO("Grasp precompute action initialized");

    ros::spin();

    delete TF_LISTENER;

    return 0;
}
