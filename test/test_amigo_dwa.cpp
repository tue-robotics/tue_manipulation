#include <tue/manipulation/dwa.h>

#include <iostream>
#include <fstream>

#include <ros/package.h>

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>

tue::manipulation::DWA dwa;
KDL::JntArray q_current(8);
std::vector<bool> q_set(8, false);
uint num_joints_set = 0;

ros::Publisher pub_torso, pub_arm;

// ----------------------------------------------------------------------------------------------------

class GraspContraint : public tue::manipulation::Constraint
{

public:

    GraspContraint(const geo::Pose3D& goal) : goal_(goal) {}

    virtual double test(const geo::Pose3D& pose) const
    {
//        double dx = goal_.t.x - pose.t.x;
//        double dy = goal_.t.y - pose.t.y;

//        double plane_dist = sqrt(dx * dx + dy * dy);

//        double wanted_dist = 0;
//        double plane_err = std::abs(plane_dist - wanted_dist);

//        double z_err = std::abs(goal_.t.z - pose.t.z);

//        double rot_err1 = std::abs((pose.R * geo::Vector3(0.1, 0, 0)).z);
        double rot_err2 = std::abs((pose.R * geo::Vector3(0, 0.1, 0)).z);

//        return plane_err * plane_err + z_err + rot_err1;

        double dist_sq = (goal_.t - pose.t).length2();
        return dist_sq + rot_err2;



//        double rot1 = 0;
//        double rot2 = 0;

//        double dist = dist_sq + rot1 + rot2;

//        return dist;
    }

private:

    geo::Pose3D goal_;

};


// ----------------------------------------------------------------------------------------------------

void jointStateCallback(const sensor_msgs::JointState& joint_msg)
{
    for(unsigned int i = 0; i < joint_msg.name.size(); ++i)
    {
        unsigned int i_joint;
        if (dwa.getJointIndex(joint_msg.name[i], i_joint))
        {
            q_current(i_joint) = joint_msg.position[i];
            if (!q_set[i_joint])
            {
                ++num_joints_set;
                q_set[i_joint] = true;
            }
        }
        else
        {
            std::cout << "Unknown joint:" << joint_msg.name[i] << std::endl;
        }
    }
}

// ----------------------------------------------------------------------------------------------------

void sendReference(const std::vector<double>& q_wanted)
{
    ros::Time t_now = ros::Time::now();

    {
        // Torso

        trajectory_msgs::JointTrajectoryPoint p;
        p.positions.push_back(q_wanted[0]);
        p.time_from_start = ros::Duration(0);

        control_msgs::FollowJointTrajectoryActionGoal g;
        g.goal.trajectory.header.stamp = t_now;
        g.header.stamp = t_now;
        g.goal.trajectory.points.push_back(p);
        g.goal.trajectory.joint_names.push_back(dwa.getJointName(0));

        pub_torso.publish(g);
    }

    {
        // Arm

        trajectory_msgs::JointTrajectoryPoint p;

        for(unsigned int i = 1; i < q_wanted.size(); ++i)
            p.positions.push_back(q_wanted[i]);
        p.time_from_start = ros::Duration(0);

        control_msgs::FollowJointTrajectoryActionGoal g;
        g.goal.trajectory.header.stamp = t_now;
        g.header.stamp = t_now;

        g.goal.trajectory.points.push_back(p);

        for(unsigned int i = 1; i < q_wanted.size(); ++i)
            g.goal.trajectory.joint_names.push_back(dwa.getJointName(i));

        pub_arm.publish(g);
    }

}


// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "amigo_manipulation_dwa");

    ros::NodeHandle n;
    ros::Subscriber sub_torso = n.subscribe("/amigo/torso/measurements", 100, jointStateCallback);
    ros::Subscriber sub_arm = n.subscribe("/amigo/right_arm/measurements", 100, jointStateCallback);

    pub_torso = n.advertise<control_msgs::FollowJointTrajectoryActionGoal>
            ("/amigo/body/joint_trajectory_action/goal", 100);

    pub_arm = n.advertise<control_msgs::FollowJointTrajectoryActionGoal>
            ("/amigo/right_arm/joint_trajectory_action/goal", 100);

    // - - - - - - - - - - - Read the AMIGO URDF description into a string - - - - - - - - - - -

    std::string amigo_urdf_path = ros::package::getPath("amigo_description") + "/urdf/amigo.urdf";
    std::ifstream f(amigo_urdf_path.c_str());

    if (!f.is_open())
    {
        std::cout << "Could not load AMIGO's URDF description: '" << amigo_urdf_path << "'." << std::endl;
        return 1;
    }

    std::stringstream buffer;
    buffer << f.rdbuf();
    std::string urdf_xml = buffer.str();

    // - - - - - - - - - - - Initialize the solver - - - - - - - - - - -

    std::string error;
    if (!dwa.initFromURDF(urdf_xml, "amigo/base_link", "amigo/grippoint_right", error))
    {
        std::cout << error << std::endl;
        return 1;
    }

    dwa.setConstraint(new GraspContraint(geo::Pose3D(0.7, -0.2, 0.8)));

    for(unsigned int i = 0; i < q_current.rows(); ++i)
        q_current(i) = 0;

    double dt = 0.05;

    // - - - - - - - - - - - - - - - - - - - - - -

    ros::Rate r(1.0 / dt);
    while(ros::ok())
    {
        ros::spinOnce();

        if (num_joints_set < q_current.rows())
            continue;

        std::vector<double> q_wanted(q_current.rows());
        dwa.calculateVelocity(q_current, dt, q_wanted);

        sendReference(q_wanted);

//        for(unsigned int i = 0; i< q_wanted.size(); ++i)
//            std::cout << q_wanted[i] << " ";
//        std::cout << std::endl;

        r.sleep();
    }


    return 0;
}
