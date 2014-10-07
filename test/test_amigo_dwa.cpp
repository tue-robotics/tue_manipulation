#include <tue/manipulation/dwa.h>

#include <iostream>
#include <fstream>

#include <ros/package.h>

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>

tue::manipulation::DWA dwa;
KDL::JntArray q_current(8);
std::vector<bool> q_set(8, false);
int num_joints_set = 0;

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "amigo_manipulation_dwa");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/amigo/torso/measurements", 100, jointStateCallback);
    ros::Subscriber sub2 = n.subscribe("/amigo/right_arm/measurements", 100, jointStateCallback);

    ros::Publisher pub = n.advertise<sensor_msgs::JointState>("/amigo/torso/references", 100);
    ros::Publisher pub2 = n.advertise<sensor_msgs::JointState>("/amigo/right_arm/references", 100);

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
    if (!dwa.initFromURDF(urdf_xml, "base_link", "grippoint_right", error))
    {
        std::cout << error << std::endl;
        return 1;
    }

    for(unsigned int i = 0; i < q_current.rows(); ++i)
        q_current(i) = 0;

    double dt = 0.05;

    ros::Rate r(1.0 / dt);
    while(ros::ok())
    {
        ros::spinOnce();

        if (num_joints_set < q_current.rows())
            continue;

        std::vector<double> q_wanted(q_current.rows());
        for(unsigned int i = 0; i < q_current.rows(); ++i)
        {
            double vel = dwa.calculateVelocity(q_current, i);
            std::cout << i << ": " << vel << std::endl;
            q_wanted[i] = q_current(i) + (vel * dt);
        }

        for(unsigned int i = 0; i< q_wanted.size(); ++i)
            std::cout << q_wanted[i] << " ";
        std::cout << std::endl;

//        for(unsigned int i = 0; i< q_wanted.size(); ++i)
//            q_wanted[i] = 0;

        // publish torso
        sensor_msgs::JointState msg1;
        msg1.name.push_back(dwa.getJointName(0));
        msg1.position.push_back(q_wanted[0]);
        msg1.header.stamp = ros::Time::now();
        pub.publish(msg1);

        // publish arm
        sensor_msgs::JointState msg2;
        for(unsigned int i = 1; i < q_wanted.size(); ++i)
        {
            msg2.name.push_back(dwa.getJointName(i));
            msg2.position.push_back(q_wanted[i]);
        }
        msg2.header.stamp = ros::Time::now();
        pub2.publish(msg2);

        r.sleep();
    }


    return 0;
}
