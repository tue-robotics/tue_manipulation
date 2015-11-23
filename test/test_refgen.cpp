#include <iostream>

#include <tue/manipulation/reference_generator.h>
#include <tue/manipulation/graph_viewer.h>

int main(int argc, char **argv)
{
    if (argc == 1)
    {
        std::cout << "Please provide set points like this: joint1 joint2 ... jointN - joint1 joint2 ... jointN - ... etc ..." << std::endl;
        return 1;
    }

    tue::manipulation::ReferenceGenerator refgen;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Create goal and initialize joints

    bool first_point = true;
    control_msgs::FollowJointTrajectoryGoal goal;
    std::vector<trajectory_msgs::JointTrajectoryPoint>::iterator itp = goal.trajectory.points.end();
    for(unsigned int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];

        if (arg == "--")
        {
            itp = goal.trajectory.points.end();
            first_point = false;
        }
        else
        {
            double pos = atof(arg.c_str());

            if (first_point)
            {
                std::stringstream ss;
                ss << "joint-" << (i-1);
                refgen.initJoint(ss.str(), 1, 1, -1, 1);
                refgen.setJointState(ss.str(), pos, 0);
            }
            else
            {
                if (itp == goal.trajectory.points.end())
                {
                    goal.trajectory.points.push_back(trajectory_msgs::JointTrajectoryPoint());
                    itp = goal.trajectory.points.end() - 1;
                }

                itp->positions.push_back(pos);
            }
        }
    }

    goal.trajectory.joint_names = refgen.joint_names();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    unsigned int num_joints = refgen.joint_names().size();

    std::vector<double> references(num_joints, 0);

    std::stringstream error_msg;
    if (!refgen.setGoal(goal, error_msg))
    {
        std::cout << "Error: " << error_msg.str() << std::endl;
        return 1;
    }

    double dt = 0.01;
    double time = 0;

    GraphViewer g;

    while(!refgen.is_idle())
    {
        refgen.calculatePositionReferences(0.01, references);
        time += dt;

//        std::cout << time << ": " << references[0] << std::endl;

        g.addPoint(0, 0, time, refgen.position(0));
        g.addPoint(0, 1, time, refgen.position(1));
    }

    g.view(true);

    return 0;
}
