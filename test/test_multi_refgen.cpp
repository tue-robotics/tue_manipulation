#include <fstream>
#include <sstream>

#include <tue/manipulation/reference_generator.h>
#include <tue/manipulation/graph_viewer.h>

int main(int argc, char **argv)
{
    if (argc == 1)
    {
        std::cout << "Please provide trajectory file" << std::endl;
        return 1;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Read trajectory from file

    std::ifstream file(argv[1]);
    if (!file.is_open())
    {
        std::cout << "Could not open " << argv[1] << std::endl;
        return 1;
    }

    tue::manipulation::ReferenceGenerator refgen;

    uint num_joints;
    file >> num_joints;

    control_msgs::FollowJointTrajectoryGoal goal;

    double v;
    for(unsigned int i = 0; i < num_joints; ++i)
    {
        std::stringstream ss;
        ss << "joint-" << (i-1);

        file >> v;
        refgen.initJoint(ss.str(), v, 0, -10000, 10000);
        refgen.setJointState(ss.str(), 0, 0);
        goal.trajectory.joint_names.push_back(ss.str());
    }

    for(unsigned int i = 0; i < num_joints; ++i)
    {
        file >> v;
        refgen.setMaxAcceleration(i, v);
    }

    while(file >> v)
    {
        trajectory_msgs::JointTrajectoryPoint p;
        p.time_from_start = ros::Duration(v);

        for(unsigned int i = 0; i < num_joints; ++i)
        {
            file >> v;
            p.positions.push_back(v);
        }

        for(unsigned int i = 0; i < num_joints; ++i)
        {
            file >> v;
            p.velocities.push_back(v);
        }

        goal.trajectory.points.push_back(p);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    std::vector<double> references(num_joints, 0);

    std::string id;
    std::stringstream error_msg;
    if (!refgen.setGoal(goal, id, error_msg))
    {
        std::cout << "Error: " << error_msg.str() << std::endl;
        return 1;
    }

    double dt = 0.01;
    double time = 0;

    GraphViewer g_pos("position");
    GraphViewer g_vel("velocity");
    GraphViewer g_acc("acceleration");

    std::vector<double> positions;
    std::vector<double> velocities;

    while(refgen.hasActiveGoals())
    {
        refgen.calculatePositionReferences(0.01, references);
        time += dt;

//        std::cout << time << ": " << references[0] << std::endl;

        for(unsigned int i = 0; i < num_joints; ++i)
        {
            const tue::manipulation::JointInfo& js = refgen.joint_state(i);
            if (positions.size() < i + 1)
            {
                positions.push_back(js.position());
                velocities.push_back(js.velocity());
            }
            else
            {
                double dx = js.position() - positions[i];
                double v = dx / dt;
                if (std::abs(v) > js.max_vel + 1e-6)
                    std::cout << "Joint " << i << " exceeded velocity: v = " << v << ", max v = " << js.max_vel << std::endl;
                positions[i] = js.position();

                double dv = js.velocity() - velocities[i];
                double a = dv / dt;
                if (std::abs(a) > js.max_acc + 1e-6)
                    std::cout << "Joint " << i << " exceeded acceleration: a = " << a << ", max a = " << js.max_acc << std::endl;
                velocities[i] = js.velocity();
            }

            g_pos.addPoint(0, i, time, js.position());
            g_vel.addPoint(0, i, time, js.velocity());
            g_acc.addPoint(0, i, time, js.acceleration());
        }
    }

    std::cout << "Total trajectory time: " << time << " seconds" << std::endl;

    g_vel.view();
    g_acc.view();
    g_pos.view(true);

    return 0;
}
