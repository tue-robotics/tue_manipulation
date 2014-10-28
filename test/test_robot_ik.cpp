#include <tue/manipulation/ik_solver.h>

#include <iostream>
#include <fstream>

#include <ros/package.h>

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    // - - - - - - - - - - - Read the robot URDF description into a string - - - - - - - - - - -

    if (argc < 2) {
        std::cout << "Usage: 'rosrun test_robot_ik [robot_name]', with robot name either amigo or sergio" << std::endl;
        return 1;
    }

    std::string robot_name(argv[1]);
    bool use_constrained_solver;
    if (robot_name == "amigo") {
        use_constrained_solver = false;
    } else if (robot_name == "sergio") {
        use_constrained_solver = true;
    } else {
        std::cout << "Robot name must be either amigo or sergio" << std::endl;
        return 1;
    }

    std::string robot_urdf_path = ros::package::getPath(robot_name+"_description") + "/urdf/"+robot_name+".urdf";
    std::ifstream f(robot_urdf_path.c_str());

    if (!f.is_open())
    {
        std::cout << "Could not load URDF description: '" << robot_urdf_path << "'." << std::endl;
        return 1;
    }

    std::stringstream buffer;
    buffer << f.rdbuf();
    std::string urdf_xml = buffer.str();

    // - - - - - - - - - - - Initialize the solver - - - - - - - - - - -

    tue::IKSolver solver;

    std::string error;
    if (!solver.initFromURDF(urdf_xml, "base_link", "grippoint_right", 500, error, use_constrained_solver))
    {
        std::cout << error << std::endl;
        return 1;
    }

    // - - - - - - - - - - - Test the solver - - - - - - - - - - -

    KDL::JntArray q;
    if (solver.cartesianToJoints(KDL::Frame(KDL::Rotation::EulerZYX(0, 0, 0), KDL::Vector(0.5, -0.2, 0.8)), q))
        for(unsigned int i = 0; i < q.rows(); ++i)
            std::cout << "Joint " << i << ": " << q(i) << std::endl;
    else
        std::cout << "No joint solution found." << std::endl;

    return 0;
}
