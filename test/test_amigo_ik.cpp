#include <tue/manipulation/ik_solver.h>

#include <iostream>
#include <fstream>

#include <ros/package.h>

// ----------------------------------------------------------------------------------------------------

KDL::Frame createKDLFrame(double x, double y, double z, double rx, double ry, double rz)
{
    return KDL::Frame(KDL::Rotation::EulerZYX(rz, ry, rx), KDL::Vector(x, y, z));
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
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

    tue::IKSolver solver;

    std::string error;
    if (!solver.initFromURDF(urdf_xml, "base_link", "grippoint_right", error))
    {
        std::cout << error << std::endl;
        return 1;
    }

    // - - - - - - - - - - - Test the solver - - - - - - - - - - -

    KDL::JntArray q;
    std::cout << solver.cartesianToJoints(createKDLFrame(0.5, -0.2, 0.8, 0, 0, 0), q) << std::endl;

    return 0;
}
