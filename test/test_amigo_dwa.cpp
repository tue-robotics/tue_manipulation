#include <tue/manipulation/dwa.h>

#include <iostream>
#include <fstream>

#include <ros/package.h>

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

    tue::manipulation::DWA dwa;

    std::string error;
    if (!dwa.initFromURDF(urdf_xml, "base_link", "grippoint_right", error))
    {
        std::cout << error << std::endl;
        return 1;
    }

    KDL::JntArray q(8);
    for(unsigned int i = 0; i < q.rows(); ++i)
        q(i) = 0;


    double dt = 0.01;
    while(true)
    {
        for(unsigned int i = 0; i < q.rows(); ++i)
        {
            double vel = dwa.calculateVelocity(q, i);
            q(i) += (vel * dt);
        }

        for(unsigned int i = 0; i< q.rows(); ++i)
            std::cout << q(i) << " ";
        std::cout << std::endl;

        usleep(dt * 1000000);
    }


    return 0;
}
