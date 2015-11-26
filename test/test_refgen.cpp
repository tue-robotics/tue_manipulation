#include <tue/manipulation/reference_interpolator.h>
#include <tue/manipulation/graph_viewer.h>

#include <vector>
#include <iostream>
#include <cstdlib>

int main(int argc, char **argv)
{
    if (argc <= 1)
    {
        std::cout << "Please provide setpoints" << std::endl;
        return 1;
    }

    std::vector<double> goals((argc - 1) / 2);
    std::vector<double> vels((argc - 1) / 2);
    for(int i = 0; i < argc - 1;)
    {
        goals[i / 2] = atof(argv[1 + i++]);
        vels[i / 2]  = atof(argv[1 + i++]);
    }

    tue::manipulation::ReferenceInterpolator r;
    r.setState(goals[0], vels[0]);
    r.setMaxAcceleration(0.2);
    r.setMaxVelocity(0.4);

    GraphViewer viewer;

    double dt = 0.01;
    double time = 0;

    for(unsigned int i = 1; i < goals.size(); ++i)
    {
        std::cout << "Goal: x = " << goals[i] << ", v = " << vels[i] << std::endl;

        double t_needed = r.calculateTime(goals[i], vels[i]);
        std::cout << "t_needed = " << t_needed << std::endl;

        r.setGoal(goals[i], vels[i], t_needed);

        while(!r.done())
        {
            time += dt;
            r.update(dt);
            viewer.addPoint(0, 0, time, r.position());
            viewer.addPoint(0, 2, time, r.velocity());
            viewer.addPoint(0, 1, time, goals[i]);
        }
    }

    viewer.view(true);

    return 0;
}
