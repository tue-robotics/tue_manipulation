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

    std::vector<double> goals(argc - 1);
    for(int i = 1; i < argc; ++i)
        goals[i - 1] = atof(argv[i]);

    tue::manipulation::ReferenceInterpolator r;
    r.setState(0, 0);
    r.setMaxAcceleration(0.2);
    r.setMaxVelocity(1);

    GraphViewer viewer;

    double dt = 0.01;
    double time = 0;

    for(unsigned int i = 0; i < goals.size(); ++i)
    {
        r.setGoal(goals[i]);

        while(!r.done())
        {
            time += dt;
            r.update(dt);
            viewer.addPoint(0, 0, time, r.position());
            viewer.addPoint(0, 1, time, goals[i]);
            viewer.view();
        }
    }

    return 0;
}
