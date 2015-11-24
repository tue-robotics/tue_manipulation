#ifndef _GRAPH_VIEWER_H_
#define _GRAPH_VIEWER_H_

#include <vector>

class GraphViewer
{

public:

    GraphViewer();

    ~GraphViewer();

    void addPoint(int graph_id, int label, double x, double y, double y_dot = 1e9);

    void view();

    void clear();

private:

    struct Point
    {
        Point(int label_, double x_, double y_, double y_dot_) : label(label_), x(x_), y(y_), y_dot(y_dot_) {}

        int label;
        double x;
        double y;
        double y_dot;
    };

    unsigned int num_points_;

    double x_min_;
    double x_max_;
    double y_min_;
    double y_max_;

    bool redraw_;

    std::vector<Point> points_;

};

#endif