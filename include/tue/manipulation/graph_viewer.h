#ifndef _GRAPH_VIEWER_H_
#define _GRAPH_VIEWER_H_

#include <vector>
#include <string>

class GraphViewer
{

public:

    GraphViewer(const std::string& name = "");

    ~GraphViewer();

    void setName(const std::string& name) { name_ = name; }

    void addPoint(int graph_id, int label, double x, double y, double y_dot = 1e9);

    void view(bool wait = false);

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

    std::string name_;

    unsigned int num_points_;

    double x_min_;
    double x_max_;
    double y_min_;
    double y_max_;

    bool redraw_;

    std::vector<Point> points_;

};

#endif
