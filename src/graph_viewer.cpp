#include "tue/manipulation/graph_viewer.h"

#include <opencv2/highgui/highgui.hpp>

// ----------------------------------------------------------------------------------------------------

GraphViewer::GraphViewer(const std::string& name) : name_(name), num_points_(0), redraw_(true)
{
}

// ----------------------------------------------------------------------------------------------------

GraphViewer::~GraphViewer()
{
}

// ----------------------------------------------------------------------------------------------------

void GraphViewer::addPoint(int graph_id, int label, double x, double y, double y_dot)
{
    if (num_points_ == 0)
    {
        x_min_ = x;
        y_min_ = y;
        x_max_ = x;
        y_max_ = y;
    }
    else
    {
        if (x < x_min_) { x_min_ = x; redraw_ = true; }
        if (y < y_min_) { y_min_ = y; redraw_ = true; }
        if (x > x_max_) { x_max_ = x; redraw_ = true; }
        if (y > y_max_) { y_max_ = y; redraw_ = true; }
    }

    points_.push_back(Point(label, x, y, y_dot));

    ++num_points_;
}

// ----------------------------------------------------------------------------------------------------

void GraphViewer::clear()
{
    points_.clear();
    num_points_ = 0;
}

// ----------------------------------------------------------------------------------------------------

void GraphViewer::view(bool wait)
{
    static cv::Scalar COLORS[] = { cv::Scalar(255, 0,   0), cv::Scalar(0, 0, 255),   cv::Scalar(0, 255, 0),
                                   cv::Scalar(255, 255, 0), cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 255),
                                   cv::Scalar(0, 0, 0),     cv::Scalar(128, 128, 128), cv::Scalar(100, 255, 0) };

    static int NUM_COLORS = 9;

    if (redraw_)
    {
        redraw_ = false;
    }

    if (num_points_ < 2)
        return;

    cv::Mat canvas(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));

    double width = canvas.cols;
    double height = canvas.rows;

    for(unsigned int i = 0; i < points_.size(); ++i)
    {
        const Point& p = points_[i];

        cv::Point2d p_canvas((p.x - x_min_) * width / (x_max_ - x_min_),
                             (p.y - y_min_) * height / (y_max_ - y_min_));
        cv::circle(canvas, p_canvas, 2, COLORS[p.label % NUM_COLORS], 1);

        if (p.y_dot != 1e9)
        {
            int w = 5;
            double rc = p.y_dot / (width / (x_max_ - x_min_)) * (height / (y_max_ - y_min_));

            cv::Point2d p1(p_canvas.x - w, p_canvas.y - w * rc);
            cv::Point2d p2(p_canvas.x + w, p_canvas.y + w * rc);
            cv::line(canvas, p1, p2, COLORS[p.label], 2);
        }
    }

    int y0 = (0 - y_min_) * height / (y_max_ - y_min_);
    cv::line(canvas, cv::Point(0, y0), cv::Point(canvas.cols, y0), cv::Scalar(0, 0, 0), 1);

    cv::imshow(name_.c_str(), canvas);

    char key;
    if (wait)
        key = cv::waitKey();
    else
        key = cv::waitKey(3);

    if (key == 'c')
        clear();
}

// ----------------------------------------------------------------------------------------------------
