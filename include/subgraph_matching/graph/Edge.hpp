#ifndef EDGE_H_
#define EDGE_H_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include "opencv2/video/tracking.hpp"

class Edge
{
public:
    std::pair<size_t, size_t> id;

    double label_distance;

public:
    Edge(/* args */){};
    Edge(size_t n1, size_t n2)
    {
        id = std::make_pair(n1, n2);
    };
    ~Edge(){};
};

#endif