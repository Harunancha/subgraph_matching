#ifndef NODE_H_
#define NODE_H_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "utility_ros/probability_util.hpp"

class Node
{
public:
    size_t id;
    std::string label_class;

    cv::Scalar color;
    cv::Point3d position3d;
    pd_u::NormalDistribution2d nd2d;

    std::map<size_t, Node> adjacent_ids; //ids of connected nodes

public:
    Node(/* args */){};
    ~Node(){};
};

#endif