#ifndef TRANSFORM_ESTIMATOR_H_
#define TRANSFORM_ESTIMATOR_H_

#include "Graph.hpp"

#include "utility_ros/geometry_util.hpp"

class TransformEstimator
{
public:
    std::vector<std::pair<Node, Node>> matched_node_set;

    double residual_error;

    TransformEstimator(/* args */){};
    ~TransformEstimator(){};

    virtual bool init(Graph &g1, Graph &g2){}; //inspect graphs for checking if it's ready for matching
    virtual geo_u::Transform2d estimate(){}; //
};

#endif