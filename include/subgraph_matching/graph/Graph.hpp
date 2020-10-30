#ifndef GRAPH_H_
#define GRAPH_H_

#include <map>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/video/tracking.hpp"

#include "utility_ros/geometry_util.hpp"
// #include "subgraph_matching/graph_utility.hpp"
#include "Node.hpp"
#include "Edge.hpp"

#include <iostream>

class Graph
{
public:
    size_t id;
    size_t n_id;
    std::map<size_t, Node> node_set;
    // std::map<std::pair<size_t, size_t>, Edge> edge_set;
    std::vector<Edge> edge_set;

    double matching_cost;

public:
    Graph(/* args */): n_id(0){};
    ~Graph(){};
    void add_node(Node &n)
    {
        node_set.insert(std::make_pair(n_id, n));
        n_id++;
    };
    void add_edge(size_t n1, size_t n2)
    {
        if (node_set.count(n1) > 0 && node_set.count(n2) > 0)
        {
            Edge e(n1, n2);
            //edge-label
            e.label_distance = geo_u::Distance3d(node_set[n1].position3d, node_set[n2].position3d);
            // edge_set.insert(std::make_pair(e.id, e));
            edge_set.push_back(e);
        }
    };
    void connect(std::string graph_shape) //TODO: deprecated
    {
        edge_set.clear();
        if (node_set.size() < 2)
            std::cout << "node shortage... number of edges zero." << std::endl;
            if (node_set.size() == 2)
                add_edge(node_set[0].id, node_set[1].id);
            else if (graph_shape == "full")
            {
                for (auto ni : node_set)
                    for (auto nj : node_set)
                        if (ni.first < nj.first)
                            add_edge(ni.first, nj.first);
            }
            else if (graph_shape == "sector")
            {
                for (size_t i = 1; i < node_set.size(); i++) //spokes
                    add_edge(0, i);
                for (size_t i = 1; i < node_set.size()-1; i++) //edges
                    add_edge(i, i + 1);
            }
            else
                std::cout << "invalid graph_shape" << std::endl;
    };
    void calculate_matching_cost(Graph q)
    {
        matching_cost = 0;
        for (size_t i = 0; i < edge_set.size(); i++)
            matching_cost += fabs(edge_set[i].label_distance - q.edge_set[i].label_distance);
    };
};

#endif