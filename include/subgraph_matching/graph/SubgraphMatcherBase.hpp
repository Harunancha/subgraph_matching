#ifndef SUBGRAPH_MATCHER_H_
#define SUBGRAPH_MATCHER_H_

#include "Graph.hpp"

class SubgraphMatcher
{
public:
    Graph query;
    Graph target;

    Graph optimum_result;

    SubgraphMatcher(/* args */){};
    ~SubgraphMatcher(){};

    virtual bool init(Graph &g1, Graph &g2){}; //inspect graphs for checking if it's ready for matching
    virtual void compress_target_graph(){}; //crop non-similar edges
    virtual bool matching(){}; //matching function
    virtual void sort_result(){};
};

#endif