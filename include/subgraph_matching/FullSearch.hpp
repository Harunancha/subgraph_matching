#include "graph/SubgraphMatcherBase.hpp"

#include "utility_ros/Timer.hpp"

class SubgraphMatcherFullSearch : public SubgraphMatcher
{
public:
    std::vector<std::vector<Node>> candidate_node_set;
    std::string graph_shape;
    bool KLD;

    std::vector<Graph> matching_candidate;

    ////for log
    size_t node_size, edge_size;
    size_t candidate_size;
    double AVG_cost;
    size_t AVG_id;
    double KLD_cost;
    size_t KLD_id;

    SubgraphMatcherFullSearch(/* args */)
    {
        graph_shape = "full";
    };
    ~SubgraphMatcherFullSearch(){};

    bool init(Graph &g1, Graph &g2); //inspect graphs for checking if it's ready for matching
    // void commpress_target_graph();
    bool matching(); //matching function
    void sort_result();

    bool make_candidates();
};

bool SubgraphMatcherFullSearch::init(Graph &g1, Graph &g2) //inspect graphs for checking if it's ready for matching
{
    query = g1;
    target = g2;
    
    if (query.node_set.size()<target.node_set.size())
        return true;
    else
        return false;
}

// void SubgraphMatcherFullSearch::commpress_target_graph()
// {}

bool SubgraphMatcherFullSearch::make_candidates()
{
    ChronoTimer timer;
    timer.start();
    std::vector<Node> temp_query_node_set;
    candidate_node_set.clear();

    for (auto q : query.node_set) //0 for target_object
    {
        bool candidates_found = false;
        std::vector<Node> temp_candidate_node_set;
        temp_candidate_node_set.clear();
        for (auto t : target.node_set)
        {
            if (q.second.label_class == t.second.label_class || t.second.label_class == "unknown")
            {
                candidates_found = true;
                temp_candidate_node_set.push_back(t.second);
            }
        }
        if (candidates_found)
        {
            temp_query_node_set.push_back(q.second);
            candidate_node_set.push_back(temp_candidate_node_set);
        }
        else
            std::cout << "delete query node: " << q.second.label_class.c_str() << std::endl;
    }
    query.node_set.clear();
    query.n_id = 0;
    for (int i = 0; i < temp_query_node_set.size(); ++i)
    {
        query.add_node(temp_query_node_set[i]);
    }
    timer.lap("init");

    size_t id_num = 0;
    size_t max_sets = 1;
    for (size_t i = 0; i < candidate_node_set.size(); ++i)
    {
        max_sets *= candidate_node_set[i].size();
    }
    std::cout << "max_sets: " << max_sets << std::endl;

    bool upstare_flag = false;
    std::vector<size_t> current_id(candidate_node_set.size(), 0);
    std::vector<std::vector<size_t>> all_ids;
    size_t skip_count = 0;
    // std::vector<Node> query_node_set;
    // for (auto q : query.node_set)
    // {
        // query_node_set.push_back(q.second);
    // }
    for (size_t i = 0; i < max_sets; ++i)
    {
        Graph one_candidate;
        std::vector<size_t> id_now = current_id;
        bool same_id = false;
        // for (size_t i = 0; i < query_node_set.size(); ++i)
        for (auto ni : query.node_set)
        {
            // for (size_t j = 0; j < query_node_set.size(); ++j)
            for (auto nj : query.node_set)
            {
                if (ni.first >= nj.first)
                    continue;
                if (ni.second.label_class == nj.second.label_class)
                {
                    if (current_id[ni.first] == current_id[nj.first])
                        same_id = true;
                }
            }
        }
        for (size_t j = 0; j < candidate_node_set.size(); ++j)
        {
            one_candidate.add_node(candidate_node_set[j][current_id[j]]);
            if (upstare_flag || j == 0)
            {
                upstare_flag = true;
                current_id[j]++;
            }
            if (current_id[j] == candidate_node_set[j].size())
                current_id[j] = 0;
            else
                upstare_flag = false;
        }
        if (same_id)
        {
            skip_count++;
        }
        else
        {
            all_ids.push_back(id_now);
            ////配置関係考慮
            bool arrangement = true;
            if (arrangement)
            {
                one_candidate.id = id_num;
                id_num++;
                matching_candidate.push_back(one_candidate);
            }
        }
    }
    timer.lap("make_candidate");

    node_size = query.node_set.size();
    candidate_size = matching_candidate.size();
    query.connect(graph_shape);
    edge_size = query.edge_set.size();
    for (size_t i = 0; i < matching_candidate.size(); ++i)
    {
        // std::cout << i << ": ";
        // for (auto c : matching_candidate[i].node_set)
        // {
        //     std::cout << c.first <<","<< c.second.id << ":" << c.second.label_class << ", ";
        // }
        // std::cout << std::endl;
        matching_candidate[i].connect(graph_shape);
        matching_candidate[i].calculate_matching_cost(query);
    }
    timer.lap("calc_matching_cost");

    timer.end();
    if (matching_candidate.size() > 0)
        return true;
    else
        return false;
}

bool SubgraphMatcherFullSearch::matching() //matching function
{
    matching_candidate.clear();
    bool candidate_exist = make_candidates();
    std::cout << "t: " << target.node_set.size() << ", q: " << query.node_set.size() << ", c: " << matching_candidate.size() << ", e: " << query.edge_set.size() << std::endl;
    if (candidate_exist)
    {
        sort_result();
        return true;
    }
    else
    {
        std::cout << "\033[31mNo candidate \033[m" << std::endl;
        return false;
    }
}

void SubgraphMatcherFullSearch::sort_result()
{
    double min_cost = 1000000;
    size_t min_id = 1000000;
    for (size_t i = 0; i < matching_candidate.size(); ++i)
    {
        if (min_cost > matching_candidate[i].matching_cost)
        {
     		min_cost = matching_candidate[i].matching_cost;
     		min_id = i;
     	}
    }
    optimum_result = matching_candidate[min_id];
    std::cout << "min id: " << min_id << ", cost: " << min_cost << std::endl;
}
