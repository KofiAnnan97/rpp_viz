#ifndef RRT_STAR_HPP
#define RRT_STAR_HPP

#include <limits>
#include <cmath>
#include <random>
#include <complex>
#include "map_data.hpp"

class RRTStar{
    public:
        RRTStar(Graph g, float step_size, int width, int height, int max_iter);
        void solve(pair<int,int> sp, pair<int,int> ep);
        pair<vector<pair<int,int>>, float> reconstruct_path(pair<int, int> sp, pair<int, int> ep);
        bool goal_reached;

        vector<pair<int,int>> get_travelled_nodes(pair<int,int> sp, pair<int,int> ep);

    private:
        pair<int,int> get_random_node();
        pair<int,int> get_nearest_node(vector<pair<int,int>> node_list, pair<int,int> random_node);
        pair<int,int> steer(pair<int,int> from_node, pair<int,int> to_node);
        vector<pair<int,int>> find_neighbors(vector<pair<int,int>> node_list, pair<int,int> node);
        pair<int,int> choose_parent(vector<pair<int,int>> neighbors, pair<int,int> nearest_node, pair<int,int> new_node);
        void rewire(pair<int,int> new_node, vector<pair<int,int>> neighbors);
        float euclidean_distance(pair<int, int> a, pair<int, int> b);
        Graph tree;
        int max_iter, height, width;
        float step_size;
        map<pair<int,int>, float> cost_map;
        vector<pair<int,int>> node_list;
        vector<pair<int,int>> all_valid_nodes;
        vector<pair<int,int>> travelled;
        map<pair<int,int>, pair<int,int>> parent;
};

#endif // RRT_STAR_HPP