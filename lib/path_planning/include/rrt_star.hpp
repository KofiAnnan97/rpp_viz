#ifndef RRT_STAR_HPP
#define RRT_STAR_HPP

#include <limits>
#include <cmath>
#include <random>
#include <complex>
#include "map_data.hpp"

class RRTStar{
    public:
        RRTStar(Graph g, int width, int height, int max_iter);
        void solve(cell sp, cell ep);
        pair<vector<cell>, float> reconstruct_path(cell sp, cell ep);
        bool goal_reached;
        vector<cell> get_travelled_nodes();

    private:
        cell get_random_node();
        cell get_nearest_node(vector<cell> node_list, cell random_node);
        cell steer(cell from_node, cell to_node);
        vector<cell> find_neighbors(vector<cell> node_list, cell node);
        cell choose_parent(vector<cell> neighbors, cell nearest_node, cell new_node);
        void rewire(cell new_node, vector<cell> neighbors);
        float euclidean_distance(cell a, cell b);
        Graph tree;
        int max_iter, height, width;
        map<cell, float> cost_map;
        vector<cell> node_list;
        vector<cell> all_valid_nodes;
        vector<cell> travelled;
        map<cell, cell> parent;
};

#endif // RRT_STAR_HPP