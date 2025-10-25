#ifndef RRT_STAR_HPP
#define RRT_STAR_HPP

#include <limits>
#include <cmath>
#include <random>
#include <complex>
#include <set>

#include "map_data.hpp"

class RRTStar{
    public:
        RRTStar(Graph g, int num_of_samples);
        void solve(cell sp, cell ep, int timeout);
        pair<vector<cell>, float> reconstruct_path(cell sp, cell ep);
        void set_step_size(float size);
        vector<cell> get_travelled_nodes();
        vector<cell> get_travelled_tree();
        
        bool goal_reached;

    private:
        cell get_nearest_node(vector<cell> node_list, cell random_node);
        cell steer(cell from_node, cell to_node);
        vector<cell> find_neighbors(vector<cell> node_list, cell node);
        cell choose_parent(vector<cell> neighbors, cell nearest_node, cell new_node);
        void rewire(cell new_node, vector<cell> neighbors);
        
        Graph tree;
        int max_num_of_samples;
        float goal_radius;
        float step_size = AlgoConstants::DEFAULT_STEP_SIZE;
        float search_radius = AlgoConstants::DEFAULT_SEARCH_RADIUS;
        unordered_map<cell, float> cost_map;
        vector<cell> node_list;
        vector<cell> all_valid_nodes;
        vector<cell> travelled;
        unordered_map<cell, cell> parent;
};

#endif // RRT_STAR_HPP