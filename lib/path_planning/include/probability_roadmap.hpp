#ifndef PROBABILITY_ROADMAP_HPP
#define PROBABILITY_ROADMAP_HPP

#include <unordered_set>
#include <set>

#include "map_data.hpp"

using namespace std::chrono;

class PROBABILITY_ROADMAP{
    public:
        PROBABILITY_ROADMAP(Graph g, int sample_count, int neighbor_count);
        void dijkstra(cell sp, cell ep, c_time_point start, int timeout);
        void solve(cell sp, cell ep, int timeout);
        pair<vector<cell>, float> reconstruct_path(pair<int, int> sp, pair<int, int> ep);
        //void set_step_size(int size);
        vector<cell> get_connected_path(vector<cell> path);
        vector<cell> get_travelled_nodes();
        vector<cell> get_travelled_roadmap();

    private:
        void construct_roadmap(cell sp, cell ep, c_time_point start, int timeout);
        pair<cell,float> get_furthest_neighbor(cell node, set<cell> neighbors);
        void find_nearest_neighbors(int k, c_time_point start, int timeout);
        bool not_in_set(vector<cell> open_set, cell p);
        cell get_min_f(vector<cell> &s);
        float get_f_score(cell p);
        void print_roadmap();

        Graph tree;
        unordered_map<cell, float> dist;
        unordered_map<cell, float> f;
        unordered_map<cell, set<cell>> kd_tree;
        unordered_map<cell, cell> parent;
        vector<cell> travelled;
        vector<cell> all_valid_nodes;
        vector<cell> all_obstacle_nodes;
        int max_sample_count;
        int max_neighbor_count;
        //int step_size;
};

#endif // PROBABILITY_ROADMAP_HPP