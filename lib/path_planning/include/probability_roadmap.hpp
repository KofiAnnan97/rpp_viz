#ifndef PROBABILITY_ROADMAP_HPP
#define PROBABILITY_ROADMAP_HPP

#include <unordered_set>
#include <set>

#include "map_data.hpp"
#include "time_helper.hpp"

using namespace std::chrono;

class PROBABILITY_ROADMAP{
    public:
        PROBABILITY_ROADMAP(Graph g, int iter, int neighbor_count);
        void dijkstra(cell sp, cell ep, c_time_point start, int timeout);
        void solve(cell sp, cell ep, int timeout);
        pair<vector<cell>, float> reconstruct_path(pair<int, int> sp, pair<int, int> ep);
        vector<cell> get_connected_path(vector<cell> path);
        vector<cell> get_travelled_nodes();
        vector<cell> get_travelled_roadmap();

    private:
        void learn(cell sp, cell ep, c_time_point start, int timeout);
        cell get_random_node();
        bool is_collision_free(cell c, cell d);
        void find_nearest_neighbors(int k, c_time_point start, int timeout);
        bool not_in_set(vector<cell> open_set, cell p);
        cell get_min_f(vector<cell> &s);
        float get_f_score(cell p);
        void print_roadmap();
        int step_dist = 20;

        Graph tree;
        map<cell, float> dist;
        map<cell, float> f;
        map<cell, set<cell>> kd_tree;
        map<cell, cell> parent;
        vector<cell> travelled;
        vector<cell> all_valid_nodes;
        vector<cell> all_obstacle_nodes;
        int max_sample_count;
        int max_neighbor_count;
};

#endif // PROBABILITY_ROADMAP_HPP