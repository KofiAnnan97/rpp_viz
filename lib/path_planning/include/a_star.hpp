#ifndef A_STAR_HPP
#define A_STAR_HPP

#include <limits>
#include <cmath>
#include "map_data.hpp"

class AStar{
    public:
        AStar(Graph g);
        void solve(cell sp, cell ep, int timeout);
        pair<vector<cell>, float> reconstruct_path(cell sp, cell ep);
        void print_map(string name, map<cell, float> map);
        vector<cell> get_travelled_nodes();

    private:
        float get_f_score(cell p);
        float euclidean_heuristic(cell a, cell b);
        bool not_in_set(vector<cell> open_set, cell p);
        cell get_min_f(vector<cell> &s);
        Graph tree;
        map<cell, float> dist;
        map<cell, float> h;
        map<cell, float> f;
        map<cell, cell> parent;
        vector<cell> travelled;
        
};

#endif // A_STAR_HPP