#ifndef A_STAR_HPP
#define A_STAR_HPP

#include <limits>
#include <cmath>
#include "map_data.hpp"

class AStar{
    public:
        AStar(Graph g);
        void solve(pair<int, int> sp, pair<int, int> ep);
        pair<vector<pair<int,int>>, float> reconstruct_path(pair<int, int> sp, pair<int, int> ep);
        void print_map(string name, map<pair<int, int>, float> map);

    private:
        float get_f_score(pair<int, int> p);
        float euclidean_heuristic(pair<int, int> a, pair<int, int> b);
        bool not_in_set(vector<pair<int,int>> open_set, pair<int,int> p);
        pair<int,int> get_min_f(vector<pair<int,int>> &s);
        Graph tree;
        map<pair<int,int>, float> dist;
        map<pair<int,int>, float> h;
        map<pair<int,int>, float> f;
        map<pair<int,int>, pair<int,int>> parent;
        
};

#endif // A_STAR_HPP