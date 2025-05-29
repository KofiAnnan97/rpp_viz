#ifndef A_STAR_HPP
#define A_STAR_HPP

#include <limits>
#include <queue>
#include <cmath>
#include <map>
#include "map_data.hpp"

class AStar{
    public:
        AStar(Graph g);
        void solve(pair<int, int> sp, pair<int, int> ep);
        pair<vector<pair<int,int>>, float> reconstruct_path();
        void print_map(string name, map<pair<int, int>, float> map);

    private:
        float get_f_score(pair<int, int> p);
        float get_distance(pair<int, int> a, pair<int, int> b);
        int get_max_idx_by_f_score(queue<pair<int, int>> &q, int s_idx);
        void insert_max_to_rear(queue<pair<int, int>> &q, int min_idx);
        void sort_queue(queue<pair<int, int>> &q);
        Graph tree;
        map <pair<int,int>, float> dist;
        map <pair<int, int>, float> h;
        map <pair<int, int>, float> f;
        queue<pair<int,int>> q;
        map <pair<int, int>, pair<int, int>> via;
};

#endif // A_STAR_HPP