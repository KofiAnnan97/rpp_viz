#ifndef BFS_HPP
#define BFS_HPP

#include <map_data.hpp>

class BFS{
    public:
        BFS(Graph g);
        void solve(cell sp, cell ep, int timeout);
        pair<vector<cell>, float> reconstruct_path(pair<int, int> sp, pair<int, int> ep);
        vector<cell> get_travelled_nodes();
    private:
        Graph tree;
        map<cell, float> dist;
        map<cell,bool> visited;
        map<cell, cell> parent;
        vector<cell> q;
};

#endif // BSF_HPP