#ifndef BFS_HPP
#define BFS_HPP

#include <map_data.hpp>

class BFS{
    public:
        BFS(Graph g);
        void solve(pair<int, int> sp, pair<int, int> ep);
        pair<vector<pair<int,int>>, float> reconstruct_path(pair<int, int> sp, pair<int, int> ep);

    private:
        Graph tree;
        map<pair<int,int>, float> dist;
        map<pair<int,int>,bool> visited;
        map<pair<int,int>, pair<int,int>> parent;
        vector<pair<int,int>> q;
};

#endif // BSF_HPP