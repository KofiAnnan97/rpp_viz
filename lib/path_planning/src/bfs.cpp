#include "bfs.hpp"

BFS::BFS(Graph g){
    tree = g;
    for(auto it = tree.g.begin(); it != tree.g.end(); ++it){
        visited[it->first] = false;
        dist[it->first] = 0;
    }
}
        
void BFS::solve(cell sp, cell ep){
    q.push_back(sp);
    visited[sp] = true;
    while(!q.empty()){
        cell curr = {q[0].first, q[0].second};
        q.erase(q.begin());
        auto children = tree.get_edges(curr);
        for(auto c: children){
            auto cp = c.first;
            auto w = c.second;
            if(visited[cp] == false){
                q.push_back(cp);
                visited[cp] = true;
                parent[cp] = curr;
                dist[cp] = dist[curr] + w;
            }
        }
    }
}
        
pair<vector<cell>, float> BFS::reconstruct_path(cell sp, cell ep){
    auto data = pair<vector<cell>, float>();
    auto curr = ep;
    data.first.push_back(curr);
    while(true){
        curr = parent[curr];
        if(curr == cell{0, 0}) break; // Stop infinite loop if path not found
        else data.first.insert(data.first.begin(), curr);
    }
    if(data.first[0] != sp) data.first = vector<cell>();
    data.second = dist[ep];
    return data;
}

vector<cell> BFS::get_travelled_nodes(){
    vector<cell> travelled;
    for(auto v: visited){
        if(v.second) travelled.push_back(v.first);
    }
    return travelled;
}