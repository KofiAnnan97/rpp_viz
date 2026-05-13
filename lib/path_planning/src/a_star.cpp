#include "a_star.hpp"

#include "geometry_utils.hpp"

using namespace std::chrono;

AStar::AStar(Graph g){
    tree = g;
    for(auto it = tree.g.begin(); it != tree.g.end(); ++it){
        dist[it->first] = std::numeric_limits<float>::infinity();
        h[it->first] = Distance::euclidean(it->first, tree.end);
    }
}

pair<vector<cell>, float> AStar::reconstruct_path(cell sp, cell ep){
    auto data = pair<vector<cell>, float>();
    if(sp != ep) data.first.push_back(ep);
    auto curr = ep;
    while(curr != sp){
        curr = parent[curr];
        if(curr == cell{0, 0}) break; // Stop infinite loop if path not found
        data.first.insert(data.first.begin(), curr);
    }
    data.second = dist[ep];
    return data;
}

void AStar::solve(cell sp, cell ep, int timeout){
    dist[sp] = 0;
    f[sp] = AStar::get_f_score(sp);
    set<cell> open_set = {sp};
    auto start = high_resolution_clock::now();
    while(!open_set.empty()){ 
        auto now = high_resolution_clock::now();
        if(duration_cast<milliseconds>(now-start).count() >= timeout) break;
        cell curr = get_min_f(open_set);
        if(curr == ep) break;
        auto children = tree.get_edges(curr);
        for(auto c : children){
            auto cp = c.first;
            auto w = c.second;
            auto new_dist = dist[curr] + w;
            auto new_cost = new_dist + h[cp];
            if(new_cost < AStar::get_f_score(cp)){
                f[cp] = new_cost;
                dist[cp] = new_dist;
                parent[cp] = curr; 
                if(open_set.find(cp) == open_set.end()){
                    open_set.insert(cp);
                    travelled.push_back(cp);
                }
            } 
        }
    }
}

cell AStar::get_min_f(set<cell> &s){
    cell temp = {-1,-1};
    float min_val = std::numeric_limits<float>::infinity();
    for(auto it = s.begin(); it != s.end(); ++it){
        auto n = cell{it->first, it->second};
        if(f[n] <= min_val){
            temp = n;
            min_val = f[n];
        }
    }
    cell mp = {temp.first, temp.second}; 
    if(temp != cell{-1,-1}) s.erase(temp);
    return mp;
}

float AStar::get_f_score(cell p){
    return dist[p] + h[p];
}

void AStar::print_map(string name, unordered_map<cell, float> map){
    cout << name << endl;
    for(auto m : map){
        cout << "[" << m.first.first << "," << m.first.second  << "] = " << m.second << endl;
    }
}

vector<cell> AStar::get_travelled_nodes(){
    return travelled;
}