#include "a_star.hpp"

using namespace std::chrono;

AStar::AStar(Graph g){
    tree = g;
    for(auto it = tree.g.begin(); it != tree.g.end(); ++it){
        dist[it->first] = std::numeric_limits<float>::infinity();
        h[it->first] = euclidean_heuristic(it->first, tree.end);
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
    vector<cell> open_set;
    open_set.push_back(sp);
    int kill_count = 0;
    auto start = high_resolution_clock::now();
    while(!open_set.empty() && kill_count < tree.get_size()){
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
                if(not_in_set(open_set, cp)) {
                    open_set.push_back(cp);
                    travelled.push_back(cp);
                }
            } 
        }
        kill_count++;
    }
}

bool AStar::not_in_set(vector<cell> open_set, cell p){
    for(auto n : open_set){
        if(n == p) return false;
    }
    return true;
}

cell AStar::get_min_f(vector<cell> &s){
    cell mp;
    int min_idx = -1;
    float min_val = std::numeric_limits<float>::infinity();
    for(int i = 0; i < s.size(); i++){
        auto n = s[i];
        if(f[n] <= min_val){
            min_idx = i;
            min_val = f[n];
        }
    }
    if(min_idx != -1){
        mp = {s[min_idx].first, s[min_idx].second};
        s.erase(s.begin()+min_idx);
    }
    return mp;
}

float AStar::get_f_score(cell p){
    return dist[p] + h[p];
}

float AStar::euclidean_heuristic(cell a, cell b){
    return sqrt(pow(a.first - b.first, 2) + pow(a.second - b.second, 2));
}

void AStar::print_map(string name, map<cell, float> map){
    cout << name << endl;
    for(auto m : map){
        cout << "[" << m.first.first << "," << m.first.second  << "] = " << m.second << endl;
    }
}

vector<cell> AStar::get_travelled_nodes(){
    return travelled;
}