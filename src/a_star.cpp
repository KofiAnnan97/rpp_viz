#include "a_star.hpp"

AStar::AStar(Graph g){
    tree = g;
    for(auto it = tree.g.begin(); it != tree.g.end(); ++it){
        dist[it->first] = std::numeric_limits<float>::infinity();
        h[it->first] = euclidean_heuristic(it->first, tree.end);
        //f[it->first] = get_f_score(it->first);
    }
}

pair<vector<pair<int,int>>, float> AStar::reconstruct_path(pair<int, int> sp, pair<int, int> ep){
    auto data = pair<vector<pair<int,int>>, float>();
    if(sp != ep) data.first.push_back(ep);
    auto curr = ep;
    while(curr != sp){
        curr = parent_node[curr];
        if(curr == pair<int, int>{0, 0}) break; // Stop infinite loop if path not found
        data.first.insert(data.first.begin(), curr);
    }
    data.second = dist[ep];
    return data;
}

void AStar::solve(pair<int, int> sp, pair<int, int> ep){
    dist[sp] = 0;
    f[sp] = AStar::get_f_score(sp);
    vector<pair<int,int>> open_set;
    open_set.push_back(sp);
    int kill_count = 0;
    while(!open_set.empty() && kill_count < tree.get_size()){
        pair<int,int> curr = get_min_f(open_set);
        if(curr == tree.end) break;
        auto children = tree.get_edges(curr);
        for(auto c : children){
            auto cp = c.first;
            auto w = c.second;
            auto new_dist = dist[curr] + w;
            auto new_cost = new_dist + h[cp];
            if(new_cost < AStar::get_f_score(cp)){
                f[cp] = new_cost;
                dist[cp] = new_dist;
                parent_node[cp] = curr; 
                if(not_in_set(open_set, cp)) open_set.push_back(cp);
            } 
        }
        kill_count++;
    }
}

bool AStar::not_in_set(vector<pair<int,int>> open_set, pair<int,int> p){
    for(auto n : open_set){
        if(n == p) return false;
    }
    return true;
}

pair<int,int> AStar::get_min_f(vector<pair<int,int>> &s){
    pair<int,int> mp;
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

float AStar::get_f_score(pair<int, int> p){
    return dist[p] + h[p];
}

float AStar::euclidean_heuristic(pair<int, int> a, pair<int, int> b){
    return sqrt(pow(a.first - b.first, 2) + pow(a.second - b.second, 2));
}

void AStar::print_map(string name, map<pair<int, int>, float> map){
    cout << name << endl;
    for(auto m : map){
        cout << "[" << m.first.first << "," << m.first.second  << "] = " << m.second << endl;
    }
}