#include "probability_roadmap.hpp"

#include "geometry_utils.hpp"

PROBABILITY_ROADMAP::PROBABILITY_ROADMAP(Graph g, int sample_count, int neighbor_count){
    tree = g;
    max_sample_count = sample_count;
    max_neighbor_count = neighbor_count;
    all_valid_nodes = tree.get_valid_nodes();
}

pair<cell,float> PROBABILITY_ROADMAP::get_furthest_neighbor(cell node, set<cell> neighbors){
    float max_dist = -1;
    auto furthest_neighbor = cell{-1,-1};
    for(auto n = neighbors.begin(); n != neighbors.end(); ++n){
        auto neighbor = cell{n->first, n->second};
        float new_dist = Distance::euclidean(node, neighbor);
        if(new_dist > max_dist){
            max_dist = new_dist;
            furthest_neighbor = neighbor;
        }
    }
    return {furthest_neighbor, max_dist};
}

void PROBABILITY_ROADMAP::find_nearest_neighbors(int k, c_time_point start, int timeout){
    for(auto it = kd_tree.begin(); it != kd_tree.end(); ++it){
        auto curr = it->first;
        for(auto it2 = kd_tree.begin(); it2 != kd_tree.end(); ++it2){
            auto now = high_resolution_clock::now();
            if(duration_cast<milliseconds>(now-start).count() >= timeout) return;
            if(it->first == it2->first) continue;
            else if(it->second.size() < k && Sampling::is_collision_free(curr, it2->first, tree))
                it->second.insert(it2->first);
            else {
                auto furthest = PROBABILITY_ROADMAP::get_furthest_neighbor(curr, it->second);
                if(furthest.second > Distance::euclidean(curr, it2->first) && furthest.first != cell{-1,-1}
                   && Sampling::is_collision_free(curr, it2->first, tree)){
                    it->second.erase(it->second.find(furthest.first));
                    it->second.insert(it2->first);
                }
            }
        }
    }
}

void PROBABILITY_ROADMAP::construct_roadmap(cell sp, cell ep, c_time_point start, int timeout){
    // Initialize KD tree with sample nodes
    set<cell> temp;
    kd_tree.insert({sp, temp});
    for(int s_idx = 0; s_idx < max_sample_count; s_idx++){
        auto node = Sampling::get_random_node(tree.end, all_valid_nodes);
        kd_tree.insert({node, set(temp)});
    }
    kd_tree.insert({ep, set(temp)});

    // Create roadmap for sample nodes (add collision-free edges)
    PROBABILITY_ROADMAP::find_nearest_neighbors(max_neighbor_count, start, timeout);
}

void PROBABILITY_ROADMAP::dijkstra(cell sp, cell ep, c_time_point start, int timeout){
    dist[sp] = 0;
    f[sp] = get_f_score(sp);
    vector<cell> open_set;
    open_set.push_back(sp);
    while(!open_set.empty()){
        auto now = high_resolution_clock::now();
        if(duration_cast<milliseconds>(now-start).count() >= timeout) return;
        cell curr = get_min_f(open_set);
        if(curr == ep) break;
        auto children = kd_tree[curr];
        for(auto child : children){
            auto w = Distance::euclidean(child, curr);
            auto new_dist = dist[curr] + w;
            if(new_dist < get_f_score(child)){
                f[child] = new_dist;
                dist[child] = new_dist;
                parent[child] = curr;
                if(not_in_set(open_set, child)){
                    open_set.push_back(child);
                    travelled.push_back(child);
                }
            }
        }
    }
}

void PROBABILITY_ROADMAP::solve(cell sp, cell ep, int timeout){
    auto start = high_resolution_clock::now();

    // Construction phase with sampled nodes 
    PROBABILITY_ROADMAP::construct_roadmap(sp, ep, start, timeout);
    //PROBABILITY_ROADMAP::print_roadmap();
    
    // Set all valid nodes to have infinite distance
    for(auto it = kd_tree.begin(); it != kd_tree.end(); ++it)
        dist[it->first] = std::numeric_limits<float>::infinity();

    // Query phase using Dijkstra's algorithm
    PROBABILITY_ROADMAP::dijkstra(sp, ep, start, timeout);
}

pair<vector<cell>, float> PROBABILITY_ROADMAP::reconstruct_path(pair<int, int> sp, pair<int, int> ep){
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

bool PROBABILITY_ROADMAP::not_in_set(vector<cell> open_set, cell p){
    for(auto n : open_set){
        if(n == p) return false;
    }
    return true;
}

cell PROBABILITY_ROADMAP::get_min_f(vector<cell> &s){
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

float PROBABILITY_ROADMAP::get_f_score(cell p){
    return dist[p];
}

/*void PROBABILITY_ROADMAP::set_step_size(int size){
    step_size = size;
}*/

vector<cell> PROBABILITY_ROADMAP::get_travelled_nodes(){
    return travelled;
}

vector<cell> PROBABILITY_ROADMAP::get_travelled_roadmap(){
    auto travelled_nodes = PROBABILITY_ROADMAP::get_travelled_nodes();
    set<cell> unique_nodes(travelled_nodes.begin(), travelled_nodes.end());
    for(int i = 0; i < travelled_nodes.size(); i++){
        auto curr = travelled_nodes[i];
        auto neighbors = kd_tree[curr];
        for(auto neighbor: neighbors){
            if(unique_nodes.find(neighbor) != unique_nodes.end() && Sampling::is_collision_free(curr, neighbor, tree)){
                auto line = Bresenham::connect_points(curr, neighbor);
                unique_nodes.insert(line.begin(), line.end());
            }
        }
    }
    vector<cell> visited;
    for(auto it = unique_nodes.begin(); it != unique_nodes.end(); ++it)
        visited.push_back(cell{it->first, it->second});
    return visited;
}

void PROBABILITY_ROADMAP::print_roadmap(){
    cout << "ROADMAP\n"; 
    for(auto t = kd_tree.begin(); t != kd_tree.end(); ++t){
        cout << "(" << t->first.first << "," << t->first.second << "): [ ";
        for(auto n: t->second)
            cout << "(" << n.first << "," << n.second << ") ";
        cout << "]\n";
    }
    cout << "\n";
}