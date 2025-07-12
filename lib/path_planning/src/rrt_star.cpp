#include "rrt_star.hpp"

using namespace std::chrono;

const float PI = 3.14159;

RRTStar::RRTStar(Graph g, int iter){
    tree = g;
    max_iter = iter;
    all_valid_nodes = tree.get_nodes();
    goal_reached = false;
}

void RRTStar::solve(cell sp, cell ep, int timeout){
    node_list.push_back(sp);
    //int compute_timeout = 60000;
    auto start = high_resolution_clock::now();
    for(int i = 0; i < max_iter; i++){
        //if(i%1000 == 0) cout << "Iteration: " << i << endl;
        auto now = high_resolution_clock::now();
        if(duration_cast<milliseconds>(now-start).count() >= timeout) break;
        auto random_node = get_random_node();
        auto nearest_node = get_nearest_node(node_list, random_node);
        auto new_node = steer(nearest_node, random_node);
        //cout << "Random node: (" << random_node.first <<"," <<random_node.second <<")\n";
        //cout << "Nearest node: (" << nearest_node.first <<"," << nearest_node.second <<")\n";
        //cout << "New node: (" << new_node.first <<"," <<new_node.second <<")\n";
        if(tree.is_node_valid(new_node)){
            auto neighbors = find_neighbors(node_list, new_node);
            new_node = choose_parent(neighbors, nearest_node, new_node);
            node_list.push_back(new_node);
            rewire(new_node, neighbors);
            travelled.push_back(new_node);
        }   
        if(RRTStar::euclidean_distance(new_node,ep) <= 1.5){
            goal_reached = true;
            if(parent[new_node] != ep) {
                parent[ep] = new_node;
                cost_map[ep] = cost_map[new_node] + euclidean_distance(new_node,ep);
            }
            break;
        }    
    }
    /*for(auto pair: parent){
        auto key = pair.first;
        auto val = pair.second;
        cout << "P[" << val.first << "," << val.second << "] <= C[";
        cout << key.first << "," << key.second << "]\n"; 
    }*/
}

pair<vector<cell>, float> RRTStar::reconstruct_path(cell sp, cell ep){
    auto data = pair<vector<cell>, float>();
    if(sp != ep) data.first.push_back(ep);
    auto curr = ep;
    while(curr != sp){
        curr = parent[curr];
        if(curr == cell{0, 0}) break; // Stop infinite loop if path not found
        data.first.insert(data.first.begin(), curr);
    }
    data.second = cost_map[ep];
    return data;
}

cell RRTStar::get_random_node(){
    double r = (double)rand()/(double)RAND_MAX;
    cell random_node;
    if(r > 0.2) {
        int r_idx = rand()%all_valid_nodes.size();
        random_node = {all_valid_nodes[r_idx].first, all_valid_nodes[r_idx].second};
        all_valid_nodes.erase(all_valid_nodes.begin()+r_idx);
        all_valid_nodes.push_back(random_node);
    }
    else random_node = {tree.end.first, tree.end.second};
    return random_node;
}

cell RRTStar::get_nearest_node(vector<cell> node_list, cell random_node){
    cell nearest_node;
    float min_dist = std::numeric_limits<float>::infinity();
    for(auto n: node_list){
        float dist = RRTStar::euclidean_distance(random_node, n); 
        if(dist < min_dist){
            nearest_node = n;
            min_dist = dist;
        }
    }
    return nearest_node;
}

vector<cell> RRTStar::find_neighbors(vector<cell> node_list, cell node){
    vector<cell> neighbors;
    for(cell n: node_list){
        if(RRTStar::euclidean_distance(n, node) < 2) neighbors.push_back(n);
    }
    return neighbors;
}

cell RRTStar::steer(cell from_node, cell to_node){
    float theta = atan2f32(float(to_node.second - from_node.second), float(to_node.first - from_node.first));
    int closest_x = std::round(from_node.first + cos(theta*180.0/PI));
    int closest_y = std::round(from_node.second + sin(theta*180.0/PI));
    cell new_node = {closest_x, closest_y};
    cost_map[new_node] = cost_map[from_node] + RRTStar::euclidean_distance(from_node, new_node);
    parent[new_node] = from_node;
    return new_node;
}

cell RRTStar::choose_parent(vector<cell> neighbors, cell nearest_node, cell new_node){
    //std::complex<float> nv (new_node.first-nearest_node.first, new_node.second-nearest_node.second); // norm of vector
    float min_cost = cost_map[nearest_node] + euclidean_distance(new_node,nearest_node);//sqrt(std::norm(nv));
    cell best_node = {nearest_node.first, nearest_node.second};
    for(auto n: neighbors){
        //std::complex<float> nv2 (new_node.first-n.first, new_node.second-n.second); // norm of vector
        float cost = cost_map[n] + euclidean_distance(new_node,n);//sqrt(std::norm(nv2));
        if(cost < min_cost && tree.is_node_valid(n)){
            best_node = {n.first,n.second};
            min_cost = cost;
        }
    }
    cost_map[new_node] = min_cost;
    parent[new_node] = best_node;
    return new_node;
}

void RRTStar::rewire(cell new_node, vector<cell> neighbors){
    for(auto n: neighbors){
        //std::complex<float> nv (n.first-new_node.first, n.second-new_node.second); // norm of vector
        float cost = cost_map[new_node] + euclidean_distance(new_node,n);//sqrt(std::norm(nv));  
        if(cost < cost_map[n] && tree.is_node_valid(new_node)){
            parent[n] = new_node;
            cost_map[n] = cost;
        }
    }
}

float RRTStar::euclidean_distance(cell a, cell b){
    return sqrt(pow(a.first - b.first, 2) + pow(a.second - b.second, 2));
}

vector<cell> RRTStar::get_travelled_nodes(){
    return travelled;
}