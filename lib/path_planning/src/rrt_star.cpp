#include "rrt_star.hpp"

#include "geometry_utils.hpp"

using namespace std::chrono;

RRTStar::RRTStar(Graph g, int num_of_samples){
    tree = g;
    max_num_of_samples = num_of_samples;
    goal_radius = (search_radius >= 2) ? (3/4.0)*search_radius : 0.85*search_radius;
    all_valid_nodes = tree.get_valid_nodes();
    goal_reached = false;
}

void RRTStar::solve(cell sp, cell ep, int timeout){
    node_list.push_back(sp);
    auto start = high_resolution_clock::now();
    for(int i = 0; i < max_num_of_samples; i++){
        //if(i%1000 == 0) cout << "Iteration: " << i << endl;
        auto now = high_resolution_clock::now();
        if(duration_cast<milliseconds>(now-start).count() >= timeout) break;
        auto random_node = Sampling::get_random_node(tree.end, all_valid_nodes);
        auto nearest_node = get_nearest_node(node_list, random_node);
        auto new_node = steer(nearest_node, random_node);
        //cout << "Random node: (" << random_node.first <<"," <<random_node.second <<")\n";
        //cout << "Nearest node: (" << nearest_node.first <<"," << nearest_node.second <<")\n";
        //cout << "New node: (" << new_node.first <<"," <<new_node.second <<")\n";
        if(Sampling::is_collision_free(nearest_node, new_node, tree)){
            auto neighbors = find_neighbors(node_list, new_node);
            new_node = choose_parent(neighbors, nearest_node, new_node);
            node_list.push_back(new_node);
            rewire(new_node, neighbors);
            travelled.push_back(new_node);
        }   
        if(Distance::euclidean(new_node,ep) <= goal_radius && Sampling::is_collision_free(new_node, ep, tree)){
            goal_reached = true;
            if(parent[new_node] != ep) {
                parent[ep] = new_node;
                cost_map[ep] = cost_map[new_node] + Distance::euclidean(new_node,ep);
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
    set<cell> loop_check;
    while(curr != sp){
        curr = parent[curr];
        if(curr == cell{0, 0}) break; // Stop infinite loop if path not found
        if(loop_check.find(curr) != loop_check.end()) break; // Stop infinite loop for finding parent node
        data.first.insert(data.first.begin(), curr);
        loop_check.insert(curr);
    }
    data.second = cost_map[ep];
    return data;
}

cell RRTStar::get_nearest_node(vector<cell> node_list, cell random_node){
    cell nearest_node;
    float min_dist = std::numeric_limits<float>::infinity();
    for(auto n: node_list){
        float dist = Distance::euclidean(random_node, n); 
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
        if(Distance::euclidean(n, node) < search_radius) neighbors.push_back(n);
    }
    return neighbors;
}

cell RRTStar::steer(cell from_node, cell to_node){
    float theta = atan2f32(float(to_node.second - from_node.second), float(to_node.first - from_node.first));
    int closest_x = std::round(from_node.first + step_size*cos(theta*180.0/MathConstants::PI));
    int closest_y = std::round(from_node.second + step_size*sin(theta*180.0/MathConstants::PI));
    cell new_node = {closest_x, closest_y};
    cost_map[new_node] = cost_map[from_node] + Distance::euclidean(from_node, new_node);
    parent[new_node] = from_node;
    return new_node;
}

cell RRTStar::choose_parent(vector<cell> neighbors, cell nearest_node, cell new_node){
    //std::complex<float> nv (new_node.first-nearest_node.first, new_node.second-nearest_node.second); // norm of vector
    float min_cost = cost_map[nearest_node] + Distance::euclidean(new_node, nearest_node);//sqrt(std::norm(nv));
    cell best_node = nearest_node; //.first, nearest_node.second};
    for(auto nb: neighbors){
        //std::complex<float> nv2 (new_node.first-n.first, new_node.second-n.second); // norm of vector
        float cost = cost_map[nb] + Distance::euclidean(new_node, nb);//sqrt(std::norm(nv2));
        if(cost < min_cost && Sampling::is_collision_free(new_node, nb, tree)){
            best_node = nb; //.first, nb.second};
            min_cost = cost;
        }
    }
    cost_map[new_node] = min_cost;
    parent[new_node] = best_node;
    return new_node;
}

void RRTStar::rewire(cell new_node, vector<cell> neighbors){
    for(auto nb: neighbors){
        //std::complex<float> nv (n.first-new_node.first, n.second-new_node.second); // norm of vector
        float cost = cost_map[new_node] + Distance::euclidean(new_node,nb);//sqrt(std::norm(nv));  
        if(cost < cost_map[nb] && Sampling::is_collision_free(new_node, nb, tree)){
            cost_map[nb] = cost;
            parent[nb] = new_node;
        }
    }
}

void RRTStar::set_step_size(float size){
    step_size = (size >= AlgoConstants::STEP_SIZE_LOWER_LIMIT) ? size : AlgoConstants::STEP_SIZE_LOWER_LIMIT;
    search_radius = 2*step_size;
    goal_radius = (search_radius >= 2) ? (3/4.0)*search_radius : 0.85*search_radius;
}

vector<cell> RRTStar::get_travelled_nodes(){
    return travelled;
}

vector<cell> RRTStar::get_travelled_tree(){
    set<cell> tree_set;
    for(auto tn : get_travelled_nodes()){
        auto parent_node = parent[tn];
        auto line = Bresenham::connect_points(tn, parent_node);
        tree_set.insert(line.begin(), line.end());
    }
    vector<cell> travelled_tree;
    for(auto it = tree_set.begin(); it != tree_set.end(); ++it)
        travelled_tree.push_back(cell{it->first, it->second});
    return travelled_tree;
}