#include "a_star.hpp"

AStar::AStar(Graph g){
    tree = g;

    for(auto it = tree.g.begin(); it != tree.g.end(); ++it){
        dist[it->first] = std::numeric_limits<int>::max();
        h[it->first] = get_distance(it->first, tree.end);
        f[it->first] = 0;
        q.push(it->first);
        //std::cout << "(" << it->first.first << ", " << it->first.second << "): " << f[it->first] << std::endl;
    }
    //print_map("H:", h);
}

pair<vector<pair<int,int>>, float> AStar::reconstruct_path(){
    auto data = pair<vector<pair<int,int>>, float>();
    if(tree.root != tree.end) data.first.push_back(tree.end);
    auto curr = tree.end;
    while(curr != tree.root){
        curr = via[curr];
        //if(curr == pair<int, int>{0, 0}) break;
        data.first.insert(data.first.begin(), curr);
    }
    data.second = dist[tree.end];
    return data;
}

void AStar::solve(pair<int, int> sp, pair<int, int> ep){
    //cout << "solve" << endl;
    dist[sp] = 0;
    f[sp] = AStar::get_f_score(sp);
    /*int n = q.size();
    for (int i = 0; i < n; i++){
            auto node = q.front();
            std::cout << "(" << node.first << "," << node.second << "){"<<f[node]<<"} ";
            q.pop();
            q.push(node);
    }
    std::cout << "]\n";*/
    while(!q.empty()){
        //AStar::sort_queue(q);
        /*int n = q.size();
        std::cout << "Queue(" << q.size() << "): [ ";
        for (int i = 0; i < n; i++){
            auto node = q.front();
            std::cout << "(" << node.first << "," << node.second << "){"<<f[node]<<"} ";
            q.pop();
            q.push(node);
        }
        std::cout << "]\n";*/
        //print_map("F", f);
        pair<int, int> curr = q.front();
        q.pop();
        if(curr == tree.end) break;
        //print_map("Distances:", dist);
        auto children = tree.get_edges(curr);
        /*cout << "(" << curr.first << ", " << curr.second << ") => Children: "; 
        for(auto c: children){
            std::cout << "(" << c.first.first << ", " << c.first.second << ") ";
        }
        std::cout << "\n";*/
        
        for(auto c : children){
            auto cp = c.first;
            auto w = c.second;
            auto new_dist = dist[curr] + w;
            auto new_cost = new_dist + h[cp];
            if(new_cost < AStar::get_f_score(cp)){
                f[cp] = new_cost;
                dist[cp] = new_dist;
                via[cp] = curr;
            } 
        }
    }
    /*for(auto it = via.cbegin(); it != via.cend(); ++it){
        std::cout << "(" << it->second.first << ", " << it->second.second << ") => ("<< it->first.first << ", " << it->first.second << ")\n";
    }*/
}
int AStar::get_max_idx_by_f_score(queue<pair<int, int>> &q, int s_idx){
    int max_idx = 0; // This being at -1 causes new pairs to be added to queue
    int max_f_score = std::numeric_limits<int>::min();
    int n = q.size();
    for (int i=0; i < n; i++){
        auto node = q.front();
        //cout << node.first << ","<< node.second << endl;
        q.pop();  
        int f_score = get_f_score(node);
        //cout << "F score: " << f_score << endl;
        if (f_score >= max_f_score && i <= s_idx){
            max_idx = i;
            max_f_score = f_score;
        }
        q.push(node);
    }
    return max_idx;
}

void AStar::insert_max_to_rear(queue<pair<int, int>> &q, int max_idx){
    pair<int, int> max_node;
    int n = q.size();
    for (int i = 0; i < n; i++){
        auto node = q.front();
        q.pop();
        if (i != max_idx) q.push(node);
        else max_node = node;
    }
    q.push(max_node);
}

void AStar::sort_queue(queue<pair<int, int>> &q){
    for(int i = 0; i <= q.size()-1; i++){
        int max_idx = AStar::get_max_idx_by_f_score(q, q.size()-i-1);
        //cout << "Max: " << max_idx << endl;
        AStar::insert_max_to_rear(q, max_idx);
    }
}

float AStar::get_f_score(pair<int, int> p){
    return dist[p] + h[p];
}

float AStar::get_distance(pair<int, int> a, pair<int, int> b){
    return sqrt(pow(a.first - b.first, 2) + pow(a.second - b.second, 2));
}

void AStar::print_map(string name, map<pair<int, int>, float> map){
    cout << name << endl;
    for(auto m : map){
        cout << "[" << m.first.first << "," << m.first.second  << "] = " << m.second << endl;
    }
}