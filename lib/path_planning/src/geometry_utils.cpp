#include "geometry_utils.hpp"

void ConsoleOutput::print_cells(string title, vector<cell> cells){
    cout << title << ": [ ";
    for(auto c: cells) cout << "(" << c.first << "," << c.second << ") ";
    cout << "]\n";
}

// Bresenham's Line Generation Algorithm
vector<cell> Bresenham::low_line(int x1, int y1, int x2, int y2){
    vector<cell> line;
    int dx = x2 - x1;
    int dy = y2 - y1;
    int yi = 1;
    if(dy < 0){
        yi = -1;
        dy = -dy;
    }
    int e = (2*dy) - dx;
    int y = y1;
    for(int x = x1; x <= x2; x++){
        line.push_back(cell{x,y});
        if(e > 0){
            y += yi;
            e += (2*(dy - dx));
        }
        else
            e += 2*dy;
    }
    return line;
}

vector<cell> Bresenham::high_line(int x1, int y1, int x2, int y2){
    vector<cell> line;
    int dx = x2 - x1;
    int dy = y2 - y1;
    int xi = 1;
    if(dx < 0){
        xi = -1;
        dx = -dx;
    }
    int e = (2*dx) - dy;
    int x = x1;
    for(int y = y1; y <= y2; y++){
        line.push_back(cell{x,y});
        if(e > 0){
            x += xi;
            e += (2*(dx - dy));
        }
        else
            e += 2*dx;
    }
    return line;
}

vector<cell> Bresenham::connect_points(cell a, cell b){
    vector<cell> line;
    if(abs(b.second - a.second) < abs(b.first - a.first)){
        if(a.first > b.first)
            line = low_line(b.first, b.second, a.first, a.second);
        else
            line = low_line(a.first, a.second, b.first, b.second);
    }
    else{
        if(a.second > b.second)
            line = high_line(b.first, b.second, a.first, a.second);
        else
            line = high_line(a.first, a.second, b.first, b.second);
    }
    return line;
}

// Distance formulas
float Distance::manhattan(cell a, cell b){
    return abs(a.first - b.first) + abs(a.second - b.second);
}

float Distance::euclidean(cell a, cell b){
    return sqrt(pow(a.first - b.first, 2) + pow(a.second - b.second, 2));
}

bool Sampling::is_collision_free(cell c, cell d, Graph &tree){
    auto line = Bresenham::connect_points(c, d);
    for(auto& pt: line){
        if(!tree.is_node_valid(pt)) return false;
    }
    return true;
}

cell Sampling::get_random_node(cell ep, vector<cell> &all_valid_nodes){
    double r = (double)rand()/(double)RAND_MAX;
    cell random_node;
    if(r > 0.2) {
        int r_idx = rand()%all_valid_nodes.size();
        random_node = {all_valid_nodes[r_idx].first, all_valid_nodes[r_idx].second};
        all_valid_nodes.erase(all_valid_nodes.begin()+r_idx);
        all_valid_nodes.push_back(random_node);
    }
    else random_node = ep;
        return random_node;
}

vector<cell> Sampling::get_connected_path(vector<cell> &path){
    vector<cell> connected_path;
    //ConsoleOuptut::print_cells("Original Path", path);
    for(int i = 0; i < path.size()-1; i++){
        auto line = Bresenham::connect_points(path[i], path[i+1]);
        if(line.size() == 2)
            connected_path.push_back(path[i]);
        else if(line[0] == path[i])
            connected_path.insert(connected_path.end(),line.begin(), line.end()-1);
        else if(line[0] == path[i+1])
            connected_path.insert(connected_path.end(),line.rbegin(), line.rend()-2);
    }
    connected_path.push_back(path[path.size()-1]);
    return connected_path;
}