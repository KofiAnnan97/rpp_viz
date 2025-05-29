#include "map_data.hpp"

Map MapData::parse_pgm(string fp){
    Map data;
    fstream map_input;
    map_input.open(fp, ios::in);
    if(map_input.is_open()){
        string line, val;
        int width, height, highest_val;
        getline(map_input, line);
        string type = line;
        getline(map_input, line);
        stringstream dim(line);        
        getline(dim, val,' ');
        width = std::stoi(val);
        getline(dim, val,' ');
        height = std::stoi(val);
        getline(map_input, line); 
        highest_val = std::stoi(line);
        getline(map_input, line);
        int** temp = new int*[height];
        for(int k = 0; k < height; k++) temp[k] = new int[width];
        int i = 0;
        int j = 0;
        for(char px : line) {
            int val = (int)px >= 0 ? (int)px : 256+(int)px;
            j++;
            if(j >= width){
                i++;
                j=0;
            }
            if(i >= height) break;
            temp[i][j] = val > 0 ? 0 : -1;
        }
        data.px_height = height;
        data.px_width = width;
        data.boundaries = temp;
    }
    return data;
}

void MapData::inflate_pixel(int** nb, int width, int height, int j, int i, int buffer_size){
    int dx = buffer_size/2;
    int dy = buffer_size/2;
    if(dx == 0 && dy == 0) nb[j][i] = -1;
    else{
        for(int y = i-dy; y < i+dy; y++){
            for(int x = j-dx; x < j+dx; x++){
                if(y >= 0 && y < height && x >= 0 && x < width){
                    nb[y][x] = -1;
                }
            }
        }  
    }    
}

int** MapData::inflate_boundary(Map map, int buffer_size){
    // Allocate new boundary
    int** new_boundaries = new int*[map.px_height];
    for(int k = 0; k < map.px_height; k++) new_boundaries[k] = new int[map.px_width];
     
    // Expand boundary based on buffer size
    for(int row = 0; row <= map.px_height-1; row++){
        for(int col = 0; col < map.px_width; col++){
            if(map.boundaries[row][col] != 0){
                MapData::inflate_pixel(new_boundaries, map.px_width, map.px_height, row, col, buffer_size);
            }
        }
    }
    //print_boundary(new_boundaries, map.px_width, map.px_height);

    //De-allocate old boundaries
    for(int l = 0; l < map.px_height; l++) delete map.boundaries[l];
    delete[] map.boundaries;
    // return inflated boundary
    return new_boundaries;
}

void MapData::print_boundary(int** b, int width, int height){
    cout << "[\n";
    for(int row = 0; row < height; row++){
        cout << "[";
        for(int col = 0; col < width; col++){
            cout << b[row][col] << ",";
        }
        cout << "]\n";
    }
    cout << "]\n";
}

Graph MapData::get_graph_from_map(Map map){
    Graph graph;
    for(int row = 0; row < map.px_height; row++){
        for(int col = 0; col < map.px_width; col++){
            if(map.boundaries[row][col] == 0){
                auto curr = pair<int, int>{row, col};
                // Up
                if(row > 0 && map.boundaries[row-1][col] == 0){
                    graph.add_edge(curr, pair<int, int>{row-1, col}, 1);  
                }
                // Down
                if (row < map.px_height - 1 && map.boundaries[row+1][col] == 0){
                    graph.add_edge(curr, pair<int, int>{row+1, col}, 1);
                }
                // Left 
                if(col > 0 && map.boundaries[row][col-1] == 0){
                    graph.add_edge(curr, pair<int, int>{row, col-1}, 1);
                }
                // Right
                if(col < map.px_width - 1 && map.boundaries[row][col+1] == 0){
                    graph.add_edge(curr, pair<int, int>{row, col+1}, 1);
                }
                // Up-Left
                if(row > 0 && col > 0 && map.boundaries[row-1][col-1] == 0){
                    graph.add_edge(curr, pair<int, int>{row-1, col-1}, 2);
                }
                // Up-Right
                if(row > 0 && col < map.px_width -1 && map.boundaries[row-1][col+1] == 0){
                    graph.add_edge(curr, pair<int, int>{row-1, col+1}, 2);
                }
                // Down-Left
                if(row < map.px_height - 1 && col > 0 && map.boundaries[row+1][col-1] == 0){
                    graph.add_edge(curr, pair<int, int>{row+1, col-1}, 2);
                }
                // Down-Right
                if(row < map.px_height - 1 && col < map.px_width -1 && map.boundaries[row+1][col+1] == 0){
                    graph.add_edge(curr, pair<int, int>{row+1, col+1}, 2);
                }
            }
        }
    }
    return graph;
}

pair<int, int> MapData::POSE2PIXEL(Map map, float x, float y){
    pair<int, int> px;
    int scale_factor = 2;
    float x_pt_res = map.resolution*scale_factor;
    float y_pt_res = map.resolution*scale_factor;
    px.first = (int)((y/-x_pt_res) + map.px_height/2);
    px.second = (int)((x/y_pt_res) + map.px_width/2);
    return px;
}

pair<float, float> MapData::PIXEL2POSE(Map map, pair<int, int> px){
    pair<float, float> pose;
    int scale_factor = 2;
    float x_pt_res = map.resolution*scale_factor;
    float y_pt_res = map.resolution*scale_factor;
    pose.first = x_pt_res*(px.second - map.px_width/2);
    pose.second = y_pt_res*(px.first - map.px_height/2);
    return pose;
}