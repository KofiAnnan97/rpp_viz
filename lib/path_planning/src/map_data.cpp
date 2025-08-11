#include "map_data.hpp"

Map MapData::parse_pgm(string mp){
    Map data;
    fstream map_input;
    map_input.open(mp, ios::in);
    if(map_input.is_open()){
        string line, val;
        int width, height, highest_val;
        getline(map_input, line);
        string type = line;
        getline(map_input, line);
        if(line[0] == '#') getline(map_input, line);
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
            //cout << "{" << i << ", " << j << "}: "<< (int)px << " => " << val << endl;
            temp[i][j] = val > 0 ? OPEN_SPACE_INT : OBSTACLE_INT;
            j++;
            if(j >= width){
                i++;
                j=0;
            }
            if(i >= height) break;
        }
        data.px_height = height;
        data.px_width = width;
        data.boundaries = temp;
    }
    return data;
}

Map MapData::get_map(string yp){
    fstream yaml_file;
    yaml_file.open(yp, ios::in);
    if(yaml_file.is_open()){
        char yaml_delim = ':';
        string line, word, image, mode;
        float resolution, negate, occupied_thresh, free_thresh;
        vector<float> origin; 
        while(getline(yaml_file, line)){
            if(line[0] == '#') getline(yaml_file, line);
            stringstream ss(line);
            getline(ss, word, yaml_delim);
            if(word == "image"){
                getline(ss, word, yaml_delim);
                image = word[0] == ' ' ? word.substr(1, word.size()-1) : word;
            }
            else if(word == "mode"){
                getline(ss, word, yaml_delim);
                mode = word[0] == ' ' ? word.substr(1, word.size()-1) : word;
            }
            else if(word == "resolution"){
                getline(ss, word, yaml_delim);
                resolution = stof(word);
            }
            else if(word == "origin"){
                getline(ss, word, yaml_delim);
                string origin_str = word[0] == ' ' ? word.substr(2, word.size()-2) : word.substr(1, word.size()-2);
                stringstream oss(origin_str);
                string val;
                while(getline(oss, val, ',')) origin.push_back(stof(val));
            }
            else if(word == "negate"){
                getline(ss, word, yaml_delim);
                negate = stof(word);
            }
            else if(word == "occupied_thresh"){
                getline(ss, word, yaml_delim);
                occupied_thresh = stof(word);
            }
            else if(word == "free_thresh"){
                getline(ss, word, yaml_delim);
                free_thresh = stof(word);
            }
            else cout << "\'" << word << "\' is not an accepted keyword." << endl;
        }
        /*cout << "Image: " << image << endl;
        cout << "Mode: " << mode << endl;
        cout << "Resoultion: " << resolution << endl;
        cout << "Origin: [";
        for(auto o: origin) cout << o << ",";
        cout << "]\n";
        cout << "Negate: " << negate << endl;
        cout << "Occupied Threshold: " << occupied_thresh << endl;
        cout << "Free Threshold: " << free_thresh << endl;*/
        std::filesystem::path yaml_path = yp;
        auto image_path = yaml_path.parent_path() / image;
        string mp = image_path.string();
        Map map = MapData::parse_pgm(mp);    
        map.m_width = origin[0] > 0 ? origin[0]*2 : origin[0]*-2;
        map.m_height = origin[1] > 0 ? origin[1]*2 : origin[1]*-2;
        map.resolution = resolution;
        return map;
    }
    else{
        cout << "Could not find file: " << yp << endl;
        Map temp;
        return temp;
    }
}

int** MapData::copy_boundaries(Map map){
    int** new_boundaries = new int*[map.px_height];
    for(int k = 0; k < map.px_height; k++) new_boundaries[k] = new int[map.px_width];
    for(int row = 0; row < map.px_height; row++){
        for(int col = 0; col < map.px_width; col++){
            new_boundaries[row][col] = map.boundaries[row][col];
        }
    }
    return new_boundaries;
}

Map MapData::copy_map(Map map){
    Map new_map;
    new_map.px_width = map.px_width;
    new_map.px_height = map.px_height;
    new_map.m_width = map.m_width;
    new_map.m_height = map.m_height;
    new_map.boundaries = MapData::copy_boundaries(map);
    return new_map;
}

void MapData::inflate_pixel(int** nb, int width, int height, int col, int row, int buffer_size){
    int dx = buffer_size/2;
    int dy = buffer_size/2;
    if(dx == 0 && dy == 0) nb[row][col] = OBSTACLE_INT;
    else{
        for(int y = row-dy; y <= row+dy; y++){
            for(int x = col-dx; x <= col+dx; x++){
                if(y >= 0 && y < height && x >= 0 && x < width && nb[y][x] != OBSTACLE_INT){
                    nb[y][x] = INFLATE_INT;
                }
            }
        }  
    }    
}

int** MapData::inflate_boundaries(Map map, int buffer_size){
    // Allocate new boundary
    int** new_boundaries = MapData::copy_boundaries(map);
     
    // Expand boundary based on buffer size
    for(int row = 0; row <= map.px_height-1; row++){
        for(int col = 0; col < map.px_width; col++){
            if(map.boundaries[row][col] != OPEN_SPACE_INT){ //-1){
                MapData::inflate_pixel(new_boundaries, map.px_width, map.px_height, col, row, buffer_size);
            }
        }
    }

    //De-allocate old boundaries
    for(int l = 0; l < map.px_height; l++) delete map.boundaries[l];
    delete[] map.boundaries;
    // return inflated boundary
    return new_boundaries;
}

int** MapData::remove_boundary_inflation(Map map){
    // Allocate new boundary
    int** original_boundaries = MapData::copy_boundaries(map);

    // Remove inflation from boundaries
    for(int row = 0; row <= map.px_height-1; row++){
        for(int col = 0; col < map.px_width; col++){
            if(map.boundaries[row][col] == INFLATE_INT){
                original_boundaries[row][col] = OPEN_SPACE_INT;
            }
        }
    }

    //De-allocate old boundaries
    for(int l = 0; l < map.px_height; l++) delete map.boundaries[l];
    delete[] map.boundaries;
    // return inflated boundary
    return original_boundaries;
}

void MapData::inflate_point(Map map, cell pt, int buffer_size){
    int dx = buffer_size/2;
    int dy = buffer_size/2;
    int** b = map.boundaries;
    int cell_val = map.boundaries[pt.second][pt.first];
    for(int y = pt.second-dy; y <= pt.second+dy; y++){
        for(int x = pt.first-dx; x <= pt.first+dx; x++){
            if(y >= 0 && y < map.px_height && x >= 0 && x < map.px_width){
                if(cell_val == -1 && b[y][x] != OBSTACLE_INT) b[y][x] = INFLATE_INT; //&& pt != cell{x,y}
                else b[y][x] = cell_val;
            }
        }
    }
}

Map MapData::add_path_to_map(Map map, vector<cell> path, cell sp, cell ep){
    int path_val = 3;
    Map new_map =  MapData::add_path_to_map_with_value(map, path_val, path, sp, ep);
    return new_map;
}

Map MapData::add_path_to_map_with_value(Map map, int pixel_val, vector<cell> path, cell sp, cell ep){
    Map new_map;
    new_map.px_height = map.px_height;
    new_map.px_width = map.px_width;
    new_map.m_height = map.m_height;
    new_map.m_width = map.m_width;
    new_map.resolution = map.resolution;
    new_map.boundaries = MapData::copy_boundaries(map);
    for(auto p: path){
        new_map.boundaries[p.second][p.first] = pixel_val;
        MapData::inflate_point(new_map, p, PATH_SIZE);
    }
    new_map.boundaries[sp.second][sp.first] = NAV_POINT_INT;
    new_map.boundaries[ep.second][ep.first] = NAV_POINT_INT;
    MapData::inflate_point(new_map, sp, POINT_SIZE);
    MapData::inflate_point(new_map, ep, POINT_SIZE);
    return new_map;
}

Map MapData::debug_map(Map map, vector<cell> path, vector<cell> travelled, cell sp, cell ep){
    Map new_map;
    new_map.px_height = map.px_height;
    new_map.px_width = map.px_width;
    new_map.m_height = map.m_height;
    new_map.m_width = map.m_width;
    new_map.resolution = map.resolution;
    new_map.boundaries = MapData::copy_boundaries(map);
    for(auto t: travelled) new_map.boundaries[t.second][t.first] = TRAVELLED_INT;
    for(auto p: path) {
        new_map.boundaries[p.second][p.first] = PATH_INT;
        MapData::inflate_point(new_map, p, PATH_SIZE);
    }
    new_map.boundaries[sp.second][sp.first] = NAV_POINT_INT;
    new_map.boundaries[ep.second][ep.first] = NAV_POINT_INT;
    MapData::inflate_point(new_map, sp, POINT_SIZE);
    MapData::inflate_point(new_map, ep, POINT_SIZE);
    return new_map;
}

Graph MapData::get_graph_from_map(Map map){
    Graph graph;
    for(int row = 0; row < map.px_height; row++){
        for(int col = 0; col < map.px_width; col++){
            if(map.boundaries[row][col] == 0){
                auto curr = cell{col, row};
                // Up
                if(row > 0 && map.boundaries[row-1][col] == 0){
                    graph.add_edge(curr, cell{col, row-1}, 1);  
                }
                // Down
                if (row < map.px_height - 1 && map.boundaries[row+1][col] == 0){
                    graph.add_edge(curr, cell{col, row+1,}, 1);
                }
                // Left 
                if(col > 0 && map.boundaries[row][col-1] == 0){
                    graph.add_edge(curr, cell{col-1, row}, 1);
                }
                // Right
                if(col < map.px_width - 1 && map.boundaries[row][col+1] == 0){
                    graph.add_edge(curr, cell{col+1, row}, 1);
                }
                // Up-Left
                if(row > 0 && col > 0 && map.boundaries[row-1][col-1] == 0){
                    graph.add_edge(curr, cell{col-1, row-1,}, 2);
                }
                // Up-Right
                if(row > 0 && col < map.px_width -1 && map.boundaries[row-1][col+1] == 0){
                    graph.add_edge(curr, cell{col+1, row-1}, 2);
                }
                // Down-Left
                if(row < map.px_height - 1 && col > 0 && map.boundaries[row+1][col-1] == 0){
                    graph.add_edge(curr, cell{col-1, row+1, }, 2);
                }
                // Down-Right
                if(row < map.px_height - 1 && col < map.px_width -1 && map.boundaries[row+1][col+1] == 0){
                    graph.add_edge(curr, cell{col+1, row+1}, 2);
                }
                // If cell has no neihbors add it to the graph
                if(!graph.is_node_valid(curr)) graph.add_node(curr);
            }
        }
    }
    return graph;
}

void MapData::print_boundary(int** b, int width, int height){
    cout << "[\n";
    for(int row = 0; row < height; row++){
        cout << row << ": [";
        for(int col = 0; col < width; col++){
            cout << b[row][col] << ",";
        }
        cout << "]\n";
    }
    cout << "]\n";
}

void MapData::show_map(string title, Map map){
    Mat img(map.px_height, map.px_width, CV_8UC3);
    for(int row = 0; row < map.px_height; row++){
        for(int col = 0; col < map.px_width; col++){
            if(map.boundaries[row][col] == PATH_INT) img.at<Vec3b>(Point(col,row)) = cv::Vec3b(0,0,255);            // Path color
            else if(map.boundaries[row][col] == TRAVELLED_INT) img.at<Vec3b>(Point(col,row)) = cv::Vec3b(230,216,173);   // Visted node color
            else if(map.boundaries[row][col] == NAV_POINT_INT) img.at<Vec3b>(Point(col,row)) = cv::Vec3b(128,0,128);     // Start and goal node
            else if(map.boundaries[row][col] == OPEN_SPACE_INT) img.at<Vec3b>(Point(col,row)) = cv::Vec3b(255,255,255);   // Empty space color
            else if(map.boundaries[row][col] <= OBSTACLE_INT) img.at<Vec3b>(Point(col,row)) = cv::Vec3b(0,0,0);          // Obstacle color
        }
    }
    cout << "Showing image: " << title;
    cout << " (press \'q\' to quit)" << endl;
    imshow(title, img);
    if(waitKey(0) && 0xFF == 'q') destroyAllWindows();
}

// Rotates map 90 degrees clockwise
cell MapData::POSE2PIXEL(Map map, float x, float y){
    cell px;
    float x_pt_res = map.m_width*(1.0/map.px_width);
    float y_pt_res = map.m_height*(1.0/map.px_height);
    px.first = (y/-x_pt_res) + map.px_width/2;
    px.second = (x/y_pt_res) + map.px_height/2;
    return px;
}
 
// Rotates map 90 degrees counter-clockwise
pair<float, float> MapData::PIXEL2POSE(Map map, cell px){
    pair<float, float> pose;
    float x_pt_res = map.m_width*(1.0/map.px_width);
    float y_pt_res = map.m_height*(1.0/map.px_height);
    pose.first = x_pt_res*(px.second - map.px_width/2);
    pose.second = -y_pt_res*(px.first - map.px_height/2);
    return pose;
}
