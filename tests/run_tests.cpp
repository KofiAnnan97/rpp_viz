#include <chrono>
#include <iomanip>
#include <gtest/gtest.h>

#include "map_data.hpp" 
#include "a_star.hpp"
#include "bfs.hpp"
#include "rrt_star.hpp"

using namespace std::chrono;

typedef std::chrono::_V2::system_clock::time_point c_time_point;

c_time_point get_time(string time_title){
    auto now = high_resolution_clock::now();
    /*auto n = std::chrono::system_clock::to_time_t(now);
    cout << time_title << ": " << std::put_time(std::localtime(&n), "%F %T\n") << std::flush;*/
    return now;
}

Map get_simple_map(){
    Map map;
    map.px_height = 10;
    map.px_width = 20;
    int temp[10][20] = {
        {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0,0},
        {-1,0,0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,-1,0,0},
        {-1,0,0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,-1,-1,-1},
        {-1,0,0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,0,0,-1},
        {-1,0,0,0,0,0,-1,0,0,0,-1,0,0,0,0,0,0,0,0,-1},
        {-1,0,0,0,-1,-1,0,0,0,0,-1,0,0,0,-1,0,0,0,0,-1},
        {-1,0,0,0,0,0,0,0,0,0,-1,0,0,0,-1,0,0,0,0,-1},
        {-1,0,0,0,0,0,0,0,0,-1,-1,-1,0,0,-1,0,0,0,0,-1},
        {-1,0,0,0,0,0,0,0,0,-1,0,-1,0,0,-1,0,0,0,0,-1},
        {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0,-1,-1,-1,-1,-1,-1,-1,-1,-1},
    };
    map.boundaries = new int*[map.px_height];
    for(int k = 0; k < map.px_height; k++) map.boundaries[k] = new int[map.px_width];
    for(int row = 0; row < map.px_height; row++){
        for(int col = 0; col < map.px_width; col++){
            map.boundaries[row][col] = temp[row][col];
        }
    }
    map.m_height = 0.56;
    map.m_width = 1.12;
    map.resolution = 0.05;
    return map;
}

/*
Map Data
    Check that inflate boundary occurres correctly
    Check that copy boundaries works correctly
    Check graph generated correctly
*/
void test_map_data(){
    Map map_data = get_simple_map();
    int passed_count = 0;
    cout << "MAP FUNCTION TESTS\n";
    
    // Test inflate boundaries function
    map_data.boundaries = MapData::inflate_boundaries(map_data, 3);
    vector<cell> expected_free_cells = {
        {19,0}, 
        {2,2}, {3,2}, {4,2}, {8,2}, {9,2}, {10,2}, {11,2}, {12,2}, {13,2}, {14,2}, {15,2}, 
        {2,3}, {3,3}, {4,3}, {8,3}, {12,3}, {13,3}, {14,3}, {15,3}, 
        {2,4}, {8,4}, {12,4}, {16,4}, {17,4},
        {2,5}, {8,5}, {12,5}, {16,5}, {17,5},
        {2,6}, {7,6}, {16,6}, {17,6},
        {2,7}, {3,7}, {4,7}, {5,7}, {6,7}, {7,7}, {16,7}, {17,7}
    }; 
    vector<cell> incorrect_free_cells;
    for(int i = 0; i < map_data.px_height; i++){
        for(int j = 0; j < map_data.px_width; j++){
            int val = map_data.boundaries[i][j];
            cell pt = {j,i};

            for(int k = 0; k < expected_free_cells.size(); k++){
                if(pt == expected_free_cells[k]){
                    if(val != 0) incorrect_free_cells.push_back(pt);
                    else {
                        expected_free_cells.erase(expected_free_cells.begin()+k);
                        break;
                    }
                }
            }
        }
    }
    cout << "\tInflated Boundaries Test: ";
    if(expected_free_cells.empty() && incorrect_free_cells.empty()){
        cout << "passed\n";
        passed_count++;
    }
    else{
        cout << "failed, \n\t\tMising free cells: [ ";
        for(auto m: expected_free_cells) cout << "(" << m.first << "," << m.second << ") ";
        cout << "]\n\t\tIncorrect free cells: [ ";
        for(auto e: incorrect_free_cells) cout << "(" << e.first << "," << e.second << ") ";
        cout << " ]\n";
    }
    
    // Test copy boundaries function
    vector<cell> incorrect_pixels;
    int** copied_boundaries = MapData::copy_boundaries(map_data);
    for(int r = 0; r < map_data.px_height; r++){
        for(int c = 0; c < map_data.px_width; c++){
            if(map_data.boundaries[r][c] != copied_boundaries[r][c]){
                incorrect_pixels.push_back(cell{c,r});
            }
        }
    }
    cout << "\tCopy Boundaries Test: ";
    if(incorrect_pixels.size() > 0){
        cout << "failed (Incorrect pixels: {";
        for(auto pixel: incorrect_pixels) cout << "(" << pixel.first << "," << pixel.second << "), ";
        cout << "}\n";
    }
    else {
        cout << "passed\n";
        passed_count++;
    }

    // Check get graph function
    vector<pair<cell,cell>> missing_edges;
    vector<pair<cell,cell>> incorrect_edges;
    vector<cell> missing_nodes;
    vector<cell> incorrect_nodes;
    Graph graph = MapData::get_graph_from_map(map_data);
    map<cell, vector<cell>> expected_tree = {
        {{19,0}, {}}, 
        {{2,2},  {{3,2}, {3,3}, {2,3}}}, 
        {{3,2},  {{2,2}, {4,2}, {2,3}, {3,3}, {4,3}}}, 
        {{4,2},  {{3,2}, {3,3}, {4,3}}}, 
        {{8,2},  {{9,2}, {8,3}}}, 
        {{9,2},  {{8,2}, {10,2}, {8,3}}}, 
        {{10,2}, {{9,2}, {11,2}}}, 
        {{11,2}, {{10,2}, {12,2}, {12,3}}}, 
        {{12,2}, {{11,2}, {13,2}, {12,3}, {13,3}}}, 
        {{13,2}, {{12,2}, {14,2}, {12,3}, {13,3}, {14,3}}}, 
        {{14,2}, {{13,2}, {15,2}, {13,3}, {14,3}, {15,3}}}, 
        {{15,2}, {{14,2}, {14,3}, {15,3}}}, 
        {{2,3},  {{2,2}, {3,2}, {3,3}, {2,4}}}, 
        {{3,3},  {{2,2}, {3,2}, {4,2}, {2,3}, {4,3}, {2,4}}}, 
        {{4,3},  {{3,2}, {4,2}, {3,3}}}, 
        {{8,3},  {{8,2}, {9,2}, {8,4}}}, 
        {{12,3}, {{11,2}, {12,2}, {13,2}, {13,3}, {12,4}}}, 
        {{13,3}, {{12,2}, {13,2}, {14,2}, {12,3}, {14,3}, {12,4}}}, 
        {{14,3}, {{13,2}, {14,2}, {15,2}, {13,3}, {15,3}}}, 
        {{15,3}, {{14,2}, {15,2}, {14,3}, {16,4}}}, 
        {{2,4},  {{2,3}, {3,3}, {2,5}}}, 
        {{8,4},  {{8,3}, {8,5}}},
        {{12,4}, {{12,3}, {13,3}, {12,5}}}, 
        {{16,4}, {{15,3}, {17,4}, {16,5}, {17,5}}}, 
        {{17,4}, {{16,4}, {16,5}, {17,5}}},
        {{2,5},  {{2,4}, {2,6}}}, 
        {{8,5},  {{8,4}, {7,6}}}, 
        {{12,5}, {{12,4}}}, 
        {{16,5}, {{16,4}, {17,4}, {17,5}, {16,6}, {17,6}}}, 
        {{17,5}, {{16,4}, {17,4}, {16,5}, {16,6}, {17,6}}},
        {{2,6},  {{2,5}, {2,7}, {3,7}}}, 
        {{7,6},  {{6,7}, {7,7}, {8,5}}}, 
        {{16,6}, {{16,5}, {17,5}, {17,6}, {16,7}, {17,7}}}, 
        {{17,6}, {{16,5}, {17,5}, {16,6}, {16,7}, {17,7}}},
        {{2,7},  {{2,6}, {3,7}}}, 
        {{3,7},  {{2,6}, {2,7}, {4,7}}}, 
        {{4,7},  {{3,7}, {5,7}}}, 
        {{5,7},  {{4,7}, {6,7}}}, 
        {{6,7},  {{5,7}, {7,6}, {7,7}}}, 
        {{7,7},  {{6,7}, {7,6}}}, 
        {{16,7}, {{16,6}, {17,6}, {17,7}}}, 
        {{17,7}, {{16,6}, {17,6}, {16,7}}}
    };
    for(auto actual_vals : graph.g){
        cell a_node = actual_vals.first;
        vector<iw_cell> a_edges = actual_vals.second;
        if(expected_tree.find(a_node) == expected_tree.end()) incorrect_nodes.push_back(a_node);
        else{
            vector<cell> expected_edges = expected_tree[a_node];
            for(int j = 0; j < a_edges.size(); j++){
                auto actual_edge = a_edges[j].first;
                for(int i = 0; i < expected_edges.size(); i++){
                    if(expected_edges[i] == actual_edge) break;
                    else if(expected_edges[i] != actual_edge && i == expected_edges.size()-1){
                       incorrect_edges.push_back(pair{a_node, actual_edge});
                    }
                }  
            }
        } 
    }
    for(auto expected_vals : expected_tree){
        cell e_node = expected_vals.first;
        vector<cell> e_edges = expected_vals.second;
        if(graph.g.find(e_node) == graph.g.end()) missing_nodes.push_back(e_node);
        else{
            auto actual_edges = graph.g[e_node];
            for(int j = 0; j < e_edges.size(); j++){
                auto expected_edge = e_edges[j];
                for(int i = 0; i < actual_edges.size(); i++){
                    if(actual_edges[i].first == expected_edge) break;
                    else if(actual_edges[i].first != expected_edge && i == actual_edges.size()-1){
                        missing_edges.push_back({e_node, expected_edge});
                    }
                }  
            }
        } 
    }
    cout << "\tMap to Graph Test: ";
    if(missing_nodes.empty() && incorrect_nodes.empty() && missing_edges.empty() && incorrect_edges.empty()){
        cout << "passed\n";
        passed_count++;
    }
    else{
        cout << "failed, \n";
        if(!missing_nodes.empty()){
            cout << "\t\tMissing nodes: [ ";
            for(auto mn: missing_nodes) cout << "(" << mn.first << "," << mn.second << ") ";
            cout << "]\n";
        }
        if(!missing_edges.empty()){
            cout << "\t\tNodes with missing edges: [ ";
            for(auto me: missing_edges) cout << "(" << me.first.first << "," << me.first.second << ") => (" << me.second.first << "," << me.second.second << "), ";
            cout << "]\n";
        }
        if(!incorrect_nodes.empty()){
            cout << "\t\tIncorrect nodes: [ ";
            for(auto in: incorrect_nodes) cout << "(" << in.first << "," << in.second << ") ";
            cout << "]\n";
        }
        if(!incorrect_edges.empty()){
            cout << "\t\tNodes with incorrect edges: [ ";
            for(auto ie: incorrect_edges) cout << "(" << ie.first.first << "," << ie.first.second << ") => (" << ie.second.first << "," << ie.second.second << "), ";
            cout << "]\n";
        }
    }
    cout << "Map Tests Passed: " << passed_count << "/3\n";
}

/*
Conversions (Both ways)
    Center Pixel 
    Top Left Pixel
    Top Right Pixel
    Bottom Left Pixel
    Bottom Right Pixel
*/
void test_conversions(){
    auto m = get_simple_map();
    float pose_err_thresh = 0.3;
    int px_err_thresh = 5, total_passed = 0;

    vector<string> title = {"Center", "Top-Left Corner", "Top-Right Corner", "Bottom-Left Corner", "Bottom-Right Corner"};
    vector<cell> expected_pxs = {{m.px_width/2,m.px_height/2}, {0,0}, {m.px_width-1,0}, {0,m.px_height-1}, {m.px_width-1,m.px_height-1}};
    vector<pair<float,float>> expected_poses = {{0,0}, {m.m_width/2,-m.m_height/2}, {m.m_width/2,m.m_height/2}, {-m.m_width/2,m.m_height/2}, {-m.m_width/2,-m.m_height/2}};

    cout << "\nCONVERSION TESTS\n";
    for(int t = 0; t < expected_pxs.size(); t++){
        cell tpx = expected_pxs[t];
        pair<float,float> pose = MapData::PIXEL2POSE(m, {tpx.first, tpx.second});
        cell px = MapData::POSE2PIXEL(m, pose.first, pose.second);

        cout << "\t" << title[t] << " Conversion to Pose Test: ";
        auto eps = expected_poses[t];
        if(abs(eps.first-pose.first)<=pose_err_thresh && abs(eps.second-pose.second)<=pose_err_thresh){
            cout << "passed\n";
            total_passed++;
        }
        else{
            cout << "failed, expected (";
            cout << eps.first << "," << eps.second << ") not ("<< pose.first << "," << pose.second << ")\n"; 
        }
        cout << "\t" << title[t] << " Conversion to Map Test: ";
        if(abs(tpx.first-px.first)<=px_err_thresh && abs(tpx.second-px.second)<=px_err_thresh){
            cout << "passed\n";
            total_passed++;
        }
        else{
            cout << "failed, expected (";
            cout << tpx.first << "," << tpx.second << ") not ("<< px.first << "," << px.second << ")\n"; 
        }
    }
    cout << "Conversion Tests Passed: " << total_passed << "/" << 2*(expected_pxs.size()) << endl;
}

void test_valid_node(Graph g, cell node, int &pass_count){
    bool node_valid = g.is_node_valid(node);
    if(node_valid){
        cout << "passed\n";
        pass_count++;
    } 
    else cout << "failed (" << node.first << "," << node.second << ") is not valid\n";
}

void test_invalid_node(Graph g, cell node, int &pass_count){
    bool node_valid = g.is_node_valid(node);
    if(!node_valid){
        cout << "passed\n";
        pass_count++;
    } 
    else cout << "failed, (" << node.first << "," << node.second << ") should not be valid";
}

float calc_distance(cell expected, cell actual){
    return sqrt(pow(expected.first - actual.first, 2) + pow(expected.second - actual.second, 2));
}

// Calculate root mean squared error (RMSE)
float path_rmse_error(vector<cell> expected, vector<cell> actual){
    float err = 0.0;
    if(expected.empty() || actual.empty()) return 100.0;
    else if(expected.size() != actual.size()) err += abs(actual.size() - expected.size());
    for(int i = 0; i < expected.size(); i++){
        if(i >= actual.size()) break;
        err += pow(calc_distance(expected[i], actual[i]), 2);
    }
    if(actual.size() <= expected.size()) err /= actual.size();
    else err /= expected.size();
    return sqrt(err);
}

/*
BFS (Using Simple Data)
    Path Generated between start and goal
    Duration is less than 2 minutes
    Check distance
*/
void test_bfs_simple(){
    auto m = get_simple_map();
    auto g = MapData::get_graph_from_map(m);
    auto bfs = BFS(g);
    g.root = {3, 3};
    g.end = {16, 7};
    
    auto start_time = get_time("Start Time"); 
    bfs.solve(g.root, g.end);
    auto end_time = get_time("End Time");
    auto duration = duration_cast<milliseconds>(end_time- start_time);

    auto results = bfs.reconstruct_path(g.root, g.end);
    vector<cell> path = results.first;
    float dist = results.second;

    // Test component
    int duration_limit = 10;
    float path_err_thresh = 2.5;
    int passed_count = 0;
    cout << "\nBFS TESTS\n";
    cout << "\tTest Start Point: ";
    test_valid_node(g, g.root, passed_count);
    cout << "\tTest End Point: ";
    test_valid_node(g, g.end, passed_count);
    cout << "\tTest Speed: ";
    if(duration.count() < duration_limit){
        cout << "passed\n";
        passed_count++;
    } 
    else cout << "failed, " << duration.count() << " ms " << duration_limit << " ms\n";
    vector<cell> expected_path = {{3,3}, {4,3}, {5,4}, {6,5}, {7,5}, {8,5}, 
                                  {9,4}, {10,3}, {11,3}, {12,3}, {13,3}, 
                                  {14,4}, {15,5}, {15,6}, {16,7}};
    int dist_limit = 25;
    cout << "\tTest Path: ";
    float rmse_err = path_rmse_error(expected_path, path);
    if(rmse_err <= path_err_thresh){
        cout << "passed\n";
        passed_count++;
    }
    else cout << "failed, RMSE for path is " << rmse_err << endl;
    cout << "\tTest Distance: ";
    if(dist <= dist_limit){
        cout << "passed\n";
        passed_count++;
    }
    else cout << "failed, distance is greater than " << dist_limit << endl;

    cout << "\tTest Invalid Point: ";
    test_invalid_node(g, {0,0}, passed_count);
    cout << "BFS Tests Passed: " << passed_count << "/6\n";
}

/*
A* (Using Simple Data)
    Algorithm Completes
    Path Generated between start and goal
    Duration is less than 2 minutes
*/
void test_a_star_simple(){
    auto m = get_simple_map();
    auto g = MapData::get_graph_from_map(m);

    g.root = {3, 3};
    g.end = {16, 7};
    auto as = AStar(g);

    auto start_time = get_time("Start Time"); 
    as.solve(g.root, g.end);
    auto end_time = get_time("End Time"); 
    auto duration = duration_cast<milliseconds>(end_time- start_time);

    auto results = as.reconstruct_path(g.root, g.end);
    vector<cell> path = results.first;
    float dist = results.second;

    // Test component
    int duration_limit = 10;
    float path_err_thresh = 2.5;
    int passed_count = 0;
    cout << "\nA-STAR TESTS\n";
    cout << "\tTest Start Point: ";
    test_valid_node(g, g.root, passed_count);
    cout << "\tTest End Point: ";
    test_valid_node(g, g.end, passed_count);
    cout << "\tTest Speed: ";
    if(duration.count() < duration_limit){
        cout << "passed\n";
        passed_count++;
    } 
    else cout << "failed, " << duration.count() << " ms " << duration_limit << " ms\n";
    vector<cell> expected_path = {{3,3}, {4,3}, {5,4}, {6,5}, {7,5}, {8,5}, 
                                  {9,4}, {10,3}, {11,3}, {12,3}, {13,3}, 
                                  {14,4}, {15,5}, {15,6}, {16,7}};
    int dist_limit = 25;
    cout << "\tTest Path: ";
    float rmse_err = path_rmse_error(expected_path, path);
    if(rmse_err <= path_err_thresh){
        cout << "passed\n";
        passed_count++;
    }
    else cout << "failed, RMSE for path is " << rmse_err << endl;
    cout << "\tTest Distance: ";
    if(dist <= dist_limit){
        cout << "passed\n";
        passed_count++;
    }
    else cout << "failed, distance is greater than " << dist_limit << endl;

    cout << "\tTest Invalid Point: ";
    test_invalid_node(g, {0,0}, passed_count);
    cout << "A-Star Tests Passed: " << passed_count << "/6\n";
}

/*
RRT* (Using Simple Data)
    Algorithm Completes
    Path Generated between start and goal
    Duration is less than 2 minutes
*/
void test_rrt_star_simple(){
    auto m = get_simple_map();
    auto g = MapData::get_graph_from_map(m);
    g.root = {3, 3};
    g.end = {16, 7}; 
    auto rrt = RRTStar(g, m.px_width, m.px_height, 1000);
    
    auto start_time = get_time("Start Time"); 
    rrt.solve(g.root, g.end);
    auto end_time = get_time("End Time"); 
    auto duration = duration_cast<milliseconds>(end_time- start_time);
    
    vector<cell> path;
    float dist = std::numeric_limits<float>::infinity();
    if(rrt.goal_reached){
        auto results = rrt.reconstruct_path(g.root, g.end);
        path = results.first;
        dist = results.second;
    }

    // Test component
    int duration_limit = 10;
    float path_err_thresh = 2.5;
    int passed_count = 0;
    cout << "\nRRT-STAR TESTS\n";
    cout << "\tTest Start Point: ";
    test_valid_node(g, g.root, passed_count);
    cout << "\tTest End Point: ";
    test_valid_node(g, g.end, passed_count);
    cout << "\tTest Speed: ";
    if(duration.count() < duration_limit){
        cout << "passed\n";
        passed_count++;
    } 
    else cout << "failed, " << duration.count() << " ms " << duration_limit << " ms\n";
    vector<cell> expected_path = {{3,3}, {4,4}, {5,4}, {6,5}, {7,4}, {8,3}, 
                                  {9,3}, {10,3}, {11,3}, {12,3}, {13,2}, {14,2},
                                  {15,3}, {16,4}, {16,5}, {16,6}, {16,7}};
    int dist_limit = 20;
    cout << "\tTest Path: ";
    float rmse_err = path_rmse_error(expected_path, path);
    if(rmse_err <= path_err_thresh){
        cout << "passed\n";
        passed_count++;
    }
    else cout << "failed, RMSE for path is " << rmse_err << endl;
    cout << "\tTest Distance: ";
    if(dist <= dist_limit){
        cout << "passed\n";
        passed_count++;
    }
    else cout << "failed, distance is greater than " << dist_limit << endl;

    cout << "\tTest Invalid Point: ";
    test_invalid_node(g, {0,0}, passed_count);
    auto short_rrt = RRTStar(g, m.px_width, m.px_height, 10);
    short_rrt.solve(g.root, g.end);
    auto invalid_result = short_rrt.reconstruct_path(g.root, g.end);
    cout << "\tTest Limited Iteration Failure: ";
    auto inv_path = invalid_result.first;
    float inv_dist = invalid_result.second;
    if(inv_path.size() == 1 && inv_dist == 0){
        cout << "passed\n";
        passed_count++;
    }
    else{
        cout << "failed, ";
        cout << "distance should be 0 not " << inv_dist;
        cout << "\n Path should only include the goal position but has the following: [";
        for(auto ip: inv_path) cout <<  "(" << ip.first << "," << ip.second << ") ";
        cout << "]\n";
    }
    cout << "RRT-Star Tests Passed: " << passed_count << "/7\n";
}


/*+------------+
  | Unit Tests |
  +------------+*/
int main(){
    test_map_data();
    test_conversions();
    test_bfs_simple();
    test_a_star_simple();
    test_rrt_star_simple();
}