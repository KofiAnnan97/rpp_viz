#include <chrono>
#include <iomanip>
#include <filesystem>
#include <cstdio>
#include <gtest/gtest.h>

#include "map_data.hpp" 
#include "bfs.hpp"
#include "a_star.hpp"
//#include "d_star_lite.hpp"
#include "rrt_star.hpp"
#include "gen_ros_map.hpp"
#include "time_helper.hpp"

using namespace std::chrono;
namespace fs = std::filesystem;

namespace  testing {
    const int COMPUTE_TIMEOUT = 60000; // in milliseconds
    const float pose_err_thresh = 0.3;
    const int px_err_thresh = 5;
    
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

    string CELL2STR(cell pt){
        return "(" + std::to_string(pt.first) + "," + std::to_string(pt.second) + ")";
    }

    string CELLF2STR(std::pair<float,float> pt){
        return "(" + std::to_string(pt.first) + "," + std::to_string(pt.second) + ")";
    }

    bool is_node_valid(Graph g, cell node){
        return g.is_node_valid(node);
    }

    float calc_distance(cell expected, cell actual){
        return sqrt(pow(expected.first - actual.first, 2) + pow(expected.second - actual.second, 2));
    }

    float path_rmse_error(vector<cell> expected, vector<cell> actual){
        float err = 0.0;
        if(expected.empty() || actual.empty()) return 100.0;
        int diff_err = actual.size() - expected.size();

        int n = (expected.size() <= actual.size()) ? expected.size() : actual.size(); 
        for(int i = 0; i < n; i++)
            err += pow(calc_distance(expected[i], actual[i]), 2);
        err/=n;
        return sqrt(err) + abs(diff_err);
    }

    /* Data Extraction 
        Check that map data is correctly converted to obstacle maps
        Make sure the yaml and pgm have the correct data
    */ 
    class Data_Extraction : public testing::Test{
        public: 
            string pgm_path, yaml_path;

        protected:
            fs::path tmp_path = "temp";
            string title = "test";
            string pgm_file = title+".pgm";
            string yaml_file = title+".yaml";

            void SetUp() override {
                Map original = get_simple_map();
                fs::create_directory(tmp_path);
                pgm_path = tmp_path / pgm_file;
                yaml_path = tmp_path / yaml_file;
            
                // Generate map files
                GenerateMap::generate_map_pgm(original, tmp_path, title);
                GenerateMap::generate_map_yaml(original, tmp_path, title);
            }

            void TearDown() override {
                fs::remove_all(tmp_path);
            }
    };

    testing::AssertionResult checkPGM(string pgm_path){
        fstream pgm;
        pgm.open(pgm_path, ios::in | ios::binary);
        if(pgm.is_open()){
            string line, word;
            bool vals_correct = true;
            int expected_height = 10;
            int expected_width = 20;
            int expected_highest_value = 255;
            string err_msg = "Error: ";
            getline(pgm, line);
            if(strcmp("P5",line.c_str()) != 0){
                vals_correct = false;
                err_msg += "P5 ,";
            }
            getline(pgm, line);
            if(strcmp("# CREATOR: gen_ros_map.cpp 0.05 m/pix",line.c_str()) != 0){
                if(vals_correct) vals_correct = false;
                err_msg += " comment is incorrect, ";
            }
            getline(pgm, line);
            stringstream ss(line);
            getline(ss, word, ' ');
            if(stoi(word) != expected_width){
                if(vals_correct) vals_correct = false;
                auto sub_err = " width should be " + std::to_string(expected_width) + " but is " + word + ", ";
                err_msg += sub_err;
            }
            getline(ss, word, ' ');
            if(stoi(word) != expected_height){
                if(vals_correct) vals_correct = false;
                auto sub_err = " height should be " + std::to_string(expected_height) + " but is " + word + ", ";
                err_msg += sub_err;
            }
            getline(pgm, line);
            if(stoi(line) != expected_highest_value){
                if(vals_correct) vals_correct = false;
                auto sub_err = " highest value should be " + std::to_string(expected_highest_value) + " but is " + line + ", ";
                err_msg += sub_err;
            }
            if(vals_correct) return testing::AssertionSuccess();
            else return testing::AssertionFailure() << "failed, " << err_msg;
        }
        else return testing::AssertionFailure() << "failed, Could not open " << pgm_path;
    }

    testing::AssertionResult checkYAML(string yaml_path, string title){
        fstream yaml;
        yaml.open(yaml_path, ios::in);
        if(yaml.is_open()){
            string line, word;
            bool vals_correct = true;
            string expected_image = " "+title+".pgm";
            float expected_resolution = 0.05;
            string expected_origin = " [-0.56, -0.28, 0]";
            int expected_negate = 0;
            float expected_occupied_thresh =  0.65;
            float expected_free_thresh = 0.25;
            string err_msg = "Errors: ";
            for(int i=0; i<6; i++){
                getline(yaml, line);
                stringstream ss(line);
                getline(ss, word, ':');
                string id = word;
                getline(ss, word, ':');
                if(strcmp("image", id.c_str()) == 0 && strcmp(expected_image.c_str(), word.c_str()) != 0){
                    if(vals_correct) vals_correct = false;
                    auto sub_err = " image name should be " + expected_image + ", ";
                    err_msg += sub_err;
                }
                else if(strcmp("resolution", id.c_str()) == 0 && expected_resolution != stof(word)){
                    if(vals_correct) vals_correct = false;
                    auto sub_err = " resolution should be " + std::to_string(expected_resolution) + " not " + word + ", ";
                    err_msg += sub_err;
                }
                else if(strcmp("origin", id.c_str()) == 0 && strcmp(expected_origin.c_str(), word.c_str()) != 0){
                    if(vals_correct) vals_correct = false;
                    auto sub_err = " resolution should be " + expected_origin + " not " + word + ", ";
                    err_msg += sub_err;
                }
                else if(strcmp("negate", id.c_str()) == 0 && expected_negate != stoi(word)){
                    if(vals_correct) vals_correct = false;
                    auto sub_err = " negate should be " + std::to_string(expected_negate) + " not " + word + ", ";
                    err_msg += sub_err;
                }
                else if(strcmp("occupied_thresh", id.c_str()) == 0 && expected_occupied_thresh != stof(word)){
                    if(vals_correct) vals_correct = false;
                    auto sub_err = " occupied threshold should be " + std::to_string(expected_occupied_thresh) + " not " + word + ", ";
                    err_msg += sub_err;
                }
                else if(strcmp("free_thresh", id.c_str()) == 0 && expected_free_thresh != stof(word)){
                    if(vals_correct) vals_correct = false;
                    auto sub_err = " free threshold should be " + std::to_string(expected_free_thresh) + " not " + word + ", ";
                    err_msg += sub_err;
                }
            }
            if(vals_correct) return testing::AssertionSuccess();
            else return testing::AssertionFailure() << "failed, " << err_msg; 
        }
        else return testing::AssertionFailure() << "failed, Could not open " << yaml_path;
    }

    testing::AssertionResult checkMapObstacles(string yaml_path){
        Map extracted_map = MapData::get_map(yaml_path);
        try{
            bool boundaries_match = true;
            Map original = testing::get_simple_map();
            if(extracted_map.px_height != original.px_height ||
            extracted_map.px_width != original.px_width){
                return testing::AssertionFailure()<< "Dimension for the original and extracted map are not the same.";
            }
            else{
                for(int row=0; row<extracted_map.px_height; row++){
                    for(int col=0; col<extracted_map.px_width; col++){
                        if(original.boundaries[row][col] != extracted_map.boundaries[row][col]){
                            cout << " failed\n";
                            boundaries_match = false;
                            break;
                        }
                    }
                    if(!boundaries_match) break;
                }
            }
            if(boundaries_match) return testing::AssertionSuccess();
            else return testing::AssertionFailure() << "One or multiples did not match the expected output";
        }
        catch(std::exception e){
            return testing::AssertionFailure() << e.what();
        }
    }

    TEST_F(Data_Extraction, pgm_correctness){
        EXPECT_TRUE(checkPGM(pgm_path));
    }

    TEST_F(Data_Extraction, yaml_correctness){
        EXPECT_TRUE(checkYAML(yaml_path, title));
    }

    TEST_F(Data_Extraction, obstacle_correctness){
        EXPECT_TRUE(checkMapObstacles(yaml_path));
    }

    /* Map Data
        Check that inflate boundary occurres correctly
        Check that copy boundaries works correctly
        Check graph generated correctly
    */

    vector<cell> get_expected_free_cells(){
        return {
            {19,0}, 
            {2,2}, {3,2}, {4,2}, {8,2}, {9,2}, {10,2}, {11,2}, {12,2}, {13,2}, {14,2}, {15,2}, 
            {2,3}, {3,3}, {4,3}, {8,3}, {12,3}, {13,3}, {14,3}, {15,3}, 
            {2,4}, {8,4}, {12,4}, {16,4}, {17,4},
            {2,5}, {8,5}, {12,5}, {16,5}, {17,5},
            {2,6}, {7,6}, {16,6}, {17,6},
            {2,7}, {3,7}, {4,7}, {5,7}, {6,7}, {7,7}, {16,7}, {17,7}
        };
    }
    
    map<cell, vector<cell>> get_expected_graph(){
        return {
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
    }

    testing::AssertionResult checkInflation(Map map){
        map.boundaries = MapData::inflate_boundaries(map, 3);
        vector<cell> expected_free_cells = get_expected_free_cells();
        vector<cell> incorrect_free_cells;
        for(int i = 0; i < map.px_height; i++){
            for(int j = 0; j < map.px_width; j++){
                int val = map.boundaries[i][j];
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
        if(expected_free_cells.empty() && incorrect_free_cells.empty())
            return testing::AssertionSuccess();
        else{
            string err_msg = "";
            if(expected_free_cells.size() > 1){
                err_msg += "\n\tMising free cells: [ ";
                for(auto m: expected_free_cells) err_msg += CELL2STR(m) + " ";
                err_msg += "]";
            }
            if(incorrect_free_cells.size() > 1){
                if(expected_free_cells.size() == 0) err_msg += "Incorrect free cells: [ ";
                else err_msg += "\n\tIncorrect free cells: [ ";
                for(auto e: incorrect_free_cells) err_msg += CELL2STR(e) + " ";
                err_msg += " ]";
            }
            return testing::AssertionFailure() << err_msg;
        }
    }

    testing::AssertionResult checkObstaclesCopy(Map map){
        vector<cell> incorrect_pixels;
        int** copied_boundaries = MapData::copy_boundaries(map);
        for(int r = 0; r < map.px_height; r++){
            for(int c = 0; c < map.px_width; c++){
                if(map.boundaries[r][c] != copied_boundaries[r][c]){
                    incorrect_pixels.push_back(cell{c,r});
                }
            }
        }
        if(incorrect_pixels.size() > 0){
            string err_msg = "Incorrect pixels: { ";
            for(auto pixel: incorrect_pixels) err_msg += CELL2STR(pixel) + " ";
            err_msg += "}";
            return testing::AssertionFailure() << err_msg;
        }
        else return testing::AssertionSuccess();
    }

    testing::AssertionResult checkGraph(Map map_data){
        map_data.boundaries = MapData::inflate_boundaries(map_data, 3);
        vector<pair<cell,cell>> missing_edges;
        vector<pair<cell,cell>> incorrect_edges;
        vector<cell> missing_nodes;
        vector<cell> incorrect_nodes;
        Graph graph = MapData::get_graph_from_map(map_data);
        map<cell, vector<cell>> expected_tree = get_expected_graph();
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
        if(missing_nodes.empty() && incorrect_nodes.empty() && missing_edges.empty() && incorrect_edges.empty())
            return testing::AssertionSuccess();
        else{
            string err_msg = "";
            if(!missing_nodes.empty()){
                err_msg += "\n\tMissing nodes: [ ";
                for(auto mn: missing_nodes) err_msg += CELL2STR(mn) + " ";
                err_msg += "]";
            }
            if(!missing_edges.empty()){
                err_msg += "\n\tNodes with missing edges: [ ";
                for(auto me: missing_edges) err_msg += CELL2STR(me.first) + " => " + CELL2STR(me.second) + ", ";
                err_msg += "]";
            }
            if(!incorrect_nodes.empty()){
                err_msg += "\n\tIncorrect nodes: [ ";
                for(auto in: incorrect_nodes) err_msg += CELL2STR(in) + " ";
                err_msg += "]";
            }
            if(!incorrect_edges.empty()){
                err_msg += "\n\tNodes with incorrect edges: [ ";
                for(auto ie: incorrect_edges) err_msg += CELL2STR(ie.first) + " => " + CELL2STR(ie.second) + ", ";
                err_msg += "]";
            }
            return testing::AssertionFailure() << err_msg;
        }
    }

    TEST(Map_Data, inflate_obstacles){
        Map map = testing::get_simple_map();
        EXPECT_TRUE(checkInflation(map));
    }

    TEST(Map_Data, copy_obstacles){
        Map map = testing::get_simple_map();
        EXPECT_TRUE(checkObstaclesCopy(map));
    }

    TEST(Map_Data, check_graph_creation){
        Map map = testing::get_simple_map();
        EXPECT_TRUE(checkGraph(map));
    }

    TEST(Map_Data, valid_point){
        auto m = testing::get_simple_map();
        auto graph = MapData::get_graph_from_map(m);
        cell test_pt = (graph.g.size() > 0) ? graph.g.begin()->first : cell{-1,-1};
        EXPECT_TRUE(testing::is_node_valid(graph, test_pt));
    }

    TEST(Map_Data, invalid_point){
        auto m = testing::get_simple_map();
        auto g = MapData::get_graph_from_map(m);
        EXPECT_FALSE(testing::is_node_valid(g, {0,0}));
    }

    /* Conversions (Both ways)
        Center Pixel 
        Top Left Pixel
        Top Right Pixel
        Bottom Left Pixel
        Bottom Right Pixel
    */

    testing::AssertionResult checkPixelToPose(Map map, cell actual_pixel, cell expected_pose){
        auto converted_pose = MapData::PIXEL2POSE(map, {actual_pixel.first, actual_pixel.second});
        if(abs(expected_pose.first-converted_pose.first)<=pose_err_thresh 
            && abs(expected_pose.second-converted_pose.second)<=pose_err_thresh)
            return testing::AssertionSuccess();
        else
            return testing::AssertionFailure() << "Expected: " << CELLF2STR(expected_pose) << " not " << CELLF2STR(converted_pose);
    }

    testing::AssertionResult checkPoseToPixel(Map map, cell actual_pose, cell expected_pixel){
        auto converted_pixel = MapData::POSE2PIXEL(map, actual_pose.first, actual_pose.second);
        if(abs(expected_pixel.first-converted_pixel.first)<=px_err_thresh 
            && abs(expected_pixel.second-converted_pixel.second)<=px_err_thresh)
            return testing::AssertionSuccess();
        else
            return testing::AssertionFailure() << "Expected: " << CELL2STR(expected_pixel) << " not " << CELL2STR(converted_pixel);
    }
    
    TEST(Map_Pose_Conversion, center_pixel_to_pose){
        auto m = testing::get_simple_map();
        cell actual_pixel = {m.px_width/2,m.px_height/2};
        std::pair<float,float> expected_pose = {0,0};
        EXPECT_TRUE(checkPixelToPose(m, actual_pixel, expected_pose));
    }

    TEST(Map_Pose_Conversion, center_pose_to_pixel){
        auto m = testing::get_simple_map();
        cell expected_pixel = {m.px_width/2,m.px_height/2};
        std::pair<float,float> actual_pose = {0,0};
        EXPECT_TRUE(checkPoseToPixel(m, actual_pose, expected_pixel));
    }

    TEST(Map_Pose_Conversion, top_left_pixel_to_pose){
        auto m = testing::get_simple_map();
        cell actual_pixel = {0,0};
        std::pair<float,float> expected_pose = {m.m_width/2,-m.m_height/2};
        EXPECT_TRUE(checkPixelToPose(m, actual_pixel, expected_pose));
    }

    TEST(Map_Pose_Conversion, top_left_pose_to_pixel){
        auto m = testing::get_simple_map();
        cell expected_pixel = {0,0};
        std::pair<float,float> actual_pose = {m.m_width/2,-m.m_height/2};
        EXPECT_TRUE(checkPoseToPixel(m, actual_pose, expected_pixel));
    }

    TEST(Map_Pose_Conversion, top_right_pixel_to_pose){
        auto m = testing::get_simple_map();
        cell actual_pixel = {m.px_width-1,0};
        std::pair<float,float> expected_pose = {m.m_width/2,m.m_height/2};
        EXPECT_TRUE(checkPixelToPose(m, actual_pixel, expected_pose));
    }

    TEST(Map_Pose_Conversion, top_right_pose_to_pixel){
        auto m = testing::get_simple_map();
        cell expected_pixel = {m.px_width-1,0};
        std::pair<float,float> actual_pose = {m.m_width/2,m.m_height/2};
        EXPECT_TRUE(checkPoseToPixel(m, actual_pose, expected_pixel));
    }

    TEST(Map_Pose_Conversion, bottom_left_pixel_to_pose){
        auto m = testing::get_simple_map();
        cell actual_pixel = {0,m.px_height-1};
        std::pair<float,float> expected_pose = {-m.m_width/2,m.m_height/2};
        EXPECT_TRUE(checkPixelToPose(m, actual_pixel, expected_pose));
    }

    TEST(Map_Pose_Conversion, bottom_left_pose_to_pixel){
        auto m = testing::get_simple_map();
        cell expected_pixel = {0,m.px_height-1};
        std::pair<float,float> actual_pose = {-m.m_width/2,m.m_height/2};
        EXPECT_TRUE(checkPoseToPixel(m, actual_pose, expected_pixel));
    }

    TEST(Map_Pose_Conversion, bottom_right_pixel_to_pose){
        auto m = testing::get_simple_map();
        cell actual_pixel = {m.px_width-1,m.px_height-1};
        std::pair<float,float> expected_pose = {-m.m_width/2,-m.m_height/2};
        EXPECT_TRUE(checkPixelToPose(m, actual_pixel, expected_pose));
    }

    TEST(Map_Pose_Conversion, bottom_right_pose_to_pixel){
        auto m = testing::get_simple_map();
        cell expected_pixel = {m.px_width-1,m.px_height-1};
        std::pair<float,float> actual_pose = {-m.m_width/2,-m.m_height/2};
        EXPECT_TRUE(checkPoseToPixel(m, actual_pose, expected_pixel));
    }

    // For Algorithm Testing
    
    testing::AssertionResult checkPath(vector<cell> path, vector<cell> expected_path, float path_err_thresh){
        if(path.empty()) return AssertionFailure() << "No path generated.";
        float rmse_err = path_rmse_error(expected_path, path);
        if(rmse_err <= path_err_thresh) return AssertionSuccess();
        else return AssertionFailure() << "failed, RMSE for path is " << rmse_err;
    }

    testing::AssertionResult checkDistance(float distance, float dist_limit){
        if(distance <= dist_limit) return testing::AssertionSuccess();
        else return testing::AssertionFailure() << distance << " > " << dist_limit;
    }

    testing::AssertionResult checkSpeed(int duration, int duration_limit){
        if(duration <= duration_limit) return testing::AssertionSuccess();
        else return testing::AssertionFailure() << duration<< " ms > " << duration_limit << " ms";
    }
    
    /* BFS (Using Simple Data)
        Path Generated between start and goal
        Duration is <= 10 ms
        Check distance
    */
    class BFS_Tests: public testing::Test {
        public:
            float dist;
            int duration;
            vector<cell> path;

        protected:
            Map m = testing::get_simple_map();
            Graph g = MapData::get_graph_from_map(m);
            const int DURATION_LIMIT = 10;
            const float PATH_ERR_THRESH = 0.24;
            const float DIST_LIMIT = 25;

            void SetUp() override {
                g.root = {3, 3};
                g.end = {16, 7};
                BFS bfs = BFS(g);
                auto start_time = TimeHelper::get_time("Start Time", false);
                bfs.solve(g.root, g.end, COMPUTE_TIMEOUT);
                auto end_time = TimeHelper::get_time("End Time", false);
                duration = duration_cast<milliseconds>(end_time- start_time).count();

                auto results = bfs.reconstruct_path(g.root, g.end);
                path = results.first;
                dist = results.second;
            }   
    };

    TEST_F(BFS_Tests, path_generated){
        vector<cell> expected_path = {{3,3}, {4,3}, {5,4}, {6,5}, {7,5}, {8,5}, 
                                      {9,4}, {10,3}, {11,3}, {12,3}, {13,3}, 
                                      {14,4}, {15,5}, {15,6}, {16,7}};
        EXPECT_TRUE(checkPath(path, expected_path, PATH_ERR_THRESH));
    }

    TEST_F(BFS_Tests, check_distance){
        EXPECT_TRUE(checkDistance(dist, DIST_LIMIT));
    }
        
    TEST_F(BFS_Tests, speed_test){
        EXPECT_TRUE(checkSpeed(duration, DURATION_LIMIT));
    }

    /* A* (Using Simple Data)
        Algorithm Completes
        Path Generated between start and goal
        Duration is <= 10 ms
    */
    class A_Star_Tests: public testing::Test {
        public:
            float dist;
            int duration;
            vector<cell> path;

        protected:
            Map m = testing::get_simple_map();
            Graph g = MapData::get_graph_from_map(m);
            const int DURATION_LIMIT = 10;
            const float PATH_ERR_THRESH = 0.24;
            const float DIST_LIMIT = 25;

            void SetUp() override {
                g.root = {3, 3};
                g.end = {16, 7};
                AStar as = AStar(g);
                auto start_time = TimeHelper::get_time("Start Time", false);
                as.solve(g.root, g.end, COMPUTE_TIMEOUT);
                auto end_time = TimeHelper::get_time("End Time", false);
                duration = duration_cast<milliseconds>(end_time- start_time).count();

                auto results = as.reconstruct_path(g.root, g.end);
                path = results.first;
                dist = results.second;
            }   
    };

    TEST_F(A_Star_Tests, path_generated){
        vector<cell> expected_path = {{3,3}, {4,3}, {5,4}, {6,5}, {7,5}, {8,5}, 
                                      {9,4}, {10,3}, {11,3}, {12,3}, {13,3}, 
                                      {14,4}, {15,5}, {15,6}, {16,7}};
        EXPECT_TRUE(checkPath(path, expected_path, PATH_ERR_THRESH));
    }

    TEST_F(A_Star_Tests, check_distance){
        EXPECT_TRUE(checkDistance(dist, DIST_LIMIT));
    }
        
    TEST_F(A_Star_Tests, speed_test){
        EXPECT_TRUE(checkSpeed(duration, DURATION_LIMIT));
    }

    /* RRT* (Using Simple Data)
        Algorithm Completes
        Path Generated between start and goal
        Duration is <= 10 ms
        Algorithm fails correctly with limited number of iterations
    */
    class RRT_Star_Tests: public testing::Test {
        public:
            float dist;
            int duration;
            vector<cell> path;

        protected:
            Map m = testing::get_simple_map();
            Graph g = MapData::get_graph_from_map(m);
            const int DURATION_LIMIT = 10;
            const float PATH_ERR_THRESH = 4;
            const float DIST_LIMIT = 25;

            void SetUp() override {
                g.root = {3, 3};
                g.end = {16, 7};
                auto rrt = RRTStar(g, 1000);

                auto start_time = TimeHelper::get_time("Start Time", false);
                rrt.solve(g.root, g.end, COMPUTE_TIMEOUT);
                auto end_time = TimeHelper::get_time("End Time", false);
                auto duration = duration_cast<milliseconds>(end_time- start_time);
                
                dist = std::numeric_limits<float>::infinity();
                if(rrt.goal_reached){
                    auto results = rrt.reconstruct_path(g.root, g.end);
                    path = results.first;
                    dist = results.second;
                }
            }   
    };

    TEST_F(RRT_Star_Tests, path_generated){
        vector<cell> expected_path = {{3,3}, {4,4}, {5,4}, {6,5}, {7,4}, {8,3}, 
                                      {9,3}, {10,3}, {11,3}, {12,3}, {13,2}, {14,2},
                                      {15,3}, {16,4}, {16,5}, {16,6}, {16,7}};
        EXPECT_TRUE(checkPath(path, expected_path, PATH_ERR_THRESH));
    }

    TEST_F(RRT_Star_Tests, check_distance){
        EXPECT_TRUE(checkDistance(dist, DIST_LIMIT));
    }
        
    TEST_F(RRT_Star_Tests, speed_test){
        EXPECT_TRUE(checkSpeed(duration, DURATION_LIMIT));
    }

    TEST_F(RRT_Star_Tests, max_iteration_too_smal){
        auto short_rrt = RRTStar(g, 10);
        short_rrt.solve(g.root, g.end, COMPUTE_TIMEOUT);
        auto invalid_result = short_rrt.reconstruct_path(g.root, g.end);
        auto inv_path = invalid_result.first;
        float inv_dist = invalid_result.second;
        EXPECT_TRUE(inv_path.size() == 1 && inv_dist == 0);
        /*Potential Error Message:
        string err_msg = "";
        if(inv_dist != 0) err_msg += "Distance should be 0 not " + std::to_string(inv_dist);
        if(inv_path.size() > 1){
            err_msg += "\nPath should only include the goal position but has the following: [";
            for(auto ip: inv_path) err_msg +=  CELL2STR(ip) + " ";
            err_msg += "]";
        }*/ 
    }

    /* D* Lite (Using Simple Data)
        Algorithm Completes
        Path Generated between start and goal
        Duration is <=  10 ms
    */
    /*class D_Star_Lite_Tests: public testing::Test {
        public:
            float dist;
            int duration;
            vector<cell> path;

        protected:
            Map m = testing::get_simple_map();
            Graph g = MapData::get_graph_from_map(m);
            const int DURATION_LIMIT = 10;
            const float PATH_ERR_THRESH = 2.5;
            const float DIST_LIMIT = 25;

            void SetUp() override {
                g.root = {3, 3};
                g.end = {16, 7};
                auto ds = DStarLite(g);

                auto start_time = get_time("Start Time"); 
                ds.solve(g.root, g.end);
                auto end_time = get_time("End Time"); 
                auto duration = duration_cast<milliseconds>(end_time- start_time);

                auto results = ds.reconstruct_path(g.root, g.end);
                vector<cell> path = results.first;
                float dist = results.second;
            }   
    };

    TEST_F(D_Star_Lite_Tests, path_generated){
        vector<cell> expected_path = {{3,3}, {4,3}, {5,4}, {6,5}, {7,5}, {8,5}, 
                                      {9,4}, {10,3}, {11,3}, {12,3}, {13,3}, 
                                      {14,4}, {15,5}, {15,6}, {16,7}};
        EXPECT_TRUE(checkPath(path, expected_path, PATH_ERR_THRESH));
    }

    TEST_F(D_Star_Lite_Tests, check_distance){
        EXPECT_TRUE(checkDistance(dist, DIST_LIMIT));
    }
        
    TEST_F(D_Star_Lite_Tests, speed_test){
        EXPECT_TRUE(checkSpeed(duration, DURATION_LIMIT));
    }*/
}

int main(int argc, char** argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}