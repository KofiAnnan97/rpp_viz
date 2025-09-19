#include <gtest/gtest.h>

#include "map_data.hpp" 
#include "common.hpp"

/* 
Map Data
    Check that inflate boundary occurres correctly
    Check that copy boundaries works correctly
    Check graph generated correctly
*/

using namespace testing;

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

AssertionResult checkInflation(Map map){
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
        return AssertionSuccess();
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
        return AssertionFailure() << err_msg;
    }
}

AssertionResult checkObstaclesCopy(Map map){
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
        return AssertionFailure() << err_msg;
    }
    else return AssertionSuccess();
}

AssertionResult checkGraph(Map map_data){
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
        return AssertionSuccess();
    else{
        string err_msg = "";
        if(!missing_nodes.empty()){
            err_msg += "\n\tMissing nodes: [ ";
            for(auto mn: missing_nodes) err_msg += CELL2STR(mn) + " ";
            err_msg += "]";
        }
        if(!missing_edges.empty()){
            err_msg += "\n\tNodes with missing edges: [ ";
            for(auto me: missing_edges) 
                err_msg += CELL2STR(me.first) + " => " + CELL2STR(me.second) + ", ";
            err_msg += "]";
        }
        if(!incorrect_nodes.empty()){
            err_msg += "\n\tIncorrect nodes: [ ";
            for(auto in: incorrect_nodes) err_msg += CELL2STR(in) + " ";
            err_msg += "]";
        }
        if(!incorrect_edges.empty()){
            err_msg += "\n\tNodes with incorrect edges: [ ";
            for(auto ie: incorrect_edges) 
                err_msg += CELL2STR(ie.first) + " => " + CELL2STR(ie.second) + ", ";
            err_msg += "]";
        }
        return AssertionFailure() << err_msg;
    }
}

TEST(Map_Data, inflate_obstacles){
    Map map = get_simple_map();
    EXPECT_TRUE(checkInflation(map));
}

TEST(Map_Data, copy_obstacles){
    Map map = get_simple_map();
    EXPECT_TRUE(checkObstaclesCopy(map));
}

TEST(Map_Data, check_graph_creation){
    Map map = get_simple_map();
    EXPECT_TRUE(checkGraph(map));
}

TEST(Map_Data, valid_point){
    auto m = get_simple_map();
    auto graph = MapData::get_graph_from_map(m);
    cell test_pt = (graph.g.size() > 0) ? graph.g.begin()->first : cell{-1,-1};
    EXPECT_TRUE(is_node_valid(graph, test_pt));
}

TEST(Map_Data, invalid_point){
    auto m = get_simple_map();
    auto g = MapData::get_graph_from_map(m);
    EXPECT_FALSE(is_node_valid(g, {0,0}));
}