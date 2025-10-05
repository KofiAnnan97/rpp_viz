#include <gtest/gtest.h>

#include "map_data.hpp" 

namespace testing{
    // Constants Values
    static const int COMPUTE_TIMEOUT = 60000; // in milliseconds
    static const float pose_err_thresh = 0.3;
    static const int px_err_thresh = 5;
            
    // Generic Functions
    static Map get_simple_map(){
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

    static string CELL2STR(cell pt){
        return "(" + std::to_string(pt.first) + "," + std::to_string(pt.second) + ")";
    }

    static string POSE2STR(std::pair<float,float> pose){
        return "(" + std::to_string(pose.first) + "," + std::to_string(pose.second) + ")";
    }

    static bool is_node_valid(Graph g, cell node){
        return g.is_node_valid(node);
    }

    static float calc_distance(cell expected, cell actual){
        return sqrt(pow(expected.first - actual.first, 2) + pow(expected.second - actual.second, 2));
    }

    static float path_rmse_error(vector<cell> expected, vector<cell> actual){
        float err = 0.0;
        if(expected.empty() || actual.empty()) return 100.0;
        int diff_err = actual.size() - expected.size();

        int n = (expected.size() <= actual.size()) ? expected.size() : actual.size(); 
        for(int i = 0; i < n; i++)
            err += pow(calc_distance(expected[i], actual[i]), 2);
        err/=n;
        return sqrt(err) + abs(diff_err);
    }

    // Algorithm-Specific Functions

    static testing::AssertionResult checkPath(vector<cell> path, vector<cell> expected_path, float path_err_thresh){
        if(path.empty()) return AssertionFailure() << "No path generated.";
        float rmse_err = path_rmse_error(expected_path, path);
        if(rmse_err <= path_err_thresh) return AssertionSuccess();
        else return AssertionFailure() << "failed, RMSE for path is " << rmse_err;
    }

    static testing::AssertionResult checkDistance(float distance, float dist_limit){
        if(distance <= dist_limit) return AssertionSuccess();
        else return AssertionFailure() << distance << " > " << dist_limit;
    }

    static testing::AssertionResult checkSpeed(int duration, int duration_limit){
        if(duration <= duration_limit) return AssertionSuccess();
        else return AssertionFailure() << duration<< " ms > " << duration_limit << " ms";
    }
}