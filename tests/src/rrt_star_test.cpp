#include <chrono>
#include <gtest/gtest.h>

#include "rrt_star.hpp"
#include "time_helper.hpp"
#include "common.hpp"

/* 
RRT* (Using Simple Data)
    Algorithm Completes
    Path Generated between start and goal
    Duration is <= 10 ms
    Algorithm fails correctly with limited number of iterations
*/

using namespace testing;

class RRT_Star_Tests: public Test {
    public:
        float dist;
        int duration;
        vector<cell> path;

    protected:
        Map m = get_simple_map();
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
    } */
}