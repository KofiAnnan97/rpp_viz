#include <chrono>
#include <gtest/gtest.h>

#include "probability_roadmap.hpp"
#include "time_helper.hpp"
#include "common.hpp"

/* 
PRM (Using Simple Data)
    Algorithm Completes
    Path Generated between start and goal
    Duration is <= 10 ms
    Algorithm fails correctly with an insufficient number of sample nodes
    Algorithm fails correctly with an insufficient number of neighbors
*/

using namespace testing;

class PRM_Tests: public Test {
    public:
        float dist = std::numeric_limits<float>::infinity();
        int duration;
        vector<cell> path;

    protected:
        Map m = get_simple_map();
        Graph g = MapData::get_graph_from_map(m);
        const int DURATION_LIMIT = 40;
        const float PATH_ERR_THRESH = 20;
        const float DIST_LIMIT = 25;

        void SetUp() override {
            g.root = {3, 3};
            g.end = {16, 7};
            auto prm = PROBABILITY_ROADMAP(g, 100, 6);

            auto start_time = TimeHelper::get_time("Start Time", false);
            prm.solve(g.root, g.end, COMPUTE_TIMEOUT);
            auto end_time = TimeHelper::get_time("End Time", false);
            duration = duration_cast<milliseconds>(end_time- start_time).count();
                
            auto results = prm.reconstruct_path(g.root, g.end);
            path = results.first;
            dist = results.second;
        }   
};

TEST_F(PRM_Tests, path_generated){
    vector<cell> expected_path = {{3,3}, {4,3}, {5,4}, {6,5}, {7,5}, {8,5}, 
                                  {9,4}, {10,3}, {11,3}, {12,3}, {13,3}, 
                                  {14,3}, {15,4}, {15,5}, {15,6}, {16,7}};
    EXPECT_TRUE(checkPath(path, expected_path, PATH_ERR_THRESH));
}

TEST_F(PRM_Tests, check_distance){
    EXPECT_TRUE(checkDistance(dist, DIST_LIMIT));
}
        
TEST_F(PRM_Tests, speed_test){
    EXPECT_TRUE(checkSpeed(duration, DURATION_LIMIT));
}

TEST_F(PRM_Tests, sample_count_too_smal){
    auto short_prm = PROBABILITY_ROADMAP(g, 5, 6);
    short_prm.solve(g.root, g.end, COMPUTE_TIMEOUT);
    auto invalid_result = short_prm.reconstruct_path(g.root, g.end);
    auto inv_path = invalid_result.first;
    float inv_dist = invalid_result.second;
    EXPECT_TRUE(inv_path.size() == 1 && inv_dist == std::numeric_limits<float>::infinity());
}

TEST_F(PRM_Tests, neighbor_count_too_smal){
    auto short_prm = PROBABILITY_ROADMAP(g, 100, 1);
    short_prm.solve(g.root, g.end, COMPUTE_TIMEOUT);
    auto invalid_result = short_prm.reconstruct_path(g.root, g.end);
    auto inv_path = invalid_result.first;
    float inv_dist = invalid_result.second;
    EXPECT_TRUE(inv_path.size() == 1 && inv_dist == std::numeric_limits<float>::infinity());
}