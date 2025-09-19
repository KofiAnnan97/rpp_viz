#include <chrono>
#include <gtest/gtest.h>

#include "bfs.hpp"
#include "time_helper.hpp"
#include "common.hpp"

/* 
BFS (Using Simple Data)
    Path Generated between start and goal
    Duration is <= 10 ms
    Check distance
*/

using namespace testing;

class BFS_Tests: public Test {
    public:
        float dist;
        int duration;
        vector<cell> path;

    protected:
        Map m = get_simple_map();
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