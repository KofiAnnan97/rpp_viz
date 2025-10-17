#ifndef SCRIPT_CONSTANTS_HPP
#define SCRIPT_CONSTANTS_HPP

#include <string>

#include "pp_constants.hpp"

using namespace std;

class ScriptConstants {
    public:
        // Algorithm
        inline static const string BFS_ID = "bfs";
        inline static const string A_STAR_ID = "a-star";
        inline static const string RRT_STAR_ID = "rrt-star";
        inline static const string PRM_ID = "prm";
        inline static const string ALL_ID = "all";

        // Defaults
        static const int DEFAULT_INFLATE_SIZE = 3;
        static const int DEFAULT_SAMPLE_COUNT = 10000;
        static const int DEFAULT_STEP_SIZE = 20;
        static const int DEFAULT_NEIGHBOR_COUNT = 8;
        static const int DEFAULT_COMPUTE_TIMEOUT = 600000; //in milliseconds
};

#endif // SCRIPT_CONSTANTS_HPP