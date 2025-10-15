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
};

#endif // SCRIPT_CONSTANTS_HPP