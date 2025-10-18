#ifndef SCRIPT_CONSTANTS_HPP
#define SCRIPT_CONSTANTS_HPP

#include <string>

#include "pp_constants.hpp"

using namespace std;

class CLIConstants {
    public:
        // Algorithm
        inline static const string BFS_ID = "bfs";
        inline static const string A_STAR_ID = "a-star";
        inline static const string RRT_STAR_ID = "rrt-star";
        inline static const string PRM_ID = "prm";
        inline static const string ALL_ID = "all";
};

class MapGenConstants{
    public:
        // Pixel values
        static const int HIGHEST_PX_VALUE = 255;
        static const int UNKNOWN_PX_VALUE = 205;
        static const int OBSTACLE_PX_VALUE = 0;

        // Defaults
        static const int DEFAULT_NEGATE = 0;
        inline static const float DEFAULT_OCCUPIED_THRESH = 0.65;
        inline static const float DEFAULT_FREE_THRESH = 0.25;
};

#endif // SCRIPT_CONSTANTS_HPP