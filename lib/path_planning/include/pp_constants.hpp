#ifndef PATH_PLANNING_CONSTANTS_HPP
#define PATH_PLANNING_CONSTANTS_HPP

class MapConstants{
    public:
        // Map Variables
        static const int INFLATE_INT = -2;
        static const int OBSTACLE_INT = -1;
        static const int OPEN_SPACE_INT = 0;
        static const int NAV_POINT_INT = 1;
        static const int TRAVELLED_INT = 2;
        static const int PATH_INT = 3;

        // Map Size Variables
        static const int PATH_SIZE = 3;
        static const int POINT_SIZE = 5;

        // Defaults
        static const int DEFAULT_INFLATE_SIZE = 3;
};

class AlgoConstants{
    public:
        // Defaults
        static const int DEFAULT_SAMPLE_COUNT = 10000;
        static const int DEFAULT_NEIGHBOR_COUNT = 6;
        inline static const float DEFAULT_STEP_SIZE = 1.0;
        inline static const float DEFAULT_SEARCH_RADIUS = 2*DEFAULT_STEP_SIZE;
        static const int DEFAULT_COMPUTE_TIMEOUT = 600000;  // in milliseconds (10 minutes)
};

class MathConstants{
    public:
        inline static const float PI = 3.14159;
};

#endif // PATH_PLANNING_CONSTANTS_HPP