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
};

#endif // PATH_PLANNING_CONSTANTS_HPP