#ifndef APP_CONSTANTS_H
#define APP_CONSTANTS_H

#include <string>

#include <QString>

#include "pp_constants.hpp"

using namespace std;

class AppConstants {
    public:
        // General
        static const int COLOR_PATH_IDX = 3;

        // Icons
        inline static const QString DRAW_ICON = ":/icons/pencil.svg";
        inline static const QString ERASE_ICON = ":/icons/eraser.svg";
        inline static const QString DRAW_CURSOR = ":/icons/cursor_pencil.svg";
        inline static const QString ERASE_CURSOR_SMALL = ":/icons/cursor_eraser_small.svg";
        inline static const QString ERASE_CURSOR = ":/icons/cursor_eraser.svg";

        // Algorithms
        inline static const QString BFS_ID = "BFS";
        inline static const QString A_STAR_ID = "A*";
        inline static const QString RRT_STAR_ID = "RRT*";
        inline static const QString ALL_ID = "All";
    
        // Map Constants
        inline static const string OBSTACLE_MAP_ID = "obstacle_map";
        inline static const string DISPLAY_MAP_ID = "display_map";
        inline static const string PATH_MAP_ID = "path_map";
};

#endif // APP_CONSTANTS_H