#ifndef GEOMETRY_UTILS_HPP
#define GEOMETRY_UTILS_HPP

#include "map_data.hpp"

class ConsoleOutput{
    public:
        static void print_cells(string title, vector<cell> cells);      
};

class Bresenham{
    private:
        static vector<cell> low_line(int x1, int y1, int x2, int y2);
        static vector<cell> high_line(int x1, int y1, int x2, int y2);
    public:
        static vector<cell> connect_points(cell a, cell b);
};

class Distance{
    public:
        static float manhattan(cell a, cell b);
        static float euclidean(cell a, cell b);
};

class Sampling{
    public:
        static bool is_collision_free(cell c, cell d, Graph &tree);
        static cell get_random_node(cell ep, vector<cell> &all_valid_nodes);
        static vector<cell> get_connected_path(vector<cell> &path);
};

#endif // GEOMETRY_UTILS_HPP