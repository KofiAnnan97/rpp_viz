#ifndef MAP_DATA_HPP
#define MAP_DATA_HPP

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <map>
#include <vector>

using namespace std;

struct Map{
    int px_width, px_height;
    float resolution, m_width, m_height;
    int** boundaries;
};

// Change this class to use unordered_map (requires default constructor)
class Graph {
    public:    
        // Pair := (y, x)
        map<pair<int, int>, vector<pair<pair<int, int>, int>>> g;
        pair<int,int> root = {0, 0};
        pair<int,int> end = {0, 0};

        vector<pair<pair<int, int>, int>> get_edges(pair<int, int> parent){
            try{
                return g[parent];
            }
            catch(const std::out_of_range& oor){
                return vector<pair<pair<int, int>, int>>();
            }
        }

        void add_node(pair<int, int> node){
            g[node] = vector<pair<pair<int, int>, int>>();
        }

        void add_edge(pair<int, int> parent, pair<int, int> child, int weight){
            g[parent].push_back({child, weight});
        }
};

class MapData {
    public:
        static Map parse_pgm(string fp);
        static int** copy_boundaries(Map m);
        static int** inflate_boundaries(Map map, int buffer_size);
        static Map add_path_to_map(Map m, vector<pair<int, int>> path);
        static Graph get_graph_from_map(Map map);
        static void print_boundary(int** b, int width, int height);

        //Conversions
        static pair<int, int> POSE2PIXEL(Map map, float x, float y);
        static pair<float, float> PIXEL2POSE(Map map, pair<int, int> px);

    private:
        static void inflate_pixel(int** nb, int width, int height, int j, int i, int buffer_size);
};

#endif // MAP_DATA_HPP