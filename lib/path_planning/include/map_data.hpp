#ifndef MAP_DATA_HPP
#define MAP_DATA_HPP

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <unordered_map>
#include <vector>
#include <filesystem>
#include <exception>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "pp_constants.hpp"

using namespace std;
using namespace cv;

typedef pair<int,int> cell;     // coordingate from map (x,y) := (col,row)
typedef pair<cell,int> iw_cell; // cell with integer weight

template <>
struct std::hash<cell> {
  size_t operator()(const cell& c) const {
    string cell_str = "(" + to_string(c.first) + "," + to_string(c.second) + ")";
    return std::hash<std::string>{}(cell_str);
  }
};

struct Map{
    int px_width, px_height;
    float resolution, m_width, m_height; // in meters
    int** boundaries;
};

// Change this class to use unordered_map (requires default constructor)
class Graph {
    public:    
        unordered_map<cell, vector<iw_cell>> g;           // Valid nodes
        vector<cell> obs;                       // Obstacle nodes 
        cell root = {0, 0};
        cell end = {0, 0};

        vector<pair<cell, int>> get_edges(cell parent){
            try{
                return g[parent];
            }
            catch(const std::out_of_range& oor){
                return vector<iw_cell>();
            }
        }

        vector<cell> get_edges_without_weights(cell parent){
            try{
                vector<cell> temp; 
                for(auto child: g[parent]){
                    temp.push_back(child.first);
                }
                return temp;
            }
            catch(const std::out_of_range& oor){
                return vector<cell>();
            }
        }

        bool is_node_valid(cell node){
            //if(g[node].size() > 0) return true;
            if(g.find(node) != g.end()) return true;
            else return false;
        }

        vector<cell> get_valid_nodes(){
            vector<cell> nodes;
            for(auto node: g) nodes.push_back(node.first);
            return nodes;
        }

        vector<cell> get_obstacle_nodes(){
            return obs;
        }

        void add_node(cell node){
            g[node] = vector<iw_cell>();
        }

        void add_edge(cell parent, cell child, int weight){
            g[parent].push_back({child, weight});
        }
        
        void add_obstacle_node(cell node){
            if (g.find(node)!=g.end()) obs.push_back(node);
        }

        int get_size(){
            return g.size();
        }
};

class MapData {
    public:
        static Map get_map(string yp);
        static int** set_boundaries(int width, int height, vector<signed char> data);
        static int** copy_boundaries(Map m);
        static Map copy_map(Map map);
        static int** inflate_boundaries(Map map, int buffer_size);
        static int** remove_boundary_inflation(Map map);
        static void inflate_point(Map map, cell pt, int inflate_size);
        static Map add_path_to_map(Map m, vector<cell> path, cell sp, cell ep);
        static Map add_path_to_map_with_value(Map map, int pixel_val, vector<cell> path, cell sp, cell ep);
        static Map debug_map(Map m, vector<cell> path, vector<cell> travelled, cell sp, cell ep);
        static Graph get_graph_from_map(Map map);
        static void print_boundary(int** b, int width, int height);
        static void show_map(string title, Map map);

        // Map-Robot Conversions
        static cell POSE2PIXEL(Map map, float x, float y);
        static pair<float, float> PIXEL2POSE(Map map, cell px);

    private:
        static Map parse_pgm(string fp);
        static void inflate_pixel(int** nb, int width, int height, int j, int i, int buffer_size);
};

#endif // MAP_DATA_HPP
