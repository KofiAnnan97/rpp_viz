#ifndef MAP_DATA_HPP
#define MAP_DATA_HPP

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <map>
#include <vector>
#include <filesystem>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;

typedef pair<int,int> cell;     // coordingate from map (x,y) := (col,row)
typedef pair<cell,int> iw_cell; // cell with integer weight

struct Map{
    int px_width, px_height;
    float resolution, m_width, m_height; // in meters
    int** boundaries;
};

// Change this class to use unordered_map (requires default constructor)
class Graph {
    public:    
        // Pair := (y, x)
        map<cell, vector<iw_cell>> g;
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

        vector<cell> get_nodes(){
            vector<cell> nodes;
            for(auto node: g) nodes.push_back(node.first);
            return nodes;
        }

        void add_node(cell node){
            g[node] = vector<iw_cell>();
        }

        void add_edge(cell parent, cell child, int weight){
            g[parent].push_back({child, weight});
        }

        int get_size(){
            return g.size();
        }
};

class MapData {
    public:
        static Map get_map(string yp);
        static int** copy_boundaries(Map m);
        static int** inflate_boundaries(Map map, int buffer_size);
        static Map add_path_to_map(Map m, vector<cell> path);
        static Map debug_map(Map m, vector<cell> path, vector<cell> travelled, cell sp, cell ep);
        static Graph get_graph_from_map(Map map);
        static void print_boundary(int** b, int width, int height);
        static void show_map(string title, Map map);

        //Conversions
        static cell POSE2PIXEL(Map map, float x, float y);
        static pair<float, float> PIXEL2POSE(Map map, cell px);

    private:
        static Map parse_pgm(string fp);
        static void inflate_pixel(int** nb, int width, int height, int j, int i, int buffer_size);
};

#endif // MAP_DATA_HPP