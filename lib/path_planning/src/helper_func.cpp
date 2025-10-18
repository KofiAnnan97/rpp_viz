#include "map_data.hpp"

// Bresenham's Line Generation Algorithm
class Bresenham{
    private:
        static vector<cell> low_line(int x1, int y1, int x2, int y2){
            vector<cell> line;
            int dx = x2 - x1;
            int dy = y2 - y1;
            int yi = 1;
            if(dy < 0){
                yi = -1;
                dy = -dy;
            }
            int e = (2*dy) - dx;
            int y = y1;
            for(int x = x1; x <= x2; x++){
                line.push_back(cell{x,y});
                if(e > 0){
                    y += yi;
                    e += (2*(dy - dx));
                }
                else
                    e += 2*dy;
            }
            return line;
        }

        static vector<cell> high_line(int x1, int y1, int x2, int y2){
            vector<cell> line;
            int dx = x2 - x1;
            int dy = y2 - y1;
            int xi = 1;
            if(dx < 0){
                xi = -1;
                dx = -dx;
            }
            int e = (2*dx) - dy;
            int x = x1;
            for(int y = y1; y <= y2; y++){
                line.push_back(cell{x,y});
                if(e > 0){
                    x += xi;
                    e += (2*(dx - dy));
                }
                else
                    e += 2*dx;
            }
            return line;
        }
    public:
        static vector<cell> connect_points(cell a, cell b){
            vector<cell> line;
            if(abs(b.second - a.second) < abs(b.first - a.first)){
                if(a.first > b.first)
                    line = low_line(b.first, b.second, a.first, a.second);
                else
                    line = low_line(a.first, a.second, b.first, b.second);
            }
            else{
                if(a.second > b.second)
                    line = high_line(b.first, b.second, a.first, a.second);
                else
                    line = high_line(a.first, a.second, b.first, b.second);
            }
            return line;
        }
};

// Distance formulas
class Distance{
    public:
        static float manhattan(cell a, cell b){
            return abs(a.first - b.first) + abs(a.second - b.second);
        }
        
        static float euclidean(cell a, cell b){
            return sqrt(pow(a.first - b.first, 2) + pow(a.second - b.second, 2));
        }
};

class Samples{
    public:
        static bool is_collision_free(cell c, cell d, Graph &tree){
            auto line = Bresenham::connect_points(c, d);
            for(auto pt: line){
                if(!tree.is_node_valid(pt)) return false;
            }
            return true;
        }

        static cell get_random_node(cell ep, vector<cell> &all_valid_nodes){
            double r = (double)rand()/(double)RAND_MAX;
            cell random_node;
            if(r > 0.2) {
                int r_idx = rand()%all_valid_nodes.size();
                random_node = {all_valid_nodes[r_idx].first, all_valid_nodes[r_idx].second};
                all_valid_nodes.erase(all_valid_nodes.begin()+r_idx);
                all_valid_nodes.push_back(random_node);
            }
            else random_node = ep;
            return random_node;
        }
};