#ifndef MAP_HELPER_HPP
#define MAP_HELPER_HPP

#include <string>

#include "map_data.hpp"

using namespace std;

struct AlgoResult{
    string type;
    int duration;
    vector<cell> path;
    vector<cell> travelled;
    float dist;
};

class MapHelper {
public:
    static cell get_positon(string pos_str);
    static void add_result(vector<AlgoResult> &results, string algo_type, int duration,
                           vector<cell> path, vector<cell> travelled, float dist);
};

#endif // MAP_HELPER_HPP
