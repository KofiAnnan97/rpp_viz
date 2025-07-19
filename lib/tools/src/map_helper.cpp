#include "map_helper.hpp"

cell MapHelper::get_positon(string pos_str){
    cell pos;
    try{
        string val;
        char delim = ',';
        stringstream pos_ss(pos_str);
        getline(pos_ss, val, delim);
        pos.first = std::stoi(val);
        getline(pos_ss, val, delim);
        pos.second = std::stoi(val);
    } catch(std::invalid_argument e){
        pos = {-1,-1};
    }
    return pos;
}

void MapHelper::add_result(vector<AlgoResult> &results, string algo_type, int duration,
                           vector<cell> path, vector<cell> travelled, float dist){
    results.push_back(AlgoResult{algo_type, duration, path, travelled, dist});
}
