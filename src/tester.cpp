#include "map_data.hpp"

void test_map_data(){
    auto map_data = MapData::parse_pgm("./maps/test.pgm");
    map_data.boundaries = MapData::inflate_boundary(map_data, 3);
    MapData::print_boundary(map_data.boundaries, map_data.px_width, map_data.px_height);

    auto g = MapData::get_graph_from_map(map_data);
    int test_x = 360;
    int test_y = 387; 
    auto edge_list = g.get_edges(pair<int, int>{test_y, test_x});
    cout << "Given (" << test_y << ", " << test_x << "):" << endl; 
    for(auto e: edge_list){
        std::cout << "(" << e.first << ", " << e.second << ") ";
    }
    std::cout << "\n";
}

int main(){
    test_map_data();
}