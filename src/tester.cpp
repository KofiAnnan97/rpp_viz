#include "map_data.hpp"
#include "a_star.hpp"
#include <chrono>

using namespace std::chrono;

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
        std::cout << "(" << e.first.first << ", " << e.first.second << ")[Cost: " << e.second << "] ";
    }
    std::cout << "\n";
}

Graph get_map_data(){
    auto map_data = MapData::parse_pgm("./maps/map.pgm");
    map_data.boundaries = MapData::inflate_boundary(map_data, 3);
    auto g = MapData::get_graph_from_map(map_data);
    return g;
}

Graph simple_data(){
    pair<int, int> a = {1, 1};
    pair<int, int> b = {2, 1};
    pair<int, int> c = {2, 3};
    pair<int, int> d = {4, 4};
    pair<int, int> e = {4, 5};
    pair<int, int> f = {1, 2};

    Graph g;
    g.add_edge(a, c, 1);
    g.add_edge(b, c, 1);
    g.add_edge(b, e, 1);
    g.add_edge(c, b, 1);
    g.add_edge(c, e, 3);
    g.add_edge(c, d, 1);
    g.add_edge(e, b, 1);
    g.add_edge(e, c, 3);
    g.add_edge(d, c, 1);
    g.add_node(f);
    return g;
}

void test_a_star(Graph g){
    std::cout << "A-STAR" << std::endl;

    g.root = {100, 50};
    g.end = {300, 360};
    auto as = AStar(g);

    auto start_time = high_resolution_clock::now();
    as.solve(g.root, g.end);
    auto end_time = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(end_time- start_time);
    std::cout << "Elapsed Time: " << duration.count() << " ms\n";

    auto results = as.reconstruct_path();
    vector<pair<int, int>> path = results.first;
    float dist = results.second;
    std::cout << "Path: [";
    for(auto p: path){
        std::cout << "(" << p.first << "," << p.second << "), ";
    }
    std::cout << "]\n";
    std::cout << "Distance: " << dist << std::endl;
}

int main(){
    //test_map_data();
    //auto g = simple_data();
    auto g = get_map_data();
    test_a_star(g);
}