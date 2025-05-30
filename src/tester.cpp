#include "map_data.hpp"
#include "a_star.hpp"
#include <chrono>
#include <iomanip>

using namespace std::chrono;

void test_map_data(){
    auto map_data = MapData::parse_pgm("./maps/test.pgm");
    map_data.boundaries = MapData::inflate_boundaries(map_data, 3);
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

Map get_map_data(){
    auto map_data = MapData::parse_pgm("./maps/test.pgm");
    //cout << "Map Dim. | W: " << map_data.px_width << ", H: " << map_data.px_height << endl;
    return map_data;
}

Graph get_graph_data(Map &m){
    m.boundaries = MapData::inflate_boundaries(m, 3);
    auto g = MapData::get_graph_from_map(m);
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

void test_a_star_simple(Graph g){
    std::cout << "A-STAR (Simple)" << std::endl;

    g.root = {1, 1};
    g.end = {4, 5};
    auto as = AStar(g);

    auto start_time = high_resolution_clock::now();
    as.solve(g.root, g.end);
    auto end_time = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(end_time- start_time);
    std::cout << "Elapsed Time: " << duration.count() << " ms\n";

    auto results = as.reconstruct_path(g.root, g.end);
    vector<pair<int, int>> path = results.first;
    float dist = results.second;
    std::cout << "Path: [";
    for(auto p: path){
        std::cout << "(" << p.first << "," << p.second << "), ";
    }
    std::cout << "]\n";
    std::cout << "Distance: " << dist << std::endl;
}

void test_a_star(Map &m, Graph g){
    std::cout << "A-STAR" << std::endl;
    g.root = {100, 50};
    g.end = {381, 360};
    auto as = AStar(g);
    
    auto start_time = high_resolution_clock::now();
    auto s_t = std::chrono::system_clock::to_time_t(start_time);
    cout << "Start Time: " << std::put_time(std::localtime(&s_t), "%F %T\n") << std::flush; 
    as.solve(g.root, g.end);
    auto end_time = high_resolution_clock::now();
    auto e_t = std::chrono::system_clock::to_time_t(end_time);
    cout << "End Time: " << std::put_time(std::localtime(&e_t), "%F %T\n") << std::flush;
    auto duration = duration_cast<milliseconds>(end_time- start_time);
    std::cout << "Elapsed Time: " << duration.count() << " ms\n";
    
    auto results = as.reconstruct_path(g.root, g.end);
    vector<pair<int, int>> path = results.first;
    float dist = results.second;
    cout << "# of Nodes: " << path.size() << endl;
    /*std::cout << "Path: [";
    for(auto p: path){
        std::cout << "(" << p.first << "," << p.second << "), ";
    }
    std::cout << "]\n";*/
    std::cout << "Distance: " << dist << std::endl;
    //Map nm = MapData::add_path_to_map(m, path);
    //MapData::print_boundary(nm.boundaries, nm.px_width, nm.px_height);
}

int main(){
    // Test Data retrieval from PGM file
    /*test_map_data();*/

    // Test A-star algorithm (Simple Data)
    //auto g = simple_data();
    //test_a_star_simple(g);

    // Test A-Star algorithm (using Map)
    auto m = get_map_data();
    auto g = get_graph_data(m);
    test_a_star(m, g);
}