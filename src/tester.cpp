#include <chrono>
#include <iomanip>
#include "a_star.hpp"
#include "bfs.hpp"
#include "rrt_star.hpp"

using namespace std::chrono;

void test_map_data(){
    Map map_data = MapData::get_map("./maps/test.yaml");
    MapData::show_map("Original", map_data);
    auto g = MapData::get_graph_from_map(map_data);
    int test_x = 360;
    int test_y = 387; 
    auto edge_list = g.get_edges(pair<int, int>{test_y, test_x});
    cout << "Given (" << test_y << ", " << test_x << "):" << endl; 
    for(auto e: edge_list){
        std::cout << "(" << e.first.first << ", " << e.first.second << ")[Cost: " << e.second << "] ";
    }
    std::cout << "\n";
    cout << "Height (m): " << map_data.m_height << endl;
    cout << "Width (m): " << map_data.m_width << endl;
}

Graph get_graph_data(Map &m){
    m.boundaries = MapData::inflate_boundaries(m, 5);
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
    Map nm = MapData::add_path_to_map(m, path);
    MapData::show_map("A*", nm);
    //MapData::print_boundary(nm.boundaries, nm.px_width, nm.px_height);
}

void test_bfs(Map &m, Graph g){
    std::cout << "BFS" << std::endl;
    auto bfs = BFS(g);
    
    auto start_time = high_resolution_clock::now();
    auto s_t = std::chrono::system_clock::to_time_t(start_time);
    cout << "Start Time: " << std::put_time(std::localtime(&s_t), "%F %T\n") << std::flush; 
    bfs.solve(g.root, g.end);
    auto end_time = high_resolution_clock::now();
    auto e_t = std::chrono::system_clock::to_time_t(end_time);
    cout << "End Time: " << std::put_time(std::localtime(&e_t), "%F %T\n") << std::flush;
    auto duration = duration_cast<milliseconds>(end_time- start_time);
    std::cout << "Elapsed Time: " << duration.count() << " ms\n";

    auto results = bfs.reconstruct_path(g.root, g.end);
    vector<pair<int, int>> path = results.first;
    float dist = results.second;
    cout << "# of Nodes: " << path.size() << endl;
    /*std::cout << "Path: [";
    for(auto p: path){
        std::cout << "(" << p.first << "," << p.second << "), ";
    }
    std::cout << "]\n";*/
    std::cout << "Distance: " << dist << std::endl;
    Map nm = MapData::add_path_to_map(m, path);
    MapData::show_map("BFS", nm);
    //MapData::print_boundary(nm.boundaries, nm.px_width, nm.px_height);
}

/*void test_rrt_star(Map &m, Graph g){
    cout << "RRT-STAR" << endl;
    auto rrt = RRTStar(g, 8.0, m.px_width, m.px_height, 500000);

    auto start_time = high_resolution_clock::now();
    auto s_t = std::chrono::system_clock::to_time_t(start_time);
    cout << "Start Time: " << std::put_time(std::localtime(&s_t), "%F %T\n") << std::flush;
    rrt.solve(g.root, g.end);
    auto end_time = high_resolution_clock::now();
    auto e_t = std::chrono::system_clock::to_time_t(end_time);
    cout << "End Time: " << std::put_time(std::localtime(&e_t), "%F %T\n") << std::flush;
    auto duration = duration_cast<milliseconds>(end_time- start_time);
    std::cout << "Elapsed Time: " << duration.count() << " ms\n";
    
    if(rrt.goal_reached){
        auto results = rrt.reconstruct_path(g.root, g.end);
        vector<pair<int, int>> path = results.first;
        float dist = results.second;
        cout << "# of Nodes: " << path.size() << endl;
        std::cout << "Path: [";
        for(auto p: path){
            std::cout << "(" << p.first << "," << p.second << "), ";
        }
        std::cout << "]\n";
        std::cout << "Distance: " << dist << std::endl;
        //Map nm = MapData::add_path_to_map(m, path);
        //MapData::print_boundary(nm.boundaries, nm.px_width, nm.px_height);
    }
    else {
        cout << "Goal could not be reached. Please check the following:";
        cout << "\n\tstart point\n\tend point\n\t# of max iterations\n\tstep size\n";
    }
}*/

int main(){
    // Test Data retrieval from PGM file
    //test_map_data();

    // Test A-star algorithm (Simple Data)
    //auto g = simple_data();
    //test_a_star_simple(g);

    // Retrieve map d << endlata and set create graph
    auto m = MapData::get_map("./maps/test.yaml");
    auto g = get_graph_data(m);
    g.root = {100, 50};
    g.end = {381, 360};
    cout << "Root node valid: " << g.is_node_valid(g.root) << endl;
    cout << "End node valid: " << g.is_node_valid(g.end) << endl;

    // Test A* algorithm (using Map)
    test_a_star(m, g);

    // Test BFS algorithm (using Map)
    test_bfs(m, g);

    // Test RRT* algorithm (using Map)
    //test_rrt_star(m, g);
}