#include <chrono>
#include <iomanip>
#include "a_star.hpp"
#include "bfs.hpp"
#include "rrt_star.hpp"

using namespace std::chrono;

void test_map_data(){
    Map map_data = MapData::get_map("./maps/example1.yaml");
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
    auto g = MapData::get_graph_from_map(m);
    return g;
}

Graph simple_graph(){
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

Map get_simple_map(){
    Map map;
    map.px_height = 10;
    map.px_width = 10;
    int temp[10][10] = {
        {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
        {-1,0,0,0,0,0,0,0,0,-1},
        {-1,0,0,0,0,0,0,0,0,-1},
        {-1,0,0,0,0,0,0,0,0,-1},
        {-1,0,0,0,-1,-1,0,0,0,-1},
        {-1,0,0,0,-1,-1,0,0,0,-1},
        {-1,0,0,0,0,0,0,0,0,-1},
        {-1,0,0,0,0,0,0,0,0,-1},
        {-1,0,0,0,0,0,0,0,0,-1},
        {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
    };
    map.boundaries = new int*[map.px_height];
    for(int k = 0; k < map.px_height; k++) map.boundaries[k] = new int[map.px_width];
    for(int row = 0; row < map.px_height; row++){
        for(int col = 0; col < map.px_width; col++){
            map.boundaries[row][col] = temp[row][col];
        }
    }
    return map;
}

void test_a_star_simple(){
    auto g = simple_graph();
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

void test_rrt_star(Map &m, Graph g, int max_iter){
    cout << "RRT-STAR" << endl;
    auto rrt = RRTStar(g, 1.0, m.px_width, m.px_height, max_iter);
    
    auto start_time = high_resolution_clock::now();
    auto s_t = std::chrono::system_clock::to_time_t(start_time);
    cout << "Start Time: " << std::put_time(std::localtime(&s_t), "%F %T\n") << std::flush;
    rrt.solve(g.root, g.end);
    auto end_time = high_resolution_clock::now();
    auto e_t = std::chrono::system_clock::to_time_t(end_time);
    cout << "End Time: " << std::put_time(std::localtime(&e_t), "%F %T\n") << std::flush;
    auto duration = duration_cast<milliseconds>(end_time- start_time);
    std::cout << "Elapsed Time: " << duration.count() << " ms\n";
    
    vector<pair<int, int>> path;
    if(rrt.goal_reached){
        auto results = rrt.reconstruct_path(g.root, g.end);
        path = results.first;
        float dist = results.second;
        cout << "# of Nodes: " << path.size() << endl;
        /*std::cout << "Path: [";
        for(auto p: path){
            std::cout << "(" << p.first << "," << p.second << "), ";
        }
        std::cout << "]\n";*/
        std::cout << "Distance: " << dist << std::endl;
    }
    else {
        cout << "Goal could not be reached. Please check the following:";
        cout << "\n\tstart point\n\tend point\n\t# of max iterations\n\tstep size\n";
    }
    auto travelled = rrt.get_travelled_nodes(g.root, g.end);
    Map dm = MapData::debug_map(m, path, travelled, g.root, g.end);
    MapData::show_map("Debug RRT*", dm);
}

void test_rrt_star_simple(){
    auto rrt_map = get_simple_map();
    //MapData::show_map("Simple Map", rrt_map);
    auto g = get_graph_data(rrt_map);
    g.root = {2,2};
    g.end = {2,7}; 
    cout << "Root node valid: " << g.is_node_valid(g.root) << endl;
    cout << "End node valid: " << g.is_node_valid(g.end) << endl;
    test_rrt_star(rrt_map, g, 50);
}

void test_conversions(){
    cout << "TESTING CONVERSIONS" << endl;
    auto m = MapData::get_map("./maps/example1.yaml");
    /*MapData::show_map("Original", m);
    m.boundaries = MapData::inflate_boundaries(m, 7);
    MapData::show_map("Inflated", m);*/
    //cout << "Pose: " << m.m_width << " (width) x " << m.m_height << " (height)\n";
    int x = m.px_width/2;
    int y = m.px_height/2;
    cout << "Center {" << y << "," << x << "}:\n";
    pair<float,float> pose = MapData::PIXEL2POSE(m, {y,x});
    cout << "\tPose: (" << pose.first << "," << pose.second << ")\n";
    pair<int,int> px = MapData::POSE2PIXEL(m, pose.first, pose.second);//0.0, 0.0);//
    cout << "\tPX: (" << px.first << "," << px.second << ")\n";
    x = 0;
    y = 0;
    cout << "Top-Left Corner {" << y << "," << x << "}:\n";
    pose = MapData::PIXEL2POSE(m, {y,x});
    cout << "\tPose: (" << pose.first << "," << pose.second << ")\n";
    px = MapData::POSE2PIXEL(m, pose.first, pose.second);//-m.m_width/2, -m.m_height/2);//
    cout << "\tPX: (" << px.first << "," << px.second << ")\n";
    x = m.px_width-1;
    y = 0;
    cout << "Top-Right Corner {" << y << "," << x << "}:\n";
    pose = MapData::PIXEL2POSE(m, {y,x});
    cout << "\tPose: (" << pose.first << "," << pose.second << ")\n";
    px = MapData::POSE2PIXEL(m, pose.first, pose.second);//-m.m_width/2, m.m_height/2);//
    cout << "\tPX: (" << px.first << "," << px.second << ")\n";
    x = 0;
    y = m.px_height-1;
    cout << "Bottom-Left Corner {" << y << "," << x << "}:\n";
    pose = MapData::PIXEL2POSE(m, {y,x});
    cout << "\tPose: (" << pose.first << "," << pose.second << ")\n";
    px = MapData::POSE2PIXEL(m, pose.first, pose.second);//m.m_width/2, -m.m_height/2);//
    cout << "\tPX: (" << px.first << "," << px.second << ")\n";
    x = m.px_width-1;
    y = m.px_height-1;
    cout << "Bottom-Right Corner {" << y << "," << x << "}:\n";
    pose = MapData::PIXEL2POSE(m, {y,x});
    cout << "\tPose: (" << pose.first << "," << pose.second << ")\n";
    px = MapData::POSE2PIXEL(m, pose.first, pose.second);//m.m_width/2, m.m_height/2);//
    cout << "\tPX: (" << px.first << "," << px.second << ")\n";
}

int main(){
    // Test Data retrieval from PGM file
    //test_map_data();

    // Test Map & Pose Conversions
    test_conversions();

    // Test A-star algorithm (Simple Data)
    //test_a_star_simple();

    // Test RRT* algorithm (Simple Data)
    //test_rrt_star_simple();

    // Retrieve map data and set create graph
    auto m = MapData::get_map("./maps/example1.yaml");
    m.boundaries = MapData::inflate_boundaries(m, 5);
    auto g = get_graph_data(m);
    g.root = {300, 50};
    g.end =  {381, 360};
    cout << "Root node valid: " << g.is_node_valid(g.root) << endl;
    cout << "End node valid: " << g.is_node_valid(g.end) << endl;

    // Test A* algorithm (using Map)
    test_a_star(m, g);

    // Test BFS algorithm (using Map)
    test_bfs(m, g);

    // Test RRT* algorithm (using Map)
    test_rrt_star(m, g, 10000);
}