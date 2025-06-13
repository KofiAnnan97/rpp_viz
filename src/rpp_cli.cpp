#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <iomanip>
#include <cstring>

#include "map_data.hpp"
#include "bfs.hpp"
#include "a_star.hpp"
#include "rrt_star.hpp"

using namespace std::chrono;

typedef std::chrono::_V2::system_clock::time_point c_time_point;

struct Parameters{
    string algo, map_yaml;
    bool show_debug = false, get_help = false, kill_script = false;
    int inflate_size = 3, max_iter = 10000;
    cell start, goal;
};

void print_help_menu(){
    cout << "Description: A simple script to test different path planning algorithms." << endl;
    cout << "options: " << endl;
    cout << "   -h, --help                            Show this help message and exit." << endl;
    cout << "   -f FILE, --file FILE                  Provide map yaml filepath." << endl;
    cout << "   -i INFLATE_SIZE. --inflate-size INFLATE_SIZE" << endl;
    cout << "                                         Set size of boundaries (Default: 3)." << endl;
    cout << "   -a ALGORITHM, --algorithm ALGORITHM   Set executed algoritm to one of the following:" << endl;
    cout << "                                         [bfs, a-star, rrt-star, all]." << endl;
    cout << "   -l MAX_ITER, --max-iter MAX_ITER      Set limit the number of iterations executed." << endl;
    cout << "                                         Only supported for sample-based methods (Default: 10000)." << endl;
    cout << "   -s START_POS, --start-pos START_POS   Set start position [Format: \"int,int\"]." << endl;
    cout << "   -e END_POS, --end-pos END_POS         Set end position [Format: \"int,int\"]." << endl;
    cout << "   -d, --debug                           Provide more information for debugging." << endl;
}

cell get_positon(string pos_str){
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
        cout << "One or both values cannot be resolved as an integer: " << pos_str << endl;
    }
    return pos;
}

Parameters get_params(int argc, char* argv[]){
    Parameters params;
    for(int i = 1; i < argc; i++){
        if(strcmp(argv[i], "-f") == 0 || strcmp(argv[i], "--file") == 0){
            if(i+1 >= argc){
                cout << "Mising file name" << endl;
                params.kill_script = true;
                break;
            } 
            else params.map_yaml = argv[i+1];
            i++;
        }
        else if(strcmp(argv[i], "-i") == 0 || strcmp(argv[i], "--inflate-map") == 0){
            if(i+1 >= argc){
                cout << "Mising inflate value (integer)" << endl;
                params.kill_script = true;
                break;
            } 
            else {
                try{
                    params.inflate_size = std::stoi(argv[i+1]);
                    i++;
                }catch(std::invalid_argument e){
                    cout << "Could not convert \"" << argv[i+1] << "\" value to integer. Defaulting to 3." << endl;
                    params.kill_script = true;
                    break;
                }  
            }       
        }
        else if(strcmp(argv[i], "-a") == 0 || strcmp(argv[i], "--algorithm") == 0){
            if(i+1 >= argc){
                cout << "Mising algorithm name" << endl;
                params.kill_script = true;
                break;
            } 
            else params.algo = argv[i+1];
            i++;
        }
        else if(strcmp(argv[i], "-l") == 0 || strcmp(argv[i], "--iter-limit") == 0){
            if(i+1 >= argc){
                cout << "Mising algorithm name" << endl;
                params.kill_script = true;
                break;
            }
            else {
                try{
                    params.max_iter = std::stoi(argv[i+1]);
                    i++;
                }catch(std::invalid_argument e){
                    cout << "Could not convert \"" << argv[i+1] << "\" value to integer. Defaulting to 10000" << endl;
                    params.kill_script = true;
                }
            }
        }
        else if(strcmp(argv[i], "-s") == 0 || strcmp(argv[i], "--start-pos") == 0){
            if(i+1 >= argc){
                 cout << "Mising start position" << endl;
                 params.kill_script = true;
                 break;
            }
            else{
                params.start = get_positon(argv[i+1]);
                i++;
            } 
        }
        else if(strcmp(argv[i], "-e") == 0 || strcmp(argv[i], "--end-pos") == 0){
            if(i+1 >= argc){
                cout << "Mising end position" << endl;
                params.kill_script = true;
                break;                
            } 
            else{
                params.goal = get_positon(argv[i+1]);
                i++;
            }
        }
        else if(strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--debug") == 0){
            params.show_debug = true;
        }
        else if(strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0){
            params.get_help = true;
            break;
        }
        else{
            cout << "Unrecognized command: " << argv[i] << endl;
            params.kill_script = true;
            break;
        }
    }
    return params;
}

void print_results(Graph g, vector<cell> path, float dist, c_time_point start_time, c_time_point end_time, bool debug){
    auto duration = duration_cast<milliseconds>(end_time- start_time);
    std::cout << "Elapsed Time: " << duration.count() << " ms\n";
    cout << "# of Nodes: " << path.size() << endl;
    if(debug){
        std::cout << "Path: [";
        for(auto p: path){
            std::cout << "(" << p.first << "," << p.second << "), ";
        }
        std::cout << "]\n"; 
    }
    std::cout << "Distance: " << dist << std::endl;
}

void show_map(string title, Map &m, cell sp, cell ep, vector<cell> path, vector<cell> travelled, bool debug){
    Map sm;
    if(debug) {
        title = "Debug " + title;
        sm = MapData::debug_map(m, path, travelled, sp, ep);
    }
    else sm = MapData::add_path_to_map(m, path);
    MapData::show_map(title, sm);
}

c_time_point get_time(string time_title){
    auto now = high_resolution_clock::now();
    auto n = std::chrono::system_clock::to_time_t(now);
    cout << time_title << ": " << std::put_time(std::localtime(&n), "%F %T\n") << std::flush;
    return now;
}

void run_bfs(Map &m, Graph g, bool debug){
    cout << "BFS" << endl;
    auto bfs = BFS(g);
    
    auto start_time = get_time("Start Time");
    bfs.solve(g.root, g.end);
    auto end_time = get_time("End Time");
    
    auto results = bfs.reconstruct_path(g.root, g.end);
    vector<cell> path = results.first;
    float dist = results.second;
    print_results(g, path, dist, start_time, end_time, debug);
    vector<cell> travelled;
    if(debug) travelled = bfs.get_travelled_nodes();
    show_map("BFS", m, g.root, g.end, path, travelled, debug);
}

void run_astar(Map &m, Graph g, bool debug){
    cout << "A-STAR" << endl;
    auto as = AStar(g);
    
    auto start_time = get_time("Start Time");
    as.solve(g.root, g.end);
    auto end_time = get_time("End Time");
    
    auto results = as.reconstruct_path(g.root, g.end);
    vector<cell> path = results.first;
    float dist = results.second;
    print_results(g, path, dist, start_time, end_time, debug);
    vector<cell> travelled;
    if(debug) travelled = as.get_travelled_nodes();
    show_map("A*", m, g.root, g.end, path, travelled, debug);
}

void run_rrt_star(Map &m, Graph g, int max_iter, bool debug){
    cout << "RRT-STAR" << endl;
    auto rrt = RRTStar(g, m.px_width, m.px_height, max_iter);
    
    auto start_time = get_time("Start Time");
    rrt.solve(g.root, g.end);
    auto end_time = get_time("End Time");
    
    vector<cell> path;
    if(rrt.goal_reached){
        auto results = rrt.reconstruct_path(g.root, g.end);
        path = results.first;
        float dist = results.second;
        print_results(g, path, dist, start_time, end_time, debug);
    }
    else {
        cout << "Goal could not be reached. Please check the following:";
        cout << "\n\tstart point\n\tend point\n\t# of max iterations\n";
    }
    vector<cell> travelled;
    if(debug) travelled = rrt.get_travelled_nodes(); 
    show_map("RRT*", m, g.root, g.end, path, travelled, debug);
}

int main(int argc, char* argv[]){
    auto params = get_params(argc, argv);
    /*cout << params.map_yaml << endl;
    cout << params.inflate_size << endl;
    cout << params.algo << endl;
    cout << params.max_iter << endl;
    cout << params.show_debug << endl;*/
    if(params.get_help){
        print_help_menu();
    }else if(!params.kill_script){
        auto map = MapData::get_map(params.map_yaml);
        map.boundaries = MapData::inflate_boundaries(map, params.inflate_size);
        auto g = MapData::get_graph_from_map(map);
        if(g.is_node_valid(params.start)) g.root = params.start;
        else cout << "Start node: {" << params.start.first << "," << params.start.second << "} is invalid\n"; 
        
        if(g.is_node_valid(params.goal)) g.end = params.goal;
        else cout << "End node: {" << params.goal.first << "," << params.goal.second << "} is invalid\n"; 

        if(g.is_node_valid(params.start) && g.is_node_valid(params.goal)){
            if(params.algo == "bfs") run_bfs(map, g, params.show_debug);
            else if(params.algo == "a-star") run_astar(map, g, params.show_debug);
            else if(params.algo == "rrt-star") run_rrt_star(map, g, params.max_iter, params.show_debug);
            else if(params.algo == "all"){
                run_bfs(map, g, params.show_debug);
                run_astar(map, g, params.show_debug);
                run_rrt_star(map, g, params.max_iter, params.show_debug);
            }
            else cout << "Unrecognized algorithm: " << params.algo << endl;
        }
    }
}