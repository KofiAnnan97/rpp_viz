#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <iomanip>
#include <cstring>

#include "map_data.hpp"
#include "bfs.hpp"
#include "a_star.hpp"
//#include "d_star_lite.hpp"
#include "rrt_star.hpp"
#include "time_helper.hpp"
#include "map_helper.hpp"

struct Parameters{
    string algo, map_yaml;
    bool show_debug = false, get_help = false, kill_script = false;
    int inflate_size = 3, max_iter = 10000;
    cell start, goal;
};

int COMPUTE_TIMEOUT = 600000; //in milliseconds
vector<AlgoResult> algo_results;

// Constants
const string BFS_ID = "bfs";
const string A_STAR_ID = "a-star";
const string RRT_STAR_ID = "rrt-star";
const string ALL_ID = "all";

void print_help_menu(){
    cout << "Description: A simple script to test different path planning algorithms.\n";
    cout << "options: \n";
    cout << "   -h, --help                            Show this help message and exit.\n";
    cout << "   -f FILE, --file FILE                  Provide map yaml filepath.\n";
    cout << "   -i INFLATE_SIZE. --inflate-size INFLATE_SIZE\n";
    cout << "                                         Set size of boundaries (Default: 3).\n";
    cout << "   -a ALGORITHM, --algorithm ALGORITHM   Set executed algoritm to one of the following:\n";
    cout << "                                         [bfs, a-star, rrt-star, all].\n";
    cout << "   -l MAX_ITER, --max-iter MAX_ITER      Set limit the number of iterations executed.\n";
    cout << "                                         Only supported for sample-based methods (Default: 10000).\n";
    cout << "   -s START_POS, --start-pos START_POS   Set start position [Format: \"int,int\"].\n";
    cout << "   -e END_POS, --end-pos END_POS         Set end position [Format: \"int,int\"].\n";
    cout << "   -d, --debug                           Provide more information for debugging.\n";
    cout << "   -t TIMEOUT, timeout TIMEOUT           Set timeout limit for algorithm computation\n";
    cout << "                                         (Default: 600000 ms).\n";
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
                params.start = MapHelper::get_positon(argv[i+1]);
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
                params.goal = MapHelper::get_positon(argv[i+1]);
                i++;
            }
        }
        else if(strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--debug") == 0){
            params.show_debug = true;
        }
        else if(strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--timeout") == 0){
            if (i+1 >= argc){
                cout << "Mising timeout value" << endl;
                params.kill_script = true;
                break;                
            }
            else{
                try{
                    COMPUTE_TIMEOUT = std::stoi(argv[i+1]);
                    i++;
                }
                catch(std::invalid_argument e){
                    cout << "Could not convert \"" << argv[i+1] << "\" value to integer. Defaulting to 600000 ms." << endl;
                    params.kill_script = true;
                    break;
                }
            }
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

void print_results(AlgoResult ar, bool debug, int timeout){
    auto duration_converted = TimeHelper::convert_from_ms(ar.duration);
    if(ar.duration >= timeout)
        cout << "Computation exceeded " << duration_converted.first << " " <<  duration_converted.second << endl;
    else
        cout << "Elapsed Time: " << duration_converted.first << " " <<  duration_converted.second << endl;

    if(debug){
        cout << "# of Nodes: " << ar.path.size() << endl;
        std::cout << "Path: [";
        for(auto p: ar.path) std::cout << "(" << p.first << "," << p.second << "), ";
        std::cout << "]\n";
    }
    std::cout << "Distance: " << ar.dist << std::endl;
}

bool is_valid_algo(string name){
    vector<string> valid_algos = {BFS_ID, A_STAR_ID, RRT_STAR_ID, ALL_ID};
    for(auto algo: valid_algos){
        if(name == algo) return true;
    }
    return false;
}

void show_map(string title, Map &m, cell sp, cell ep, vector<cell> path, vector<cell> travelled, bool debug){
    Map sm;
    if(debug) {
        title = "Debug " + title;
        sm = MapData::debug_map(m, path, travelled, sp, ep);
    }
    else sm = MapData::add_path_to_map(m, path, sp, ep);
    MapData::show_map(title, sm);
}

void run_bfs(Map &m, Graph g, bool debug){
    cout << "BFS" << endl;
    auto bfs = BFS(g);
    
    auto start_time = TimeHelper::get_time("Start Time", true);
    bfs.solve(g.root, g.end, COMPUTE_TIMEOUT);
    auto end_time = TimeHelper::get_time("End Time", true);
    int duration = duration_cast<milliseconds>(end_time - start_time).count();
    
    auto results = bfs.reconstruct_path(g.root, g.end);
    vector<cell> path = results.first;
    float dist = results.second;
    vector<cell> travelled = bfs.get_travelled_nodes();
    AlgoResult ar = {BFS_ID, duration, path, travelled, dist};
    print_results(ar, debug, COMPUTE_TIMEOUT);
    show_map("BFS", m, g.root, g.end, path, travelled, debug);
}

void run_astar(Map &m, Graph g, bool debug){
    cout << "A-STAR" << endl;
    auto as = AStar(g);
    
    auto start_time = TimeHelper::get_time("Start Time", true);
    as.solve(g.root, g.end, COMPUTE_TIMEOUT);
    auto end_time = TimeHelper::get_time("End Time", true);
    int duration = duration_cast<milliseconds>(end_time - start_time).count();
    
    auto results = as.reconstruct_path(g.root, g.end);
    vector<cell> path = results.first;
    float dist = results.second;
    vector<cell> travelled = as.get_travelled_nodes();
    AlgoResult ar = {A_STAR_ID, duration, path, travelled, dist};
    print_results(ar, debug, COMPUTE_TIMEOUT);
    show_map("A*", m, g.root, g.end, path, travelled, debug);
}

void run_rrt_star(Map &m, Graph g, int max_iter, bool debug){
    cout << "RRT-STAR" << endl;
    auto rrt = RRTStar(g, max_iter);
    
    auto start_time = TimeHelper::get_time("Start Time", true);
    rrt.solve(g.root, g.end, COMPUTE_TIMEOUT);
    auto end_time = TimeHelper::get_time("End Time", true);
    int duration = (end_time - start_time).count();

    vector<cell> path, travelled;
    if(rrt.goal_reached){
        auto results = rrt.reconstruct_path(g.root, g.end);
        path = results.first;
        float dist = results.second;
        travelled = rrt.get_travelled_nodes();
        AlgoResult ar = {RRT_STAR_ID, duration, path, travelled, dist};
        print_results(ar, debug, COMPUTE_TIMEOUT);
    }
    else {
        cout << "Goal could not be reached. Please check the following:";
        cout << "\n\tstart point\n\tend point\n\t# of max iterations\n\talgorithm timeout limit\n";
    }
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
            if(params.algo == BFS_ID || params.algo == ALL_ID) run_bfs(map, g, params.show_debug);
            if(params.algo == A_STAR_ID || params.algo == ALL_ID) run_astar(map, g, params.show_debug);
            if(params.algo == RRT_STAR_ID || params.algo == ALL_ID) run_rrt_star(map, g, params.max_iter, params.show_debug);
            //if(params.algo == "d-lite" || params.algo == ALL_ID) run_d_star_lite(map, g, params.show_debug);
            if(!is_valid_algo(params.algo)) cout << "Unrecognized algorithm: " << params.algo << endl;
        }
    }
}
