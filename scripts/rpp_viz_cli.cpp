#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <chrono>
#include <iomanip>
#include <cstring>

#include "map_data.hpp"
#include "bfs.hpp"
#include "a_star.hpp"
#include "rrt_star.hpp"
#include "probability_roadmap.hpp"
//#include "d_star_lite.hpp"

#include "time_helper.hpp"
#include "map_helper.hpp"
#include "structs.hpp"
#include "script_constants.hpp"

#include "geometry_utils.hpp"

struct Parameters{
    string algo, map_yaml;
    bool show_debug = false, get_help = false, kill_script = false;
    int inflate_size = MapConstants::DEFAULT_INFLATE_SIZE, 
        neighbor_count = AlgoConstants::DEFAULT_NEIGHBOR_COUNT;
    float step_size = AlgoConstants::DEFAULT_STEP_SIZE;
    SampleCountByAlgo sample_counts;
    cell start, goal;
};

int compute_timeout = AlgoConstants::DEFAULT_COMPUTE_TIMEOUT;
vector<AlgoResult> algo_results;

void print_help_menu(){
    cout << "Description: A simple script to test different path planning algorithms.\n";
    cout << "options: \n";
    cout << "   -h, --help                                 Show this help message and exit.\n";
    cout << "   -f FILE, --file FILE                       Provide map yaml filepath.\n";
    cout << "   -i INFLATE_SIZE. --inflate-size INFLATE_SIZE\n";
    cout << "                                              Set size of boundaries (Default: " << MapConstants::DEFAULT_INFLATE_SIZE << ").\n";
    cout << "   -a ALGORITHM, --algorithm ALGORITHM        Set executed algoritm to one of the following:\n";
    cout << "                                              [bfs, a-star, rrt-star, prm, all].\n";
    cout << "   -l SAMPLE_LIMIT, --sample-limit SAMPLE_LIMIT\n";
    cout << "                                              Set a limit on the number of samples generated.\n";
    cout << "                                              Only supported for sample-based methods (Default: " << AlgoConstants::DEFAULT_SAMPLE_COUNT << ").\n";
    cout << "   -k NEIGHBORS, --neighbors NEIGHBORS        Set the number of neighbors a node can have.\n";
    cout << "                                              Exlusive to PRM algorithm (Default: " << AlgoConstants::DEFAULT_NEIGHBOR_COUNT << ")\n";
    cout << "   -z STEP_SIZE, --step-size STEP_SIZE        Set the step size for certain algorithms (Default: " << AlgoConstants::DEFAULT_STEP_SIZE << ")\n";
    cout << "   -s START_POS, --start-pos START_POS        Set start position [Format: \"int,int\"].\n";
    cout << "   -e END_POS, --end-pos END_POS              Set end position [Format: \"int,int\"].\n";
    cout << "   -d, --debug                                Provide more information for debugging.\n";
    cout << "   -t TIMEOUT, timeout TIMEOUT                Set timeout limit for algorithm computation\n";
    cout << "                                              (Default: " << AlgoConstants::DEFAULT_COMPUTE_TIMEOUT << " ms).\n";
}

string trim_whitespace(string word){
    int start = 0;
    int end = word.length()-1;
    while(start < end){
        if(word[start] != ' ' && word[end] != ' ') break;
        if(word[start] == ' ') start++;
        if(word[end] == ' ') end--;
    }
    return word.substr(start, end-start+1);
}

string remove_quotes(string word){
    int start = 0, end = word.length()-1;
    if(word[start] == '\'' && word[end] == '\'' ||
       word[start] == '\"' || word[end] == '\"') 
       return word.substr(start+1, end-start-1);
    else return word;   
}

bool is_valid_algo(string name){
    vector<string> valid_algos = {CLIConstants::BFS_ID, CLIConstants::A_STAR_ID, 
                                  CLIConstants::RRT_STAR_ID, CLIConstants::PRM_ID,
                                  CLIConstants::ALL_ID};
    for(auto algo: valid_algos){
        if(name == algo) return true;
    }
    return false;
}

Parameters get_params(int argc, char* argv[]){
    Parameters params;
    for(int i = 1; i < argc; i++){
        if(strcmp(argv[i], "-f") == 0 || strcmp(argv[i], "--file") == 0){
            string missing_err = "[ERROR] Mising file name";
            if(i+1 >= argc){
                cout << missing_err << endl;
                params.kill_script = true;
                break;
            } 
            else {
                if(argv[i+1][0] == '-'){
                    cout << missing_err << endl; 
                    params.kill_script = true;
                    break;
                }
                params.map_yaml = argv[i+1];
            }
            i++;
        }
        else if(strcmp(argv[i], "-i") == 0 || strcmp(argv[i], "--inflate-map") == 0){
            string missing_err = "[ERROR] Mising inflate value (integer)";
            if(i+1 >= argc){
                cout << missing_err << endl;
                params.kill_script = true;
                break;
            } 
            else {
                try{
                    params.inflate_size = std::stoi(argv[i+1]);
                }catch(std::invalid_argument e){
                    if(argv[i+1][0] == '-'){
                        cout << missing_err << endl; 
                        params.kill_script = true;
                        break;
                    }
                    cout << "[WARN] Could not convert \"" << argv[i+1] << "\" to an integer. Defaulting obstacle inflation to " 
                         << MapConstants::DEFAULT_INFLATE_SIZE << ".\n";
                    params.kill_script = false;
                }
                i++;  
            }       
        }
        else if(strcmp(argv[i], "-a") == 0 || strcmp(argv[i], "--algorithm") == 0){
            if(i+1 >= argc){
                cout << "[ERROR] Mising algorithm name" << endl;
                params.kill_script = true;
                break;
            } 
            else if(is_valid_algo(argv[i+1])){
                params.algo = argv[i+1];
                i++;
            }
            else{
                cout << "[ERROR] Unrecognized algorithm: \'" << params.algo << "\'\n"; 
                params.kill_script = true;   
            }
        }
        else if(strcmp(argv[i], "-l") == 0 || strcmp(argv[i], "--sample-limit") == 0){
            if(i+1 >= argc){
                cout << "[ERROR] Mising sample number limit" << endl;
                params.kill_script = true;
                break;
            }
            else {
                try{
                    string original_str = trim_whitespace(argv[i+1]);
                    string bracketless_str = original_str.substr(1, original_str.length()-2);
                    if(original_str[0] == '{'){  
                        string key, value, kv_pair;
                        stringstream json_str(bracketless_str);
                        while(getline(json_str, kv_pair, ',')){
                            stringstream kv_ss(kv_pair);
                            getline(kv_ss, key, ':');
                            getline(kv_ss, value, ':');
                            key = trim_whitespace(key);
                            key = remove_quotes(key);
                            value = trim_whitespace(value);
                            if(key == CLIConstants::RRT_STAR_ID)
                                params.sample_counts.rrt_star_count = std::stoi(value);
                            else if(key == CLIConstants::PRM_ID)
                                params.sample_counts.prm_count = std::stoi(value);
                            else {
                                string err_msg = "Make sure the " + key + " is a supported algoirthm and quotation marks are balanced.";
                                throw std::invalid_argument(err_msg);
                            }
                        }
                    }
                    else{
                        params.sample_counts.rrt_star_count = std::stoi(original_str);
                        params.sample_counts.prm_count = std::stoi(original_str);
                    }    
                }catch(std::invalid_argument e){
                    cout << "[ERROR] Invalid Argument: " << argv[i+1] << ".\nValue should be a single integer or json-like string.\nStack trace: " << e.what() << endl;
                    params.kill_script = true;   
                }
                i++;
            }
        }
        else if(strcmp(argv[i], "-k") == 0 || strcmp(argv[i], "--neighbors") == 0){
            string missing_err = "[ERROR] Mising neighbor count";
            if(i+1 >= argc){
                cout << missing_err << endl;
                params.kill_script = true;
                break;
            }
            else {
                try{
                    params.neighbor_count = std::stoi(argv[i+1]);
                }catch(std::invalid_argument e){
                    if(argv[i+1][0] == '-'){
                        cout << missing_err << endl;
                        params.kill_script = true;
                        break;
                    }
                    cout << "[WARN] Could not convert \"" << argv[i+1] << "\" to an integer. Defaulting max neighbor count to " 
                         << AlgoConstants::DEFAULT_NEIGHBOR_COUNT << ".\n";
                    params.kill_script = false;
                }
            }
            i++;
        }
        else if(strcmp(argv[i], "-z") == 0 || strcmp(argv[i], "--step-size") == 0){
            string missing_err = "[ERROR] Mising step size (float)";
            if(i+1 >= argc){
                cout << missing_err << endl;
                params.kill_script = true;
                break;
            }
            else {
                try{
                    params.step_size = std::stof(argv[i+1]);
                }catch(std::invalid_argument e){
                    if(argv[i+1][0] == '-'){
                        cout << missing_err << endl;
                        params.kill_script = true;
                        break;
                    }
                    cout << "[WARN] Could not convert \"" << argv[i+1] << "\" to an float value. Defaulting step size to " 
                         << AlgoConstants::DEFAULT_STEP_SIZE << ".\n";
                    params.kill_script = false;
                }
            }
            i++;
        }
        else if(strcmp(argv[i], "-s") == 0 || strcmp(argv[i], "--start-pos") == 0){
            string missing_err = "[ERROR] Mising start position";
            if(i+1 >= argc){
                cout << missing_err << endl;
                params.kill_script = true;
                break;                
            }
            else{
                if(argv[i+1][0] == '-'){
                    cout << missing_err << endl;
                    params.kill_script = true;
                    break;
                }
                params.start = MapHelper::get_positon(argv[i+1]);
                i++;
            } 
        }
        else if(strcmp(argv[i], "-e") == 0 || strcmp(argv[i], "--end-pos") == 0){
            string missing_err = "[ERROR] Mising end position";
            if(i+1 >= argc){
                cout << missing_err << endl;
                params.kill_script = true;
                break;                
            } 
            else{
                if(argv[i+1][0] == '-'){
                    cout << missing_err << endl;
                    params.kill_script = true;
                    break;
                }
                params.goal = MapHelper::get_positon(argv[i+1]);
                i++;
            }
        }
        else if(strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--debug") == 0){
            params.show_debug = true;
        }
        else if(strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--timeout") == 0){
            string missing_err = "[ERROR] Mising timeout value";
            if (i+1 >= argc){
                cout << missing_err << endl;
                params.kill_script = true;
                break;                
            }
            else{
                try{
                    compute_timeout = std::stoi(argv[i+1]);
                }
                catch(std::invalid_argument e){
                    if(argv[i+1][0] == '-'){
                        cout << missing_err << endl; 
                        params.kill_script = true;
                        break;
                    }
                    cout << "[WARN] Could not convert \"" << argv[i+1] << "\" to an integer. Defaulting timeout to " 
                         << AlgoConstants::DEFAULT_COMPUTE_TIMEOUT << " ms." << endl;
                    params.kill_script = false;
                }
                i++;
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
    cout << "\nBFS" << endl;
    auto bfs = BFS(g);
    
    auto start_time = TimeHelper::get_time("Start Time", true);
    bfs.solve(g.root, g.end, compute_timeout);
    auto end_time = TimeHelper::get_time("End Time", true);
    int duration = duration_cast<milliseconds>(end_time - start_time).count();
    
    auto results = bfs.reconstruct_path(g.root, g.end);
    vector<cell> path = results.first;
    float dist = results.second;
    vector<cell> travelled = bfs.get_travelled_nodes();
    AlgoResult ar = {CLIConstants::BFS_ID, duration, path, travelled, dist};
    print_results(ar, debug, compute_timeout);
    show_map("BFS", m, g.root, g.end, path, travelled, debug);
}

void run_astar(Map &m, Graph g, bool debug){
    cout << "\nA-STAR" << endl;
    auto as = AStar(g);
    
    auto start_time = TimeHelper::get_time("Start Time", true);
    as.solve(g.root, g.end, compute_timeout);
    auto end_time = TimeHelper::get_time("End Time", true);
    int duration = duration_cast<milliseconds>(end_time - start_time).count();
    
    auto results = as.reconstruct_path(g.root, g.end);
    vector<cell> path = results.first;
    float dist = results.second;
    vector<cell> travelled = as.get_travelled_nodes();
    AlgoResult ar = {CLIConstants::A_STAR_ID, duration, path, travelled, dist};
    print_results(ar, debug, compute_timeout);
    show_map("A*", m, g.root, g.end, path, travelled, debug);
}

void run_rrt_star(Map &m, Graph g, int sample_count, float step_size, bool debug){
    cout << "\nRRT-STAR" << endl;
    if(debug) cout << "Configuration:\n\tSample Count: " << sample_count << endl;
    auto rrt = RRTStar(g, sample_count);
    rrt.set_step_size(step_size);

    auto start_time = TimeHelper::get_time("Start Time", true);
    rrt.solve(g.root, g.end, compute_timeout);
    auto end_time = TimeHelper::get_time("End Time", true);
    int duration = duration_cast<milliseconds>(end_time - start_time).count();

    vector<cell> path;
    auto travelled = rrt.get_travelled_tree();
    if(rrt.goal_reached){
        auto results = rrt.reconstruct_path(g.root, g.end);
        path = Sampling::get_connected_path(results.first);
        float dist = results.second;
        AlgoResult ar = {CLIConstants::RRT_STAR_ID, duration, path, travelled, dist};
        print_results(ar, debug, compute_timeout);
    }
    else {
        cout << "Goal could not be reached. Please check the following:";
        cout << "\n\tstart point\n\tend point\n\t# of max num of samples\n\talgorithm timeout limit\n";
    }
    show_map("RRT*", m, g.root, g.end, path, travelled, debug);
}

void run_prm(Map &m, Graph g, int sample_count, int neighbor_count, bool debug){
    cout << "\nPRM" << endl;
    /*// Not sure step size/max distance is needed
    int step_size;
    string step_size_str;
    try{  
        cout << "Set step size: ";
        cin >> step_size_str;
        step_size = std::stoi(step_size_str);
    } 
    catch(std::invalid_argument e){
        step_size = CLIConstants::DEFAULT_STEP_SIZE;
        cout << "Invalid value: " << step_size_str << ", Defaulting to " << step_size << endl;
    }*/
    if(debug){
        cout << "Configuration\n\tSample Count: " << sample_count
             << "\n\tNeighbor count: " << neighbor_count << endl; 
             //<< "\n\tStep size: " << step_size << endl;
    }

    auto prm = PROBABILITY_ROADMAP(g, sample_count, neighbor_count);
    
    auto start_time = TimeHelper::get_time("Start Time", true);
    prm.solve(g.root, g.end, compute_timeout);
    auto end_time = TimeHelper::get_time("End Time", true);
    int duration = duration_cast<milliseconds>(end_time - start_time).count();

    vector<cell> path, travelled;
    auto results = prm.reconstruct_path(g.root, g.end);
    path = Sampling::get_connected_path(results.first);
    float dist = results.second;
    travelled = prm.get_travelled_roadmap();
    AlgoResult ar = {CLIConstants::PRM_ID, duration, path, travelled, dist};
    print_results(ar, debug, compute_timeout);
    show_map("PRM", m, g.root, g.end, path, travelled, debug);
}

int main(int argc, char* argv[]){
    auto params = get_params(argc, argv);
    if(params.get_help){
        print_help_menu();
    }else if(!params.kill_script){
        try{
            Map map = MapData::get_map(params.map_yaml);
            map.boundaries = MapData::inflate_boundaries(map, params.inflate_size);
            auto g = MapData::get_graph_from_map(map);
            if(g.is_node_valid(params.start)) g.root = params.start;
            else cout << "Start node: {" << params.start.first << "," << params.start.second << "} is invalid\n"; 
            
            if(g.is_node_valid(params.goal)) g.end = params.goal;
            else cout << "End node: {" << params.goal.first << "," << params.goal.second << "} is invalid\n"; 

            if(g.is_node_valid(params.start) && g.is_node_valid(params.goal)){
                if(params.algo == CLIConstants::BFS_ID || params.algo == CLIConstants::ALL_ID) 
                    run_bfs(map, g, params.show_debug);
                if(params.algo == CLIConstants::A_STAR_ID || params.algo == CLIConstants::ALL_ID) 
                    run_astar(map, g, params.show_debug);
                if(params.algo == CLIConstants::RRT_STAR_ID || params.algo == CLIConstants::ALL_ID) 
                    run_rrt_star(map, g, params.sample_counts.rrt_star_count, params.step_size, params.show_debug);
                if(params.algo == CLIConstants::PRM_ID || params.algo == CLIConstants::ALL_ID)
                    run_prm(map, g, params.sample_counts.prm_count, params.neighbor_count, params.show_debug);
                //if(params.algo == "d-lite" || params.algo == ALL_ID) run_d_star_lite(map, g, params.show_debug);
                if(!is_valid_algo(params.algo)) cout << "Unrecognized algorithm: " << params.algo << endl;
            }
        }
        catch(std::runtime_error re){
            cout << re.what() << endl;
        }
    }
}