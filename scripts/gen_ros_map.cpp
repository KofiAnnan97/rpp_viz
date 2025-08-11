#include <string>

#include "gen_ros_map.hpp"

using namespace std;

void print_help_menu(){
    cout << "Description: Generate ROS map using yaml file.\n";
    cout << "options: \n";
    cout << "   -h, --help                  Show this help message and exit.\n";
    cout << "   -f FILE, --file FILE        Provide path for yaml file.\n";
    cout << "   -t TITLE. --title TITLE     Set name of generated map.\n";
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
            else params.txt_path = argv[i+1];
            i++;
        }
        else if(strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--title") == 0){
            if(i+1 >= argc){
                cout << "Mising title for generated map" << endl;
                params.kill_script = true;
                break;
            } 
            else params.title = argv[i+1];
            i++;       
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

int main(int argc, char* argv[]){
    auto params = get_params(argc, argv);
    if(params.get_help) print_help_menu();
    else if(!params.kill_script){
        auto map = GenerateMap::parse_text(params.txt_path);
        GenerateMap::generate_map_pgm(map, ".", params.title);
        GenerateMap::generate_map_yaml(map, ".", params.title);
    }
}