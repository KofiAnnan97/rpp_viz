#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem>

#include "map_data.hpp"
#include "script_constants.hpp"

namespace fs = std::filesystem;

struct Parameters{
    string txt_path, title;
    bool get_help = false, kill_script = false;
};

class GenerateMap{
    public:
        static Map parse_text(string filepath){
            Map map;
            fstream txt_input;
            txt_input.open(filepath, ios::in);
            if(txt_input.is_open()){
                string line, value;
                getline(txt_input, line);
                map.resolution = stof(line);
                getline(txt_input, line);
                stringstream ss(line);
                getline(ss, value, ' ');
                map.px_height = stoi(value);
                map.boundaries = new int*[map.px_height];
                getline(ss, value, ' ');
                map.px_width = stoi(value);
                for(int k = 0; k < map.px_height; k++) map.boundaries[k] = new int[map.px_width];
                for(int row=0; row<map.px_height; row++){
                    getline(txt_input, line);
                    stringstream ss(line);
                    for(int col=0; col<map.px_width; col++){
                        getline(ss, value, ',');
                        int pixel = stoi(value);
                        if(pixel == 0) map.boundaries[row][col] = MapConstants::OPEN_SPACE_INT;
                        else if(pixel == 1) map.boundaries[row][col] = MapConstants::OBSTACLE_INT;
                    }
                }
                map.m_width = (map.resolution+0.005)*map.px_width;
                map.m_height = (map.resolution+0.005)*map.px_height;
                //MapData::print_boundary(map.boundaries, map.px_width, map.px_height);
            }
            else throw std::runtime_error("Could not find file: " + filepath);
            return map;
        }

        static void generate_map_pgm(Map map, fs::path path, string title){
            string filename = title + ".pgm";
            fs::path filepath = path / filename.c_str();
            std::ofstream pgm_output(filepath, std::ios_base::binary | std::ios_base::out);
            if(pgm_output.is_open()){
                // Populating file
                pgm_output << "P5" << endl;
                pgm_output << "# CREATOR: gen_ros_map.cpp " << map.resolution << " m/pix" << endl;
                pgm_output << map.px_width << " " << map.px_height << endl;
                pgm_output << MapGenConstants::HIGHEST_PX_VALUE << endl;
                int size = map.px_width*map.px_height; 
                char *buffer = new char[size];
                for(int j = 0; j < size; j++) buffer[j] = 0x00;
                for(int row = 0; row<map.px_height; row++){
                    for(int col=0; col<map.px_width; col++){
                        int idx = row*map.px_width + col;
                        uint8_t px;
                        if (map.boundaries[row][col] == MapConstants::OBSTACLE_INT)         
                            px = MapGenConstants::OBSTACLE_PX_VALUE;
                        else if(map.boundaries[row][col] == MapConstants::OPEN_SPACE_INT)
                            px = MapGenConstants::HIGHEST_PX_VALUE;
                        else
                            px = MapGenConstants::UNKNOWN_PX_VALUE;
                        buffer[idx] = char(px);
                    }
                }
                pgm_output.write(buffer, size);
                pgm_output.close();
            }
        }

        static void generate_map_yaml(Map map, fs::path path, string title){
            string filename = title + ".yaml";
            fs::path filepath = path / filename.c_str();
            std::ofstream yaml_output(filepath, std::ios_base::out);
            if(yaml_output.is_open()){
                // Default values 
                float origin_x = 0;
                float origin_y = 0;
                float origin_z = 0;
                int negate = MapGenConstants::DEFAULT_NEGATE;
                float occupied_thresh = MapGenConstants::DEFAULT_OCCUPIED_THRESH;
                float free_thresh = MapGenConstants::DEFAULT_FREE_THRESH;

                // updating values
                origin_x = -1.0*(map.m_width/2);
                origin_y = -1.0*(map.m_height/2);

                // Populating file
                yaml_output << "image: " << title << ".pgm" << endl;
                yaml_output << "resolution: " << map.resolution << endl;
                yaml_output << "origin: [" << origin_x << ", " << origin_y << ", " << origin_z << "]\n";
                yaml_output << "negate: " << negate << endl;
                yaml_output << "occupied_thresh: " << occupied_thresh << endl;
                yaml_output << "free_thresh: " << free_thresh;
                yaml_output.close();
            }
        }
};