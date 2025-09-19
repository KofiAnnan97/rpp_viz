#include <filesystem>
#include <gtest/gtest.h>

#include "gen_ros_map.hpp"
#include "common.hpp"

/* 
Data Extraction 
    Check that map data is correctly converted to obstacle maps
    Make sure the yaml and pgm have the correct data
*/ 

using namespace testing;
using namespace std::chrono;
namespace fs = std::filesystem;

class Data_Extraction : public Test{
    public: 
        string pgm_path, yaml_path;

    protected:
        fs::path tmp_path = "temp";
        string title = "test";
        string pgm_file = title+".pgm";
        string yaml_file = title+".yaml";

        void SetUp() override {
            Map original = get_simple_map();
            fs::create_directory(tmp_path);
            pgm_path = tmp_path / pgm_file;
            yaml_path = tmp_path / yaml_file;
            
            // Generate map files
            GenerateMap::generate_map_pgm(original, tmp_path, title);
            GenerateMap::generate_map_yaml(original, tmp_path, title);
        }

        void TearDown() override {
            fs::remove_all(tmp_path);
        }
};

AssertionResult checkPGM(string pgm_path){
    fstream pgm;
    pgm.open(pgm_path, ios::in | ios::binary);
    if(pgm.is_open()){
        string line, word;
        bool vals_correct = true;
        int expected_height = 10;
        int expected_width = 20;
        int expected_highest_value = 255;
        string err_msg = "Error: ";
        getline(pgm, line);
        if(strcmp("P5",line.c_str()) != 0){
            vals_correct = false;
            err_msg += "P5 ,";
        }
        getline(pgm, line);
        if(strcmp("# CREATOR: gen_ros_map.cpp 0.05 m/pix",line.c_str()) != 0){
            if(vals_correct) vals_correct = false;
            err_msg += " comment is incorrect, ";
        }
        getline(pgm, line);
        stringstream ss(line);
        getline(ss, word, ' ');
        if(stoi(word) != expected_width){
            if(vals_correct) vals_correct = false;
            auto sub_err = " width should be " + std::to_string(expected_width) + " but is " + word + ", ";
            err_msg += sub_err;
        }
        getline(ss, word, ' ');
        if(stoi(word) != expected_height){
            if(vals_correct) vals_correct = false;
            auto sub_err = " height should be " + std::to_string(expected_height) + " but is " + word + ", ";
            err_msg += sub_err;
        }
        getline(pgm, line);
        if(stoi(line) != expected_highest_value){
            if(vals_correct) vals_correct = false;
            auto sub_err = " highest value should be " + std::to_string(expected_highest_value) + " but is " + line + ", ";
            err_msg += sub_err;
        }
        if(vals_correct) return AssertionSuccess();
        else return AssertionFailure() << "failed, " << err_msg;
    }
    else return AssertionFailure() << "failed, Could not open " << pgm_path;
}

AssertionResult checkYAML(string yaml_path, string title){
    fstream yaml;
    yaml.open(yaml_path, ios::in);
    if(yaml.is_open()){
        string line, word;
        bool vals_correct = true;
        string expected_image = " "+title+".pgm";
        float expected_resolution = 0.05;
        string expected_origin = " [-0.56, -0.28, 0]";
        int expected_negate = 0;
        float expected_occupied_thresh =  0.65;
        float expected_free_thresh = 0.25;
        string err_msg = "Errors: ";
        for(int i=0; i<6; i++){
            getline(yaml, line);
            stringstream ss(line);
            getline(ss, word, ':');
            string id = word;
            getline(ss, word, ':');
            if(strcmp("image", id.c_str()) == 0 && strcmp(expected_image.c_str(), word.c_str()) != 0){
                if(vals_correct) vals_correct = false;
                auto sub_err = " image name should be " + expected_image + ", ";
                err_msg += sub_err;
            }
            else if(strcmp("resolution", id.c_str()) == 0 && expected_resolution != stof(word)){
                if(vals_correct) vals_correct = false;
                auto sub_err = " resolution should be " + std::to_string(expected_resolution) + " not " + word + ", ";
                err_msg += sub_err;
            }
            else if(strcmp("origin", id.c_str()) == 0 && strcmp(expected_origin.c_str(), word.c_str()) != 0){
                if(vals_correct) vals_correct = false;
                auto sub_err = " resolution should be " + expected_origin + " not " + word + ", ";
                err_msg += sub_err;
            }
            else if(strcmp("negate", id.c_str()) == 0 && expected_negate != stoi(word)){
                if(vals_correct) vals_correct = false;
                auto sub_err = " negate should be " + std::to_string(expected_negate) + " not " + word + ", ";
                err_msg += sub_err;
            }
            else if(strcmp("occupied_thresh", id.c_str()) == 0 && expected_occupied_thresh != stof(word)){
                if(vals_correct) vals_correct = false;
                auto sub_err = " occupied threshold should be " + std::to_string(expected_occupied_thresh) + " not " + word + ", ";
                err_msg += sub_err;
            }
            else if(strcmp("free_thresh", id.c_str()) == 0 && expected_free_thresh != stof(word)){
                if(vals_correct) vals_correct = false;
                auto sub_err = " free threshold should be " + std::to_string(expected_free_thresh) + " not " + word + ", ";
                err_msg += sub_err;
            }
        }
        if(vals_correct) return AssertionSuccess();
        else return AssertionFailure() << "failed, " << err_msg; 
    }
    else return AssertionFailure() << "failed, Could not open " << yaml_path;
}

AssertionResult checkMapObstacles(string yaml_path){
    Map extracted_map = MapData::get_map(yaml_path);
    try{
        bool boundaries_match = true;
        Map original = get_simple_map();
        if(extracted_map.px_height != original.px_height ||
        extracted_map.px_width != original.px_width){
            return AssertionFailure()<< "Dimension for the original and extracted map are not the same.";
        }
        else{
            for(int row=0; row<extracted_map.px_height; row++){
                for(int col=0; col<extracted_map.px_width; col++){
                    if(original.boundaries[row][col] != extracted_map.boundaries[row][col]){
                        cout << " failed\n";
                        boundaries_match = false;
                        break;
                    }
                }
                if(!boundaries_match) break;
            }
        }
        if(boundaries_match) return AssertionSuccess();
        else return AssertionFailure() << "One or multiples did not match the expected output";
    }
    catch(std::exception e){
        return AssertionFailure() << e.what();
    }
}

TEST_F(Data_Extraction, pgm_correctness){
    EXPECT_TRUE(checkPGM(pgm_path));
}

TEST_F(Data_Extraction, yaml_correctness){
    EXPECT_TRUE(checkYAML(yaml_path, title));
}

TEST_F(Data_Extraction, obstacle_correctness){
    EXPECT_TRUE(checkMapObstacles(yaml_path));
}