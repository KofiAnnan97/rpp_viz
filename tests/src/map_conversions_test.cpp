#include <chrono>
#include <iomanip>
#include <filesystem>
#include <cstdio>
#include <gtest/gtest.h>

#include "map_data.hpp" 
#include "time_helper.hpp"
#include "common.hpp"

/* 
Conversions (Both ways)
    Center Pixel 
    Top Left Pixel
    Top Right Pixel
    Bottom Left Pixel
    Bottom Right Pixel
*/

using namespace testing;

//typedef pair<float, float> pose;

class Conversion_Tests: public Test {
    public:
        Map m = get_simple_map();
        cell center_px, top_left_px, top_right_px, 
             bottom_left_px, bottom_right_px;
        pose2D center_pose, top_left_pose, top_right_pose, 
             bottom_left_pose, bottom_right_pose;

    protected:
        void SetUp() override {
            center_px = {m.px_width/2,m.px_height/2};
            top_left_px = {0,0};
            top_right_px = {m.px_width-1,0}; 
            bottom_left_px = {0,m.px_height-1};
            bottom_right_px = {m.px_width-1,m.px_height-1};
            center_pose = {0,0};
            top_left_pose = {m.m_width/2,-m.m_height/2};
            top_right_pose = {m.m_width/2,m.m_height/2}; 
            bottom_left_pose = {-m.m_width/2,m.m_height/2};
            bottom_right_pose = {-m.m_width/2,-m.m_height/2};
        }   
};

AssertionResult checkPixelToPose(Map map, cell actual_pixel, pose2D expected_pose){
    auto converted_pose = MapData::PIXEL2POSE(map, actual_pixel);
    if(abs(expected_pose.first-converted_pose.first)<=pose_err_thresh 
       && abs(expected_pose.second-converted_pose.second)<=pose_err_thresh)
        return AssertionSuccess();
    else
        return AssertionFailure() << "Expected: " << POSE2STR(expected_pose) 
                                  << " not " << POSE2STR(converted_pose);
}

AssertionResult checkPoseToPixel(Map map, pose2D actual_pose, cell expected_pixel){
    auto converted_pixel = MapData::POSE2D2PIXEL(map, actual_pose);
    if(abs(expected_pixel.first-converted_pixel.first)<=px_err_thresh 
           && abs(expected_pixel.second-converted_pixel.second)<=px_err_thresh)
        return AssertionSuccess();
    else
        return AssertionFailure() << "Expected: " << CELL2STR(expected_pixel) 
                                  << " not " << CELL2STR(converted_pixel);
}

/*
    Based on how the map to pose conversion is implemented the should be a 90 shift clockwise and 
    counter-clockwise from map to pose and pose to map respectively.
*/
    
TEST_F(Conversion_Tests, center_pixel_to_pose){
    EXPECT_TRUE(checkPixelToPose(m, center_px, center_pose));
}

TEST_F(Conversion_Tests, center_pose_to_pixel){
    EXPECT_TRUE(checkPoseToPixel(m, center_pose, center_px));
}

TEST_F(Conversion_Tests, top_left_pixel_to_pose){
    EXPECT_TRUE(checkPixelToPose(m, top_left_px, top_right_pose));
}

TEST_F(Conversion_Tests, top_left_pose_to_pixel){
    EXPECT_TRUE(checkPoseToPixel(m, top_left_pose, bottom_left_px));
}

TEST_F(Conversion_Tests, top_right_pixel_to_pose){
    EXPECT_TRUE(checkPixelToPose(m, top_right_px, bottom_right_pose));
}

TEST_F(Conversion_Tests, top_right_pose_to_pixel){
    EXPECT_TRUE(checkPoseToPixel(m, top_right_pose, top_left_px));
}

TEST_F(Conversion_Tests,  bottom_left_pixel_to_pose){
    EXPECT_TRUE(checkPixelToPose(m, bottom_left_px, top_left_pose));
}

TEST_F(Conversion_Tests, bottom_left_pose_to_pixel){
    EXPECT_TRUE(checkPoseToPixel(m, bottom_left_pose, bottom_right_px));
}

TEST_F(Conversion_Tests, bottom_right_pixel_to_pose){
    EXPECT_TRUE(checkPixelToPose(m, bottom_right_px, bottom_left_pose));
}

TEST_F(Conversion_Tests, bottom_right_pose_to_pixel){
    EXPECT_TRUE(checkPoseToPixel(m, bottom_right_pose, top_right_px));
}