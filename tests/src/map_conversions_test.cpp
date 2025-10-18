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

AssertionResult checkPixelToPose(Map map, cell actual_pixel, cell expected_pose){
    auto converted_pose = MapData::PIXEL2POSE(map, {actual_pixel.first, actual_pixel.second});
    if(abs(expected_pose.first-converted_pose.first)<=pose_err_thresh 
       && abs(expected_pose.second-converted_pose.second)<=pose_err_thresh)
        return AssertionSuccess();
    else
        return AssertionFailure() << "Expected: " << POSE2STR(expected_pose) 
                                  << " not " << POSE2STR(converted_pose);
}

AssertionResult checkPoseToPixel(Map map, cell actual_pose, cell expected_pixel){
    auto converted_pixel = MapData::POSE2PIXEL(map, actual_pose.first, actual_pose.second);
    if(abs(expected_pixel.first-converted_pixel.first)<=px_err_thresh 
           && abs(expected_pixel.second-converted_pixel.second)<=px_err_thresh)
        return AssertionSuccess();
    else
        return AssertionFailure() << "Expected: " << CELL2STR(expected_pixel) 
                                  << " not " << CELL2STR(converted_pixel);
}
    
TEST(Map_Pose_Conversion, center_pixel_to_pose){
    auto m = get_simple_map();
    cell actual_pixel = {m.px_width/2,m.px_height/2};
    std::pair<float,float> expected_pose = {0,0};
    EXPECT_TRUE(checkPixelToPose(m, actual_pixel, expected_pose));
}

TEST(Map_Pose_Conversion, center_pose_to_pixel){
    auto m = get_simple_map();
    cell expected_pixel = {m.px_width/2,m.px_height/2};
    std::pair<float,float> actual_pose = {0,0};
    EXPECT_TRUE(checkPoseToPixel(m, actual_pose, expected_pixel));
}

TEST(Map_Pose_Conversion, top_left_pixel_to_pose){
    auto m = get_simple_map();
    cell actual_pixel = {0,0};
    std::pair<float,float> expected_pose = {m.m_width/2,-m.m_height/2};
    EXPECT_TRUE(checkPixelToPose(m, actual_pixel, expected_pose));
}

TEST(Map_Pose_Conversion, top_left_pose_to_pixel){
    auto m = get_simple_map();
    cell expected_pixel = {0,0};
    std::pair<float,float> actual_pose = {m.m_width/2,-m.m_height/2};
    EXPECT_TRUE(checkPoseToPixel(m, actual_pose, expected_pixel));
}

TEST(Map_Pose_Conversion, top_right_pixel_to_pose){
    auto m = get_simple_map();
    cell actual_pixel = {m.px_width-1,0};
    std::pair<float,float> expected_pose = {m.m_width/2,m.m_height/2};
    EXPECT_TRUE(checkPixelToPose(m, actual_pixel, expected_pose));
}

TEST(Map_Pose_Conversion, top_right_pose_to_pixel){
    auto m = get_simple_map();
    cell expected_pixel = {m.px_width-1,0};
    std::pair<float,float> actual_pose = {m.m_width/2,m.m_height/2};
    EXPECT_TRUE(checkPoseToPixel(m, actual_pose, expected_pixel));
}

TEST(Map_Pose_Conversion, bottom_left_pixel_to_pose){
    auto m = get_simple_map();
    cell actual_pixel = {0,m.px_height-1};
    std::pair<float,float> expected_pose = {-m.m_width/2,m.m_height/2};
    EXPECT_TRUE(checkPixelToPose(m, actual_pixel, expected_pose));
}

TEST(Map_Pose_Conversion, bottom_left_pose_to_pixel){
    auto m = get_simple_map();
    cell expected_pixel = {0,m.px_height-1};
    std::pair<float,float> actual_pose = {-m.m_width/2,m.m_height/2};
    EXPECT_TRUE(checkPoseToPixel(m, actual_pose, expected_pixel));
}

TEST(Map_Pose_Conversion, bottom_right_pixel_to_pose){
    auto m = get_simple_map();
    cell actual_pixel = {m.px_width-1,m.px_height-1};
    std::pair<float,float> expected_pose = {-m.m_width/2,-m.m_height/2};
    EXPECT_TRUE(checkPixelToPose(m, actual_pixel, expected_pose));
}

TEST(Map_Pose_Conversion, bottom_right_pose_to_pixel){
    auto m = get_simple_map();
    cell expected_pixel = {m.px_width-1,m.px_height-1};
    std::pair<float,float> actual_pose = {-m.m_width/2,-m.m_height/2};
    EXPECT_TRUE(checkPoseToPixel(m, actual_pose, expected_pixel));
}