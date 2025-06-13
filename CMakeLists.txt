cmake_minimum_required(VERSION 3.5)
project("rpp_viz")
find_package(OpenCV REQUIRED)
find_package(GTest REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} )
include_directories(include)

set(DATA_SOURCES
    src/map_data.cpp
    include/map_data.hpp
)

set(PATH_PLANNING_SOURCES
    src/a_star.cpp
    include/a_star.hpp
    src/bfs.cpp
    include/bfs.hpp
    src/rrt_star.cpp
    include/rrt_star.hpp
)

add_executable(rpp_cli
    src/rpp_cli.cpp
    ${PATH_PLANNING_SOURCES}
    ${DATA_SOURCES}
)
target_link_libraries(rpp_cli ${OpenCV_LIBS})

add_executable(run_tests
    tests/run_tests.cpp
    ${PATH_PLANNING_SOURCES}
    ${DATA_SOURCES}  
)
target_link_libraries(run_tests ${OpenCV_LIBS} gtest gtest_main)

