cmake_minimum_required(VERSION 3.5)
project("rpp_viz")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} )
include_directories(include)

set(PATH_PLANNING_SOURCES
    src/a_star.cpp
    include/a_star.hpp
    src/bfs.cpp
    include/bfs.hpp
)

add_executable(tester
    src/tester.cpp
    ${PATH_PLANNING_SOURCES}
    src/map_data.cpp
    include/map_data.hpp  
)
target_link_libraries(tester ${OpenCV_LIBS} )