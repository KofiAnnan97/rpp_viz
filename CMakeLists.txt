cmake_minimum_required(VERSION 3.5)
project("rpp_viz")
find_package(OpenCV REQUIRED)
#find_package(GTest REQUIRED)
#find_package(GTK REQUIRED gtk-4.0)
#find_package(PkgConfig)
#pkg_check_modules(GTKMM gtkmm-3.0)

set(PATH_PLANNING_LIB_PATH "./lib/path_planning/")

FILE(GLOB PATH_PLANNING_SRC
  ${PATH_PLANNING_LIB_PATH}/src/map_data.cpp
  ${PATH_PLANNING_LIB_PATH}/src/a_star.cpp
  ${PATH_PLANNING_LIB_PATH}/src/bfs.cpp
  ${PATH_PLANNING_LIB_PATH}/src/rrt_star.cpp
  ${PATH_PLANNING_LIB_PATH}/src/d_star_lite.cpp
)

include_directories(
    include
    ${PATH_PLANNING_LIB_PATH}/include
    ${PATH_PLANNING_LIB_PATH}/src
    ${OpenCV_INCLUDE_DIRS}
    ${GTKMM_INCLUDE_DIRS}
)
link_directories(${GTKMM_LIBRARY_DIRS})

add_executable(rpp_cli
    src/rpp_viz_cli.cpp
    ${PATH_PLANNING_SRC}
)
target_link_libraries(rpp_cli ${OpenCV_LIBS}) 

add_executable(run_tests
    src/run_tests.cpp
    ${PATH_PLANNING_SOURCES}
    ${PATH_PLANNING_SRC} 
)
target_link_libraries(run_tests ${OpenCV_LIBS}) #gtest gtest_main)

#add_executable(rpp_viz
#    app/main.cpp
#    app/main_window.cpp
#    app/main_window.h
#    ${PATH_PLANNING_SOURCES}
#    ${DATA_SOURCES}
#)
#target_link_libraries(rpp_viz ${GTKMM_LIBRARIES})

#add_executable(old_tests
#    tests/old_tests.cpp
#    ${PATH_PLANNING_SOURCES}
#    ${DATA_SOURCES}  
#)
#target_link_libraries(old_tests ${OpenCV_LIBS})

