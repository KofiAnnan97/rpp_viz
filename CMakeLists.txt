cmake_minimum_required(VERSION 3.16)

project(rpp_viz VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets LinguistTools)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets LinguistTools)

if( CMAKE_RUNTIME_OUTPUT_DIRECTORY )
	set( executableOutputPath ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} )
elseif( EXECUTABLE_OUTPUT_PATH )
	set( executableOutputPath ${EXECUTABLE_OUTPUT_PATH} )
else()
	set( executableOutputPath ${CMAKE_CURRENT_BINARY_DIR} )
endif()

set(PATH_PLANNING_LIB_PATH "./lib/path_planning/")

FILE(GLOB PATH_PLANNING_SRC
  ${PATH_PLANNING_LIB_PATH}/src/map_data.cpp
  ${PATH_PLANNING_LIB_PATH}/src/a_star.cpp
  ${PATH_PLANNING_LIB_PATH}/src/bfs.cpp
  ${PATH_PLANNING_LIB_PATH}/src/rrt_star.cpp
  #${PATH_PLANNING_LIB_PATH}/src/d_star_lite.cpp
)

include_directories(
    include
    ${PATH_PLANNING_LIB_PATH}/include
    ${PATH_PLANNING_LIB_PATH}/src
    ${OpenCV_INCLUDE_DIRS}
)

set(TS_FILES rpp_viz_en_US.ts)

set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
        ${TS_FILES}
        ${PATH_PLANNING_SRC}
)

#add_executable(old_tests
#    script/old_tests.cpp
#    ${PATH_PLANNING_SRC}
#)
#target_link_libraries(old_tests ${OpenCV_LIBS})

# Map Manipulation and Algorithm Testing
add_executable(run_tests
    scripts/run_tests.cpp
    ${PATH_PLANNING_SRC} 
)
target_link_libraries(run_tests ${OpenCV_LIBS}) #gtest gtest_main)

# Command line implementation
add_executable(rpp_cli
    scripts/rpp_viz_cli.cpp
    ${PATH_PLANNING_SRC}
)
target_link_libraries(rpp_cli ${OpenCV_LIBS})

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(rpp_viz
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET test APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation

    qt_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})
else()
    if(ANDROID)
        add_library(rpp_viz SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(rpp_viz
            ${PROJECT_SOURCES}
        )
    endif()

    qt5_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})
endif()

target_link_libraries(rpp_viz PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)
target_include_directories(rpp_viz PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rpp_viz PRIVATE ${OpenCV_LIBS})

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
if(${QT_VERSION} VERSION_LESS 6.1.0)
  set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.rpp_viz)
endif()
set_target_properties(rpp_viz PROPERTIES
    ${BUNDLE_ID_OPTION}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

include(GNUInstallDirs)
install(TARGETS rpp_viz
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(rpp_viz)
endif()
