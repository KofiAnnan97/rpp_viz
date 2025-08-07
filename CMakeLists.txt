cmake_minimum_required(VERSION 3.16)

project(rpp_viz VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets LinguistTools)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Widgets LinguistTools Svg)

set(PATH_PLANNING_LIB_PATH "./lib/path_planning")
set(TOOLS_LIB_PATH "./lib/tools")
set(APP_PATH "./app")

FILE(GLOB PATH_PLANNING_LIB
    ${PATH_PLANNING_LIB_PATH}/include/*.hpp
    ${PATH_PLANNING_LIB_PATH}/src/*.cpp
)

FILE(GLOB TOOLS_LIB
    ${TOOLS_LIB_PATH}/include/*.hpp
    ${TOOLS_LIB_PATH}/src/*.cpp
)

FILE(GLOB APP_CODE
    ${APP_PATH}/include/*.h
    ${APP_PATH}/src/*.cpp
)

include_directories(
    ${APP_PATH}/include/
    ${APP_PATH}/src/
    ${PATH_PLANNING_LIB_PATH}/include
    ${PATH_PLANNING_LIB_PATH}/src
    ${TOOLS_LIB_PATH}/include
    ${TOOLS_LIB_PATH}/src
    ${OpenCV_INCLUDE_DIRS}
)

set(TS_FILES app/rpp_viz_en_US.ts)

set(PROJECT_SOURCES
    ${APP_CODE}
    ${APP_PATH}/mainwindow.ui
    ${PATH_PLANNING_LIB}
    ${TOOLS_LIB}
    ${APP_PATH}/icons.qrc
    ${TS_FILES}
)

#add_executable(old_tests
#    scripts/old_tests.cpp
#    ${PATH_PLANNING_LIB}
#)
#target_link_libraries(old_tests ${OpenCV_LIBS})

# Map Manipulation and Algorithm Testing
add_executable(run_tests
    scripts/run_tests.cpp
    ${PATH_PLANNING_LIB} 
)
target_link_libraries(run_tests ${OpenCV_LIBS}) #gtest gtest_main)

# Command line implementation
add_executable(rpp_cli
    scripts/rpp_viz_cli.cpp
    ${PATH_PLANNING_LIB}
    ${TOOLS_LIB}
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
target_link_libraries(rpp_viz PRIVATE Qt6::Svg)

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
if(${QT_VERSION} VERSION_LESS 6.1.0)
  set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.kofiannan97.rpp_viz)
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
