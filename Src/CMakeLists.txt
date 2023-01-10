cmake_minimum_required(VERSION 3.0.0)
project(ArMarker VERSION 0.1.0)
set (OpenCV_DIR "../Libs/opencv/")
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
include(CTest)
enable_testing()
set(HEADERS
        singleImage.h
        multiImage.h)

set(SOURCES
        main.cpp
        foreground.cpp)

add_executable(ArMarker ${HEADERS} main.cpp)
add_executable(foreground ${HEADERS} foreground.cpp)

target_link_libraries(ArMarker ${OpenCV_LIBS})
target_link_libraries(foreground ${OpenCV_LIBS})
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
set(LIBRARY_OUTPUT_PATH "${CMAKE_BINARY_DIR}")
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}")
include(CPack)
