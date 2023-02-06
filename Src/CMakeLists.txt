cmake_minimum_required(VERSION 3.0.0)
project(ArMarker VERSION 0.1.0)
set (OpenCV_DIR "../Libs/opencv/")
set(Eigen3_DIR "../Libs/Eigen/share/eigen3/cmake" CACHE PATH "Path to installed Eigen")
find_package(OpenCV REQUIRED )
find_package(Eigen3 REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
include(CTest)
enable_testing()
set(HEADERS
        Eigen.h
        singleImage.h
        multiImage.h
        voxel.h
        )

set(SOURCES
        main.cpp
        )

add_executable(ArUco ${HEADERS} main.cpp)

target_link_libraries(ArUco ${OpenCV_LIBS})
target_link_libraries(ArUco Eigen3::Eigen)
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
set(LIBRARY_OUTPUT_PATH "${CMAKE_BINARY_DIR}")
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}")
include(CPack)
