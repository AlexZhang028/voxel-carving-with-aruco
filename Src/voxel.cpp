#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include "Eigen.h"
#include <iostream>
#include <fstream>
#include "voxel.h"

void readTXT(std::string fileName, std::vector<Eigen::Vector3d> &output){
    std::ifstream input(fileName);
    if (!input.is_open()) {
        std::cout << "Read error" << std::endl;
    } else {
        std::string line;
        double x, y, z;
        Eigen::Vector3d point;
        while(!input.eof()){
            std::getline(input, line);
            std::stringstream s(line);
            s>>x>>y>>z;
            point << x, y, z;
            output.push_back(point);
        }
    }
}

int main(){
//    std::vector<Eigen::Vector3d> centerPoints;
//    Voxel test2(centerPoints,0.124324, 0.201723, 0.201557);
//    test2.writeMesh();
    std::vector<Eigen::Vector3d> centerPoints;
    readTXT("E:/Documents/3D-Scanning/InputData/01.txt", centerPoints);
    Voxel test2(centerPoints, 0.124324, 0.201723, 0.201557);
    test2.writeMesh("E:/Documents/3D-Scanning/OutputData/01.off");
    return 0;
};

