#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include "singleImage.h"
#include "multiImage.h"
#include "voxel.h"
#include "Eigen.h"

// read list.txt to get file name of input images
void readTXT(std::string inputDataDir, std::vector<std::string> &fileNames) {
    std::ifstream input(inputDataDir + "list.txt");
    if (!input.is_open()) {
        std::cout << "Error: error when reading 'list.txt'" << std::endl;
        throw -1;
    } else {
        std::string line;
        while (!input.eof()) {
            std::getline(input, line);
            fileNames.push_back(inputDataDir + line);
        }
    }
}

// save multiple images in specific directory
void saveImages(std::string dstDir, std::vector<cv::Mat> images) {
    std::string fileName;
    for (int i = 0; i < images.size(); i++) {
        if (i < 9) {
            fileName = "0" + std::to_string(i+1) + ".png";
        } else {
            fileName = std::to_string(i+1) + ".png";
        }
        cv::imwrite(dstDir + fileName, images[i]);
    }
}

cv::Mat rgbRange(cv::Mat image, int black_max, int white_min) {
    cv::Mat blackMask, whiteMask, mask;
    cv::Scalar blackMin(0, 0, 0);
    cv::Scalar blackMax(black_max, black_max, black_max);
    cv::Scalar whiteMin(white_min, white_min, white_min);
    cv::Scalar whiteMax(255, 255, 255);
    cv::inRange(image, blackMin, blackMax, blackMask);
    cv::inRange(image, whiteMin, whiteMax, whiteMask);
    cv::bitwise_or(blackMask, whiteMask, mask);
    cv::bitwise_not(mask, mask);
    return mask;
}

int main()
{
    // define file path
    std::string inputDataDir = "e:/documents/3D-Scanning/InputData/05/";
    std::string outputDataDir = "e:/documents/3D-Scanning/OutputData/05/";
    // read image path
    std::vector<std::string> fileNames;
    readTXT(inputDataDir, fileNames);
    // create MultiImage class, get all information
    cv::Scalar hsv_min(44,95,40);
    cv::Scalar hsv_max(57,203,255);
    int erodeIter = 0;
    int dilateIter = 2;
    MultiImage images(fileNames, erodeIter, dilateIter, hsv_min, hsv_max, 25, 245, true);
//    cv::imshow("01",images.silhouettes[0]);
//    cv::waitKey(0);
    saveImages(outputDataDir,images.silhouettes);
//    images.extractForeground();
//    saveImages(outputDataDir+"foreground/iter3/", images.foregrounds);
    // define bounding box

//    bounding box for stone
    Eigen::Vector3d startPoint(0.08,0.11,-0.065);
    Eigen::Vector3d endPoint(0.13,0.16,0);
    int n = 500;
    Voxel space(n,n,n, startPoint, endPoint, images.images, images.silhouettes, images.p_Matrices);
    space.carve(0);
    space.colorRender(3);
    space.writeCenterPoints(outputDataDir + "test3.off", true);
    space.writeMesh(outputDataDir + "iter_v_1b.off", true);
    std::cout<<space.n_pointWithColor<<" "<<space.n_vertices<<" "<<space.n_faces<<std::endl;


//    cv::Mat image = cv::imread("e:/documents/3D-Scanning/InputData/01/IMG_1398.JPG");
//    cv::resize(image,image,cv::Size(960,720));
//    cv::Mat mask = rgbRange(image,70,245);
//    cv::Mat mask2;
//    cv::inRange(image,cv::Scalar(0,155,0), cv::Scalar(255,255,255),mask2);
//    cv::bitwise_not(mask2,mask2);
//    cv::bitwise_and(mask, mask2, mask2);
//    cv::imshow("mask", mask2);
//    cv::waitKey(0);

    return 0;
}