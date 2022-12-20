#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include "singleImage.h"
#include "multiImage.h"

int main()
{

    std::string inputDataDir = "c:/3D-Scanning/FinalProject/InputData/";
    std::string outputDataDir = "c:/3D-Scanning/FinalProject/OutputData/";
    std::vector<std::string> fileNames;
    for (int i = 0; i <= 2; i++){
        fileNames.push_back(inputDataDir + std::to_string(i + 1) + ".jpg");
    }
    MultiImage test(fileNames);
    for (size_t i = 0; i < fileNames.size(); i++) {
        cv::namedWindow(std::to_string(i), cv::WINDOW_KEEPRATIO);
        cv::imshow(std::to_string(i), test.getImageWithAxes(i));
        cv::imwrite(outputDataDir + std::to_string(i + 1) + ".png", test.getImageWithAxes(i));
    }
    SingleImage testSingle(inputDataDir + "1.jpg");
    cv::imwrite(outputDataDir + "single_1.png", testSingle.getImageWithAxes());
    cv::waitKey(0);
    return 0;
}