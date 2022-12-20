#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include "cameraView.h"

int main()
{

    std::string inputDataDir = "c:/3D-Scanning/FinalProject/InputData/";
    std::string outputDataDir = "c:/3D-Scanning/FinalProject/OutputData/";
    CameraView distortionTest1(inputDataDir + "distortionTest1.png");
    CameraView distortionTest2(inputDataDir + "distortionTest2.png");
    CameraView markerBoard1(inputDataDir + "ArUcoBoard1.jpg");
    CameraView markerBoard2(inputDataDir + "ArUcoBoard2.jpg");

    // saving undistorted image with marker detection
    cv::imwrite(outputDataDir + "out_distortionTest1.png",
                distortionTest1.getImageWithMarker());
    cv::imwrite(outputDataDir + "out_distortionTest2.png",
                distortionTest2.getImageWithMarker());
    cv::imwrite(outputDataDir + "out_ArUcoBoard1.png",
                markerBoard1.getImageWithMarker());
    cv::imwrite(outputDataDir + "out_ArUcoBoard2.png",
                markerBoard2.getImageWithMarker());
    cv::imwrite(outputDataDir + "calibrated1.png",
                distortionTest1.getUndistortedImage());
    cv::imwrite(outputDataDir + "calibrated2.png",
                distortionTest2.getUndistortedImage());
    return 0;
}