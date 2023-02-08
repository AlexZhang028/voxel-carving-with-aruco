#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>

class MultiImage{
public:
    std::vector<cv::Mat> images, cameraPoses, silhouettes, foregrounds, p_Matrices;

    // Marker detection, camera calibration while creating the class
    MultiImage(std::vector<std::string> fileNames, int erodeIter, int dilateIter, cv::Scalar hsv_min, cv::Scalar hsv_max, int black_max, int white_min, bool flipHSVMask) {
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        board = new cv::aruco::GridBoard(cv::Size(5,7),0.035,0.005,dictionary);
        cv::aruco::ArucoDetector detector(dictionary, detectorParams);
        for (int i = 0; i < fileNames.size(); i++) {
            images.push_back(cv::imread(fileNames[i]));
            detector.detectMarkers(images[i], corners, ids);
            allCorners.push_back(corners);
            allIds.push_back(ids);
        }
        for (int i = 0; i < allCorners.size(); i++) {
            markerCounterPerFrame.push_back(allIds[i].size());
            for (int j = 0; j < allCorners[i].size(); j++) {
                allCornersConcatenated.push_back(allCorners[i][j]);
                allIdsConcatenated.push_back(allIds[i][j]);
            }
        }
        repError = cv::aruco::calibrateCameraAruco(
                allCornersConcatenated, allIdsConcatenated, markerCounterPerFrame, board,
                images[0].size(), intrinsicMatrix, distCoeffs,
                rvecs, tvecs,stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors, calibrationFlags);
        getProjectionMatrices();
        getSilhouettes(erodeIter, dilateIter, hsv_min, hsv_max, black_max, white_min, flipHSVMask);
    }

    cv::Mat getImageWithMarker(int index) {
        cv::Mat imageCopy;
        images[index].copyTo(imageCopy);
        cv::aruco::drawDetectedMarkers(imageCopy, allCorners[index], allIds[index]);
        return imageCopy;
    }

    void drawArUcoBoard(cv::Mat boardImage) {
        board->generateImage( cv::Size(1400, 1900), boardImage, 10, 1 );
    }

    cv::Mat getImageWithAxes(int index) {
        cv::Mat imageCopy;
        images[index].copyTo(imageCopy);
        cv::drawFrameAxes(imageCopy,intrinsicMatrix, distCoeffs, rvecs[index], tvecs[index],0.3, 10);
        return imageCopy;
    }

    void extractForeground() {
        for (int i = 0; i < images.size(); i++) {
            cv::Mat foreground;
            images[i].copyTo(foreground,silhouettes[i]);
            foregrounds.push_back(foreground);
        }
    }

private:
    cv::Ptr<cv::aruco::GridBoard> board;
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    std::vector<int> ids, markerCounterPerFrame, allIdsConcatenated;
    std::vector<std::vector<cv::Point2f>> corners, allCornersConcatenated;
    std::vector<std::vector<std::vector<cv::Point2f>>> allCorners;
    std::vector<std::vector<int>> allIds;
    std::vector<cv::Mat> rvecs, tvecs;
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::Mat intrinsicMatrix, distCoeffs, stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors;
    double repError;
    int calibrationFlags = 0;

    void getProjectionMatrices() {
        for (int i = 0; i < images.size(); i++){
            cv::Mat rt_Matrix, cameraPose, p_Matrix;
            cv::Mat lastLine = (cv::Mat_<double>(1,4) << 0, 0, 0, 1);
            cv::Mat rotationMatrix;
            cv::Rodrigues(rvecs[i], rotationMatrix);
            cv::hconcat(rotationMatrix, tvecs[i], rt_Matrix);
            cv::vconcat(rt_Matrix,lastLine, cameraPose);
            p_Matrix = intrinsicMatrix * rt_Matrix;
            cameraPoses.push_back(cameraPose);
            p_Matrices.push_back(p_Matrix);
        }
    }

    void getSilhouettes(int erodeIter, int dilateIter, cv::Scalar hsv_min, cv::Scalar hsv_max, int black_max, int white_min, bool flipHSVMask) {
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
        for (int i = 0; i < images.size(); i++) {
            cv::Mat hsv, erode, silhouette, mask;
            cv::cvtColor(images[i], hsv, cv::COLOR_BGR2HSV);
            cv::inRange(hsv, hsv_min, hsv_max,mask);
            if (flipHSVMask) cv::bitwise_not(mask, mask);
            cv::Mat bwMask = rgbRange(images[i], black_max, white_min);
            cv::bitwise_and(mask, bwMask, mask);
            cv::erode(mask,erode, kernel, cv::Point(-1,-1),erodeIter);
            cv::dilate(erode,silhouette, kernel, cv::Point(-1,-1),dilateIter);
            silhouettes.push_back(silhouette);
        }
    }

    cv::Mat rgbRange(cv::Mat image, int black_max, int white_min) {
        cv::Mat blackMask, whiteMask, greenMask, mask;
        cv::Scalar blackMin(0, 0, 0);
        cv::Scalar blackMax(black_max, black_max, black_max);
        cv::Scalar whiteMin(white_min, white_min, white_min);
        cv::Scalar whiteMax(255, 255, 255);
        cv::inRange(image, blackMin, blackMax,blackMask);
        cv::inRange(image, whiteMin, whiteMax, whiteMask);
        cv::bitwise_or(blackMask,whiteMask, mask);
//        cv::inRange(image,cv::Scalar(0,180,0), cv::Scalar(255,255,255),greenMask);
//        cv::bitwise_or(mask, greenMask, mask);
        cv::bitwise_not(mask, mask);
        return mask;
    }
};
