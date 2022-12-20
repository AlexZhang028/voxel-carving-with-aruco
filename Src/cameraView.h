#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>

class CameraView{
public:
    cv::Mat image, intrinsicMatrix, distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    double repError;

    // marker detection and camera calibration integrated in construction function of this class
    CameraView(std::string imagePath) {
        image = cv::imread(imagePath);
        dictionaryTemp = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        dictionary = cv::Ptr<cv::aruco::Dictionary>(&dictionaryTemp);
        board = cv::aruco::GridBoard::create(5,7,0.1,0.01,dictionaryTemp);
        cv::aruco::detectMarkers(image, dictionary, corners, ids);
        markerCounterPerFrame.reserve(1);
        markerCounterPerFrame.push_back(corners.size());
        repError = cv::aruco::calibrateCameraAruco(
                corners, ids, markerCounterPerFrame, board,
                image.size(), intrinsicMatrix, distCoeffs,
                rvecs, tvecs, calibrationFlags);
    }

    cv::Mat getImageWithMarker() {
        cv::Mat imageCopy;
        image.copyTo(imageCopy);
        cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
        return imageCopy;
    }

    cv::Mat getUndistortedImage() {
        cv::Mat imageCopy;
        image.copyTo(imageCopy);
        cv::undistort(image,imageCopy,intrinsicMatrix, distCoeffs);
        return imageCopy;
    }

private:
    cv::aruco::Dictionary dictionaryTemp;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::GridBoard> board;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> markerCounterPerFrame;
    int calibrationFlags = 0;
};
