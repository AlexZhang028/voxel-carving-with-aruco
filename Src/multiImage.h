#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>

class MultiImage{
public:
    std::vector<cv::Mat> images, rvecs, tvecs;
    cv::Mat intrinsicMatrix, distCoeffs, stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors;
    double repError;

    MultiImage(std::vector<std::string> fileNames) {
        dictionaryTemp = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        dictionary = cv::Ptr<cv::aruco::Dictionary>(&dictionaryTemp);
        cv::Ptr<cv::aruco::GridBoard> board = new cv::aruco::GridBoard(cv::Size(5,7),0.1,0.01,dictionaryTemp);
        for (int i = 0; i < fileNames.size(); i++) {
            images.push_back(cv::imread(fileNames[i]));
            cv::aruco::detectMarkers(images[i], dictionary, corners, ids);
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
    }

    cv::Mat getImageWithMarker(int index) {
        cv::Mat imageCopy;
        images[index].copyTo(imageCopy);
        cv::aruco::drawDetectedMarkers(imageCopy, allCorners[index], allIds[index]);
        return imageCopy;
    }

    cv::Mat getImageWithAxes(int index) {
        cv::Mat imageCopy;
        images[index].copyTo(imageCopy);
        cv::drawFrameAxes(imageCopy,intrinsicMatrix, distCoeffs, rvecs[index], tvecs[index],0.3, 10);
        return imageCopy;
    }

private:
    cv::aruco::Dictionary dictionaryTemp;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
//    cv::Ptr<cv::aruco::GridBoard> board;
    std::vector<int> ids, markerCounterPerFrame, allIdsConcatenated;
    std::vector<std::vector<cv::Point2f>> corners, allCornersConcatenated;
    std::vector<std::vector<std::vector<cv::Point2f>>> allCorners;
    std::vector<std::vector<int>> allIds;
    int calibrationFlags = 0;
};
