#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include "Eigen.h"
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include <fstream>

class Voxel{
public:
    int n_pointWithColor = 0;
    int n_centerPoints = 0;
    int n_vertices = 0;
    int n_faces = 0;

    Voxel(std::vector<Eigen::Vector3d> centerPoints_input, double size_x_input, double size_y_input, double size_z_input) {
        size_x = size_x_input;
        size_y = size_y_input;
        size_z = size_z_input;
        centerPoints = centerPoints_input;
    }
    // n_x: number of voxels in x direction, size_x: size of a single voxel in x direction (length)
    Voxel(int n_x, int n_y, int n_z, Eigen::Vector3d startPoint, Eigen::Vector3d endPoint) {
        size_x = (endPoint(0) - startPoint(0)) / n_x;
        size_y = (endPoint(1) - startPoint(1)) / n_y;
        size_z = (endPoint(2) - startPoint(2)) / n_z;
        for (int x_idx = 0; x_idx < n_x; x_idx++){
            for (int y_idx = 0; y_idx < n_y; y_idx++){
                for (int z_idx = 0; z_idx < n_z; z_idx++){
                    Eigen::Vector3d centerPoint(x_idx * size_x + startPoint(0), y_idx * size_y + startPoint(1), z_idx * size_z + startPoint(2));
                    centerPoints.push_back(centerPoint);
                }
            }
        }
    }

    void writeCenterPoints(std::string fileName, bool color) {
        std::ofstream outFile(fileName);
        outFile << "COFF" << std::endl;

        outFile << "# numVertices numFaces numEdges" << std::endl;



        std::cout<<"writing center points"<<std::endl;

        if (color) {
            outFile << n_pointWithColor << " " << "0" << " 0" << std::endl;

            for (int idx = 0; idx < n_centerPoints; idx ++){
                if (pointColor[idx](3) != 0){
                    outFile << centerPoints[idx][0] << " " << centerPoints[idx][1] << " " << centerPoints[idx][2] << " "
                            << pointColor[idx](0)<<" "<<pointColor[idx](1)<<" "<<pointColor[idx](2)<<" "<<pointColor[idx](3)<<std::endl;
                }
            }
        } else {
            outFile << n_centerPoints << " " << "0" << " 0" << std::endl;

            for (int idx = 0; idx < n_centerPoints; idx ++){
                outFile << centerPoints[idx][0] << " " << centerPoints[idx][1] << " " << centerPoints[idx][2] << std::endl;
            }
        }

        outFile.close();
    }


    void writeVertices(std::string fileName) {
        getVertices(false);
        std::ofstream outFile(fileName);
        outFile << "COFF" << std::endl;

        outFile << "# numVertices numFaces numEdges" << std::endl;

        outFile << n_vertices << " " << "0" << " 0" << std::endl;

        std::cout<<"writing Vertices"<<std::endl;

        for (int i = 0; i < allVertices.size(); i ++){
            for (int j = 0; j < allVertices[i].size(); j ++){
                outFile << allVertices[i][j][0] << " " << allVertices[i][j][1] << " " << allVertices[i][j][2] << std::endl;
            }
        }
        outFile.close();
    }

    void writeMesh(std::string fileName, bool color) {
        getVertices(color);
        getFaces();
        std::ofstream outFile(fileName);
        std::cout << "Writing vertices" << std::endl;
        outFile << "COFF" << std::endl;

        outFile << "# numVertices numFaces numEdges" << std::endl;
        outFile << n_vertices << " " << n_faces << " 0" << std::endl;
        if (color){
            for (int i = 0; i < allVertices.size(); i ++){
                for (int j = 0; j < allVertices[i].size(); j ++){
                    if (pointColor[i](3) != 0){
                        outFile << allVertices[i][j][0] << " " << allVertices[i][j][1] << " " << allVertices[i][j][2]<< " "
                                << pointColor[i][0]<<" "<<pointColor[i][1]<<" "<<pointColor[i][2]<<" "<<pointColor[i][3]<<std::endl;
                    }
                }
            }
        } else {
            for (int i = 0; i < allVertices.size(); i ++){
                for (int j = 0; j < allVertices[i].size(); j ++){
                    outFile << allVertices[i][j][0] << " " << allVertices[i][j][1] << " " << allVertices[i][j][2]<<std::endl;
                }
            }
        }

        std::cout << "Writing faces" << std::endl;
        outFile << "# list of faces" << std::endl;
        outFile << "# nVerticesPerFace idx0 idx1 idx2 ..." << std::endl;
        for (int idx = 0; idx < n_faces; idx++) {
            outFile << faces[idx] << std::endl;
        }
        outFile.close();
    }

    void carve(std::vector<cv::Mat> silhouettes, std::vector<cv::Mat> projectionMatrices) {
        std::vector<bool> centerPointFlags(centerPoints.size(), true);
        for (int imageIdx = 0; imageIdx < silhouettes.size(); imageIdx++) {
            std::cout<<"Carving: processing image ["<<imageIdx + 1<<"/"<<silhouettes.size()<<"]"<<std::endl;
            for (int pointIdx = 0; pointIdx < centerPoints.size(); pointIdx++) {
                // perform only to points that are retained from last iteration to save resource
                if (centerPointFlags[pointIdx]) {
                    // mark all points with projection outside silhouette
                    Eigen::Vector2i imagePoint = projection(projectionMatrices[imageIdx], centerPoints[pointIdx]);
                    centerPointFlags[pointIdx] = inSilhouette(imagePoint, silhouettes[imageIdx]);
                }
            }
        }
        std::vector<Eigen::Vector3d> remainedPoints;

        for (int i = 0; i< centerPoints.size(); i++) {
            if (centerPointFlags[i]) {
                remainedPoints.push_back(centerPoints[i]);
            }
        }
        centerPoints = remainedPoints;
        n_centerPoints = centerPoints.size();
    }

    void colorRender(std::vector<cv::Mat> projectionMatrices, std::vector<cv::Mat> images, int threshold) {
        std::cout<<"start retrieving color information"<<std::endl;
        Eigen::Vector4i colorInitial(0,0,0,0);
        std::vector<Eigen::Vector4i> colorTemp(centerPoints.size(), colorInitial);
        for (int imageIdx = 0; imageIdx < images.size(); imageIdx++) {
            int nextImageIdx = imageIdx + 1;
            if (nextImageIdx == images.size()) {
                nextImageIdx = 0;
            }
            for (int pointIdx = 0; pointIdx < centerPoints.size(); pointIdx++) {
                Eigen::Vector2i imagePoint = projection(projectionMatrices[imageIdx], centerPoints[pointIdx]);
                Eigen::Vector2i nextImagePoint = projection(projectionMatrices[nextImageIdx], centerPoints[pointIdx]);
                if (colorConsistency(images, imagePoint, nextImagePoint, imageIdx, nextImageIdx, threshold) && colorTemp[pointIdx](3) == 0 ) {
                    int B = images[imageIdx].at<cv::Vec3b>(imagePoint(1), imagePoint(0))[0];
                    int G = images[imageIdx].at<cv::Vec3b>(imagePoint(1), imagePoint(0))[1];
                    int R = images[imageIdx].at<cv::Vec3b>(imagePoint(1), imagePoint(0))[2];
                    int A = 255;

                    Eigen::Vector4i color(R, G, B, A);
                    colorTemp[pointIdx] = color;
                }
            }
        }
        for (int i = 0; i < colorTemp.size(); i++) {
            if (colorTemp[i](3) != 0) {
                pointWithColor.push_back(centerPoints[i]);
                pointColor.push_back(colorTemp[i]);
            }
        }
        n_pointWithColor = pointWithColor.size();
    }


private:
    std::vector<Vector4i> pointColor;
    double size_x, size_y, size_z;
    std::vector<Eigen::Vector3d> centerPoints, pointWithColor;
    std::vector<std::vector<Eigen::Vector3d>> allVertices;
    std::vector<std::string> faces;

    int distance(std::vector<cv::Mat> images, Eigen::Vector2i imagePoint1, Eigen::Vector2i imagePoint2, int imageIdx1, int imageIdx2)
    {
        int distanceB = images[imageIdx1].at<cv::Vec3b>(imagePoint1(1),imagePoint1(0))[0] - images[imageIdx2].at<cv::Vec3b>(imagePoint2(1),imagePoint2(0))[0];
        int distanceG = images[imageIdx1].at<cv::Vec3b>(imagePoint1(1),imagePoint1(0))[1] - images[imageIdx2].at<cv::Vec3b>(imagePoint2(1),imagePoint2(0))[1];
        int distanceR = images[imageIdx1].at<cv::Vec3b>(imagePoint1(1),imagePoint1(0))[2] - images[imageIdx2].at<cv::Vec3b>(imagePoint2(1),imagePoint2(0))[2];
        int distance= distanceB * distanceB + distanceG * distanceG + distanceR * distanceR;
        return distance;
    }

    bool colorConsistency(std::vector<cv::Mat> images ,Eigen::Vector2i imagePoint1, Eigen::Vector2i imagePoint2, int imageIdx1, int imageIdx2, int threshold) {
        if (distance(images, imagePoint1, imagePoint2, imageIdx1, imageIdx2) <= threshold){
            return true;
        }
        return false;
    }

    void getFaces(){
        for (int i = 0; i < allVertices.size(); i++) {
            int idx0 = i * 8;
            int idx1 = idx0 + 1;
            int idx2 = idx1 + 1;
            int idx3 = idx2 + 1;
            int idx4 = idx3 + 1;
            int idx5 = idx4 + 1;
            int idx6 = idx5 + 1;
            int idx7 = idx6 + 1;
            faces.push_back(
                    "4 " +
                    std::to_string(idx0) + " " +
                    std::to_string(idx1) + " " +
                    std::to_string(idx2) + " " +
                    std::to_string(idx3) + " ");
            faces.push_back(
                    "4 " +
                    std::to_string(idx0) + " " +
                    std::to_string(idx1) + " " +
                    std::to_string(idx5) + " " +
                    std::to_string(idx4) + " ");
            faces.push_back(
                    "4 " +
                    std::to_string(idx0) + " " +
                    std::to_string(idx3) + " " +
                    std::to_string(idx7) + " " +
                    std::to_string(idx4) + " ");
            faces.push_back(
                    "4 " +
                    std::to_string(idx2) + " " +
                    std::to_string(idx1) + " " +
                    std::to_string(idx5) + " " +
                    std::to_string(idx6) + " ");
            faces.push_back(
                    "4 " +
                    std::to_string(idx2) + " " +
                    std::to_string(idx3) + " " +
                    std::to_string(idx7) + " " +
                    std::to_string(idx6) + " ");
            faces.push_back(
                    "4 " +
                    std::to_string(idx4) + " " +
                    std::to_string(idx5) + " " +
                    std::to_string(idx6) + " " +
                    std::to_string(idx7) + " ");
        }
        n_faces = faces.size();
    }

    void getVertices(bool color) {
        std::vector<Eigen::Vector3d> vertices;
        if (color) {
            for (int i = 0; i < pointWithColor.size(); i++) {
                vertices.clear();
                vertices.push_back(pointWithColor[i] + Eigen::Vector3d(-size_x/2,-size_y/2,-size_z/2));
                vertices.push_back(pointWithColor[i] + Eigen::Vector3d(+size_x/2,-size_y/2,-size_z/2));
                vertices.push_back(pointWithColor[i] + Eigen::Vector3d(+size_x/2,+size_y/2,-size_z/2));
                vertices.push_back(pointWithColor[i] + Eigen::Vector3d(-size_x/2,+size_y/2,-size_z/2));
                vertices.push_back(pointWithColor[i] + Eigen::Vector3d(-size_x/2,-size_y/2,+size_z/2));
                vertices.push_back(pointWithColor[i] + Eigen::Vector3d(+size_x/2,-size_y/2,+size_z/2));
                vertices.push_back(pointWithColor[i] + Eigen::Vector3d(+size_x/2,+size_y/2,+size_z/2));
                vertices.push_back(pointWithColor[i] + Eigen::Vector3d(-size_x/2,+size_y/2,+size_z/2));
                allVertices.push_back(vertices);
            }
            n_vertices = allVertices.size() * 8;
        } else {
            for (int i = 0; i < centerPoints.size(); i++) {
                vertices.clear();
                vertices.push_back(centerPoints[i] + Eigen::Vector3d(-size_x/2,-size_y/2,-size_z/2));
                vertices.push_back(centerPoints[i] + Eigen::Vector3d(+size_x/2,-size_y/2,-size_z/2));
                vertices.push_back(centerPoints[i] + Eigen::Vector3d(+size_x/2,+size_y/2,-size_z/2));
                vertices.push_back(centerPoints[i] + Eigen::Vector3d(-size_x/2,+size_y/2,-size_z/2));
                vertices.push_back(centerPoints[i] + Eigen::Vector3d(-size_x/2,-size_y/2,+size_z/2));
                vertices.push_back(centerPoints[i] + Eigen::Vector3d(+size_x/2,-size_y/2,+size_z/2));
                vertices.push_back(centerPoints[i] + Eigen::Vector3d(+size_x/2,+size_y/2,+size_z/2));
                vertices.push_back(centerPoints[i] + Eigen::Vector3d(-size_x/2,+size_y/2,+size_z/2));
                allVertices.push_back(vertices);
            }
            n_vertices = allVertices.size() * 8;
        }
    }

    // project a 3D point into image coordinates
    Eigen::Vector2i projection(cv::Mat projectionMatrix, Eigen::Vector3d centerPoint){
        //in progress
        Eigen::MatrixXd p_Matrix;
        Eigen::Vector4d centerPoint4d;
        centerPoint4d<<centerPoint, 1;
        cv::cv2eigen(projectionMatrix, p_Matrix);
        Eigen::Vector3d temp = p_Matrix * centerPoint4d;
        Eigen::Vector2i imagePoint;
        imagePoint<<temp(0) / temp(2), temp(1) / temp(2);
        return imagePoint;
    }

    // judge if a 2d point is inside the silhouette
    bool inSilhouette(Eigen::Vector2i imagePoint, cv::Mat silhouette) {
        if (imagePoint(0) <= 0 || imagePoint(1) <= 0 ||
            imagePoint(0) >= silhouette.size().width || imagePoint(1) >= silhouette.size().height) {
            return false;
        }
        if (silhouette.at<uchar>(imagePoint(1), imagePoint(0)) == 0) {
            return false;
        }
        return true;
    }
};