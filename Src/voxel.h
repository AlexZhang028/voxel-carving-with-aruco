#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include "Eigen.h"
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include <fstream>

class Voxel {
public:
    int n_pointWithColor = 0;
    int n_centerPoints = 0;
    int n_vertices = 0;
    int n_faces = 0;
    std::vector<cv::Mat> images, silhouettes, projectionMatrices;

    Voxel(std::vector<Eigen::Vector3d> centerPoints_input, double size_x_input, double size_y_input,
          double size_z_input) {
        size_x = size_x_input;
        size_y = size_y_input;
        size_z = size_z_input;
        centerPoints = centerPoints_input;
    }

    // n_x: number of voxels in x direction, size_x: size of a single voxel in x direction (length)
    Voxel(int n_x, int n_y, int n_z,
          Eigen::Vector3d startPoint, Eigen::Vector3d endPoint,
          std::vector<cv::Mat> images_input,
          std::vector<cv::Mat> silhouettes_input,
          std::vector<cv::Mat> p_Matrices_input) {
        images = images_input;
        silhouettes = silhouettes_input;
        projectionMatrices = p_Matrices_input;
        size_x = (endPoint(0) - startPoint(0)) / n_x;
        size_y = (endPoint(1) - startPoint(1)) / n_y;
        size_z = (endPoint(2) - startPoint(2)) / n_z;
        for (int x_idx = 0; x_idx < n_x; x_idx++) {
            for (int y_idx = 0; y_idx < n_y; y_idx++) {
                for (int z_idx = 0; z_idx < n_z; z_idx++) {
                    Eigen::Vector3d centerPoint(x_idx * size_x + startPoint(0), y_idx * size_y + startPoint(1),
                                                z_idx * size_z + startPoint(2));
                    centerPoints.push_back(centerPoint);
                }
            }
        }
    }

    void writeCenterPoints(std::string fileName, bool color) {
        std::cout << "saving center points in: "<< fileName << std::endl;
        double timex = static_cast<double>(cv::getTickCount());
        std::ofstream outFile(fileName, std::ios::binary);
        outFile << "COFF" << std::endl;

        outFile << "# numVertices numFaces numEdges" << std::endl;

        if (color) {
            outFile << n_pointWithColor << " " << "0" << " 0" << std::endl;

            for (int idx = 0; idx < n_pointWithColor; idx++) {
                if (pointColor[idx](3) != 0) {
                    outFile << pointWithColor[idx][0] << " " << pointWithColor[idx][1] << " " << pointWithColor[idx][2]
                            << " "
                            << pointColor[idx](0) << " " << pointColor[idx](1) << " " << pointColor[idx](2) << " "
                            << pointColor[idx](3) << std::endl;
                }
            }
        } else {
            outFile << n_centerPoints << " " << "0" << " 0" << std::endl;

            for (int idx = 0; idx < n_centerPoints; idx++) {
                outFile << centerPoints[idx][0] << " " << centerPoints[idx][1] << " " << centerPoints[idx][2]
                        << std::endl;
            }
        }
        outFile.close();
        timex = ((double) cv::getTickCount() - timex) / cv::getTickFrequency();
        std::cout << "time for saving: " << timex << "s" << std::endl;
    }


    void writeVertices(std::string fileName) {
        getVertices(false);
        std::ofstream outFile(fileName, std::ios::binary);
        outFile << "COFF" << std::endl;

        outFile << "# numVertices numFaces numEdges" << std::endl;

        outFile << n_vertices << " " << "0" << " 0" << std::endl;

        std::cout << "writing Vertices" << std::endl;

        for (int i = 0; i < allVertices.size(); i++) {
            for (int j = 0; j < allVertices[i].size(); j++) {
                outFile << allVertices[i][j][0] << " " << allVertices[i][j][1] << " " << allVertices[i][j][2]
                        << std::endl;
            }
        }
        outFile.close();
    }

    void writeMesh(std::string fileName, bool color) {
        getVertices(color);
        getFaces();
        std::ofstream outFile(fileName, std::ios::binary);
        std::cout << "Writing vertices" << std::endl;
        outFile << "COFF" << std::endl;

        outFile << "# numVertices numFaces numEdges" << std::endl;
        outFile << n_vertices << " " << n_faces << " 0" << std::endl;
        if (color) {
            for (int i = 0; i < allVertices.size(); i++) {
                for (int j = 0; j < allVertices[i].size(); j++) {
                    if (pointColor[i](3) != 0) {
                        outFile << allVertices[i][j][0] << " " << allVertices[i][j][1] << " " << allVertices[i][j][2]
                                << " "
                                << pointColor[i][0] << " " << pointColor[i][1] << " " << pointColor[i][2] << " "
                                << pointColor[i][3] << std::endl;
                    }
                }
            }
        } else {
            for (int i = 0; i < allVertices.size(); i++) {
                for (int j = 0; j < allVertices[i].size(); j++) {
                    outFile << allVertices[i][j][0] << " " << allVertices[i][j][1] << " " << allVertices[i][j][2]
                            << std::endl;
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

    void carve(int carvingThreshold) {
        double timex = static_cast<double>(cv::getTickCount());
        std::vector<bool> centerPointFlags(centerPoints.size(), true);
        std::cout << "Carving" << std::endl;
//        for (int imageIdx = 0; imageIdx < silhouettes.size(); imageIdx++) {
//            std::cout<<"Carving: processing image ["<<imageIdx + 1<<"/"<<silhouettes.size()<<"]"<<std::endl;
        for (int pointIdx = 0; pointIdx < centerPoints.size(); pointIdx++) {
            // perform only to points that are retained from last iteration to save resource
            if (centerPointFlags[pointIdx]) {
                // mark all points with projection outside silhouette
//                    Eigen::Vector2i imagePoint = projection(projectionMatrices[imageIdx], centerPoints[pointIdx]);
//                    centerPointFlags[pointIdx] = inSilhouette(imagePoint, silhouettes[imageIdx]);
                centerPointFlags[pointIdx] = visibility(centerPoints[pointIdx], carvingThreshold);
            }
        }
//        }
        std::vector<Eigen::Vector3d> remainedPoints;

        for (int i = 0; i < centerPoints.size(); i++) {
            if (centerPointFlags[i]) {
                remainedPoints.push_back(centerPoints[i]);
            }
        }
        centerPoints = remainedPoints;
        n_centerPoints = centerPoints.size();
        std::cout << "Carving done" << std::endl;
        timex = ((double) cv::getTickCount() - timex) / cv::getTickFrequency();
        std::cout << "time for carving: " << timex << "s" << std::endl;
    }

    void colorRender(int colorThreshold) {
        std::cout << "start retrieving color information" << std::endl;
        double timex = static_cast<double>(cv::getTickCount());
        Eigen::Vector4i colorInitial(0, 0, 0, 0);
        std::vector<Eigen::Vector4i> colorTemp(centerPoints.size(), colorInitial);
        for (int imageIdx = 0; imageIdx < images.size(); imageIdx++) {
            int nextImageIdx = imageIdx + 1;
            if (nextImageIdx == images.size()) {
                nextImageIdx = 0;
            }
            for (int pointIdx = 0; pointIdx < centerPoints.size(); pointIdx++) {
                Eigen::Vector2i imagePoint = projection(projectionMatrices[imageIdx],centerPoints[pointIdx]);
                Eigen::Vector2i nextImagePoint = projection(projectionMatrices[nextImageIdx],centerPoints[pointIdx]);
                if (colorConsistency(imagePoint, nextImagePoint, imageIdx, nextImageIdx, colorThreshold) &&
                    colorTemp[pointIdx](3) == 0) {
                    int B = images[imageIdx].at<cv::Vec3b>(imagePoint(1), imagePoint(0))[0];
                    int G = images[imageIdx].at<cv::Vec3b>(imagePoint(1), imagePoint(0))[1];
                    int R = images[imageIdx].at<cv::Vec3b>(imagePoint(1), imagePoint(0))[2];
                    int A = 255;
                    Eigen::Vector4i color(R, G, B, A);
                    colorTemp[pointIdx] = color;
                }
            }
        }
        timex = ((double) cv::getTickCount() - timex) / cv::getTickFrequency();
        std::cout << "time for rendering: " << timex << "s" << std::endl;
        for (int i = 0; i < colorTemp.size(); i++) {
            if (colorTemp[i](3) != 0) {
                pointWithColor.push_back(centerPoints[i]);
                pointColor.push_back(colorTemp[i]);
            }
        }
        n_pointWithColor = pointWithColor.size();
        std::cout << "color information retrieved" << std::endl;

    }


private:
    std::vector<Vector4i> pointColor;
    double size_x, size_y, size_z;
    std::vector<Eigen::Vector3d> centerPoints, pointWithColor;
    std::vector<std::vector<Eigen::Vector3d>> allVertices;
    std::vector<std::string> faces;

    int distance(Eigen::Vector2i imagePoint1, Eigen::Vector2i imagePoint2, int imageIdx1,
                 int imageIdx2) {
        int distanceB = images[imageIdx1].at<cv::Vec3b>(imagePoint1(1), imagePoint1(0))[0] -
                        images[imageIdx2].at<cv::Vec3b>(imagePoint2(1), imagePoint2(0))[0];
        int distanceG = images[imageIdx1].at<cv::Vec3b>(imagePoint1(1), imagePoint1(0))[1] -
                        images[imageIdx2].at<cv::Vec3b>(imagePoint2(1), imagePoint2(0))[1];
        int distanceR = images[imageIdx1].at<cv::Vec3b>(imagePoint1(1), imagePoint1(0))[2] -
                        images[imageIdx2].at<cv::Vec3b>(imagePoint2(1), imagePoint2(0))[2];
        int distance = distanceB * distanceB + distanceG * distanceG + distanceR * distanceR;
        return distance;
    }

    bool colorConsistency(Eigen::Vector2i imagePoint1, Eigen::Vector2i imagePoint2,
                          int imageIdx1, int imageIdx2, int threshold) {
        if (distance(imagePoint1, imagePoint2, imageIdx1, imageIdx2) <= threshold) {
            return true;
        }
        return false;
    }

    void getFaces() {
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
                vertices.push_back(pointWithColor[i] + Eigen::Vector3d(-size_x / 2, -size_y / 2, -size_z / 2));
                vertices.push_back(pointWithColor[i] + Eigen::Vector3d(+size_x / 2, -size_y / 2, -size_z / 2));
                vertices.push_back(pointWithColor[i] + Eigen::Vector3d(+size_x / 2, +size_y / 2, -size_z / 2));
                vertices.push_back(pointWithColor[i] + Eigen::Vector3d(-size_x / 2, +size_y / 2, -size_z / 2));
                vertices.push_back(pointWithColor[i] + Eigen::Vector3d(-size_x / 2, -size_y / 2, +size_z / 2));
                vertices.push_back(pointWithColor[i] + Eigen::Vector3d(+size_x / 2, -size_y / 2, +size_z / 2));
                vertices.push_back(pointWithColor[i] + Eigen::Vector3d(+size_x / 2, +size_y / 2, +size_z / 2));
                vertices.push_back(pointWithColor[i] + Eigen::Vector3d(-size_x / 2, +size_y / 2, +size_z / 2));
                allVertices.push_back(vertices);
            }
            n_vertices = allVertices.size() * 8;
        } else {
            for (int i = 0; i < centerPoints.size(); i++) {
                vertices.clear();
                vertices.push_back(centerPoints[i] + Eigen::Vector3d(-size_x / 2, -size_y / 2, -size_z / 2));
                vertices.push_back(centerPoints[i] + Eigen::Vector3d(+size_x / 2, -size_y / 2, -size_z / 2));
                vertices.push_back(centerPoints[i] + Eigen::Vector3d(+size_x / 2, +size_y / 2, -size_z / 2));
                vertices.push_back(centerPoints[i] + Eigen::Vector3d(-size_x / 2, +size_y / 2, -size_z / 2));
                vertices.push_back(centerPoints[i] + Eigen::Vector3d(-size_x / 2, -size_y / 2, +size_z / 2));
                vertices.push_back(centerPoints[i] + Eigen::Vector3d(+size_x / 2, -size_y / 2, +size_z / 2));
                vertices.push_back(centerPoints[i] + Eigen::Vector3d(+size_x / 2, +size_y / 2, +size_z / 2));
                vertices.push_back(centerPoints[i] + Eigen::Vector3d(-size_x / 2, +size_y / 2, +size_z / 2));
                allVertices.push_back(vertices);
            }
            n_vertices = allVertices.size() * 8;
        }
    }

    // project a 3D point into image coordinates
    Eigen::Vector2i projection(cv::Mat projectionMatrix ,Eigen::Vector3d centerPoint) {
        Eigen::Vector2i imagePoint;
        double z = projectionMatrix.at<double>(2, 0) * centerPoint[0] +
                   projectionMatrix.at<double>(2, 1) * centerPoint[1] +
                   projectionMatrix.at<double>(2, 2) * centerPoint[2] +
                   projectionMatrix.at<double>(2, 3);

        imagePoint[0] = (projectionMatrix.at<double>(0, 0) * centerPoint[0] +
                         projectionMatrix.at<double>(0, 1) * centerPoint[1] +
                         projectionMatrix.at<double>(0, 2) * centerPoint[2] +
                         projectionMatrix.at<double>(0, 3)) / z;

        imagePoint[1] = (projectionMatrix.at<double>(1, 0) * centerPoint[0] +
                         projectionMatrix.at<double>(1, 1) * centerPoint[1] +
                         projectionMatrix.at<double>(1, 2) * centerPoint[2] +
                         projectionMatrix.at<double>(1, 3)) / z;

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

    bool
    visibility(Eigen::Vector3d centerPoint, int carvingThreshold) {
        int visibility = silhouettes.size();
        for (int imageIdx = 0; imageIdx < silhouettes.size(); imageIdx++) {
            Eigen::Vector2i imagePoint = projection(projectionMatrices[imageIdx], centerPoint);
            if (imagePoint(0) <= 0 || imagePoint(1) <= 0 ||
                imagePoint(0) >= silhouettes[imageIdx].size().width ||
                imagePoint(1) >= silhouettes[imageIdx].size().height) {
                visibility--;
            } else {
                if (silhouettes[imageIdx].at<uchar>(imagePoint(1), imagePoint(0)) == 0) {
                    visibility--;
                }
            }

        }
        if (visibility < silhouettes.size() - carvingThreshold) {
            return false;
        }
        return true;
    }
};