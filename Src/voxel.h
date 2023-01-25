#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include "Eigen.h"
#include <iostream>
#include <fstream>

class Voxel{
public:

    Voxel(std::vector<Eigen::Vector3d> centerPoints_input, double size_x, double size_y, double size_z) {
        centerPoints = centerPoints_input;
        n_centerPoints = centerPoints.size();
        n_vertices = 8 * n_centerPoints;
        n_faces = 6 * n_centerPoints;
        for (int i = 0; i < centerPoints_input.size(); i++){
            getVertices(centerPoints_input[i], size_x, size_y, size_z);
        }
    }
    // n_x: number of voxels in x direction, size_x: size of a single voxel in x direction (length)
    Voxel(int n_x, int n_y, int n_z, double size_x, double size_y, double size_z, Eigen::Vector3d startPoint) {
        for (int x_idx = 0; x_idx < n_x; x_idx++){
            for (int y_idx = 0; y_idx < n_y; y_idx++){
                for (int z_idx = 0; z_idx < n_z; z_idx++){
                    Eigen::Vector3d centerPoint;
                    centerPoint<<x_idx * size_x + startPoint(0), y_idx * size_y + startPoint(1), z_idx * size_z + startPoint(2);
                    centerPoints.push_back(centerPoint);
                    getVertices(centerPoint, size_x, size_y, size_z);
                }
            }
        }
        n_centerPoints = n_x * n_y * n_z;
        n_vertices = 8 * n_centerPoints;
        n_faces = 6 * n_centerPoints;
    }

    void writeCenterPoints(std::string fileName) {
        std::ofstream outFile(fileName);
        outFile << "COFF" << std::endl;

        outFile << "# numVertices numFaces numEdges" << std::endl;

        outFile << n_centerPoints << " " << "0" << " 0" << std::endl;

        for (int idx = 0; idx < n_centerPoints; idx ++){
            outFile << centerPoints[idx][0] << " " << centerPoints[idx][1] << " " << centerPoints[idx][2] << std::endl;
        }
        outFile.close();
    }

    void writeVertices(std::string fileName) {
        std::ofstream outFile(fileName);
        outFile << "COFF" << std::endl;

        outFile << "# numVertices numFaces numEdges" << std::endl;

        outFile << n_vertices << " " << "0" << " 0" << std::endl;

        for (int i = 0; i < allVertices.size(); i ++){
            for (int j = 0; j < allVertices[i].size(); j ++){
                outFile << allVertices[i][j][0] << " " << allVertices[i][j][1] << " " << allVertices[i][j][2] << std::endl;
            }
        }
        outFile.close();
    }

    void writeMesh(std::string fileName) {
        getFaces();
        std::ofstream outFile(fileName);
        std::cout << "Writing vertices" << std::endl;
        outFile << "COFF" << std::endl;

        outFile << "# numVertices numFaces numEdges" << std::endl;

        outFile << n_vertices << " " << n_faces << " 0" << std::endl;

        for (int i = 0; i < allVertices.size(); i ++){
            for (int j = 0; j < allVertices[i].size(); j ++){
                outFile << allVertices[i][j][0] << " " << allVertices[i][j][1] << " " << allVertices[i][j][2] << std::endl;
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
    }

    void getVertices(Eigen::Vector3d centerPoint, double size_x, double size_y, double size_z) {
        std::vector<Eigen::Vector3d> vertices;
        vertices.push_back(centerPoint + Eigen::Vector3d(-size_x/2,-size_y/2,-size_z/2));
        vertices.push_back(centerPoint + Eigen::Vector3d(+size_x/2,-size_y/2,-size_z/2));
        vertices.push_back(centerPoint + Eigen::Vector3d(+size_x/2,+size_y/2,-size_z/2));
        vertices.push_back(centerPoint + Eigen::Vector3d(-size_x/2,+size_y/2,-size_z/2));
        vertices.push_back(centerPoint + Eigen::Vector3d(-size_x/2,-size_y/2,+size_z/2));
        vertices.push_back(centerPoint + Eigen::Vector3d(+size_x/2,-size_y/2,+size_z/2));
        vertices.push_back(centerPoint + Eigen::Vector3d(+size_x/2,+size_y/2,+size_z/2));
        vertices.push_back(centerPoint + Eigen::Vector3d(-size_x/2,+size_y/2,+size_z/2));
        allVertices.push_back(vertices);
    }

    void projection(cv::Mat projectionMatrix){
        //in progress
    }

    void carving(cv::Mat silhouette){
        //in progress
    }
    std::vector<Eigen::Vector3d> centerPoints;
    std::vector<std::vector<Eigen::Vector3d>> allVertices;
    std::vector<std::string> faces;
    int n_centerPoints;
    int n_vertices;
    int n_faces;
};