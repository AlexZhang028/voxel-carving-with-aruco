#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

void showImg(const std::string& name,cv::Mat& img) {
    cv::namedWindow(name, cv::WINDOW_KEEPRATIO);
    cv::imshow(name,img);
    cv::waitKey(0);
}


int main() {
    for (int i = 0; i<=35; i++){
        cv::Mat src = cv::imread("e:/documents/3D-Scanning/DataSet_Old/image_"+ std::to_string(i) + ".jpg");
        assert(!src.empty());
        //    cv::Rect boundingBox(150,80,410,500); //My Image Bounding Box Value
        /* Reading Bounding Box */
        cv::Rect boundingBox(280, 50, 700, 800);
        cv::Mat temp = src.clone();
        cv::rectangle(temp, boundingBox, cv::Scalar(0, 255, 0), 3);
        //    showImg("boundingBox", temp);
        cv::Mat mask = cv::Mat::zeros(src.rows, src.cols, CV_8UC1);
        cv::Mat bgModel, fgModel;
        unsigned int iteration = 5; //Tune Parameter according to need
        cv::grabCut(src, mask, boundingBox, bgModel, fgModel, iteration, cv::GC_INIT_WITH_RECT);
        cv::Mat mask2 = (mask == 1) + (mask == 3);  // 0 = cv::GC_BGD, 1 = cv::GC_FGD, 2 = cv::PR_BGD, 3 = cv::GC_PR_FGD
        cv::Mat dest;
        src.copyTo(dest, mask2);
        std::cout<<"Writing image "<<i<<std::endl;
        cv::imwrite("e:/documents/3D-Scanning/DataSet_Old/Processed/image" + std::to_string(i) + ".jpg", dest);
    }
//    showImg("dest", dest);
//    cv::waitKey(0);
    return 0;

}