# Current progress
## Marker detection and camera calibration with ArUco board

Class `CameraView` need an input parameter `imagePath` and automatically detect all markers and calibrate camera. Original input data can be accessed with attribute `image`. The attributes `intrinsicMatrix, distCoeffs, rvecs, tvecs` and `repError` store all the information gained during calibration process.

Method `getImageWithMarker()` returns raw input image and highlights all detected marks on it. `getUndistortedImage()` returns calibrated image.

# Example pictures

## 1 highlighting marker
<img src = "./inputData/ArUcoBoard1.jpg" width = 50%/><img src = "./outputData/out_ArUcoBoard1.png" width = 50%/>
<center style="font-size:14px;color:#C0C0C0">left: raw image, right: getImageWithMarker()</center>

## 2 camera calibration
<img src = "./inputData/distortionTest1.png" width = 33%/><img src = "./outputData/out_distortionTest1.png" width = 33%/><img src = "./outputData/calibrated1.png" width = 33%/>
<center style="font-size:14px;color:#C0C0C0">left: raw image, mid: getImageWithMarker(), right: getUndistortedImage()</center>

You can see more data in `InputData` and `OutputData` folder.

# To do
- Estimation of camera pose.
- Voxel carving.

# Build project
source code and `CMakeLists.txt` are in `Src` folder, OpenCV path is set with absolute path to `C:/3D-Scanning/FinalProject/Lib/opencv/`, you need to change it to your own install folder of opencv.