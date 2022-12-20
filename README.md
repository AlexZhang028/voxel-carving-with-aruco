# AR marker and voxel carving

## Current progress

- Marker detection ✔
- Camera calibration on sigle image ✔
- Distortion fix ✔
- Camera pose estimation ✔ (?)

## Introduction

Class `CameraView` need an input parameter `imagePath` and automatically detect all markers and calibrate camera. Original input image will be calibrated and all later works are implemented on calibrated image. The attributes.

### Public variables

`image`: calibrated image.

`intrinsicMatrix, distCoeffs, rvecs, tvecs` and `repError` store all the information gained during calibration process.

### Public methods

`getImageWithMarker()`: returns calibrated image and highlights all detected marks on it.

`getImageWithAxes()`: returns calibrated image and draws world frame onit.

`getCameraPose()`: returns pose matrix of camera:
$$\left[\begin{array}{cc}
R & T\\
0 & 1
\end{array}\right]$$

## Example pictures

### 1 Highlighting marker

<img src = "https://github.com/ge35tay/AR_Marker_and_Voxel_Carving/raw/HaifanZhang/InputData/ArUcoBoard1.jpg" width = 50%/><img src = "https://github.com/ge35tay/AR_Marker_and_Voxel_Carving/raw/HaifanZhang/OutputData/marker_ArUcoBoard1.png" width = 50%/>
<center style="font-size:14px;color:#C0C0C0">left: raw image, right: getImageWithMarker()</center>

### 2 Fix distortion

<img src = "https://github.com/ge35tay/AR_Marker_and_Voxel_Carving/raw/HaifanZhang/InputData/distortionTest1.png" width = 50%/><img src = "https://github.com/ge35tay/AR_Marker_and_Voxel_Carving/raw/HaifanZhang/OutputData/marker_distortionTest1.png" width = 50%/>
<center style="font-size:14px;color:#C0C0C0">left: raw image, right: getImageWithMarker()</center>

### 3 Pose estimation

<img src = "https://github.com/ge35tay/AR_Marker_and_Voxel_Carving/raw/HaifanZhang/InputData/ArUcoBoard2.jpg" width = 50%/><img src = "https://github.com/ge35tay/AR_Marker_and_Voxel_Carving/raw/HaifanZhang/OutputData/axes_ArUcoBoard2.png" width = 50%/>
<center style="font-size:14px;color:#C0C0C0">left: raw image, right: getImageWithAxes()</center>

You can see more data in `InputData` and `OutputData` folder.

## To do

- Reconstruction 3D model
- Voxel carving.

## Build project

source code and `CMakeLists.txt` are in `Src` folder, OpenCV path is set with absolute path to `C:/3D-Scanning/FinalProject/Lib/opencv/`, you need to change it to your own install folder of opencv.
