# AR marker and voxel carving

## Current progress

- Marker detection ✔
- Camera calibration on sigle image ✔
- Distortion fix ✔
- Camera pose estimation ✔ (?)

## Class `SingleImage` (not in use)

Class `SingleImage` need an input parameter `imagePath` and automatically detect all markers and calibrate camera. 

### Public variables

`image`: input image.

`intrinsicMatrix, distCoeffs, rvecs, tvecs` and `repError` store all the information gained during calibration process.

### Public methods

`getImageWithMarker()`: returns calibrated image and highlights all detected marks on it.

`getImageWithAxes()`: returns calibrated image and draws world frame onit.

`getCameraPose()`: returns pose matrix of camera:
$$\left[\begin{array}{cc}
R & T\\
0 & 1
\end{array}\right]$$

## Class `MultiImage`

Class `MultiImage` takes an input of file name list and calibrate the camera based on multiple image.

### Public variables

`images`: input images.

`intrinsicMatrix, distCoeffs, rvecs, tvecs` and `repError` store all the information gained during calibration process.

### Public methods

`getImageWithMarker(index)`: returns image and highlights all detected marks on it.

`getImageWithAxes(index)`: returns image and draws world frame onit.

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

### 4 Pose estimation with multiple images

<img src = "https://github.com/ge35tay/AR_Marker_and_Voxel_Carving/raw/HaifanZhang/InputData/1.jpg" width = 50%/><img src = "https://github.com/ge35tay/AR_Marker_and_Voxel_Carving/raw/HaifanZhang/OutputData/1.png" width = 50%/>
<center style="font-size:14px;color:#C0C0C0">left: raw image, right: getImageWithAxes()</center>

<img src = "https://github.com/ge35tay/AR_Marker_and_Voxel_Carving/raw/HaifanZhang/OutputData/single_1.png" width = 50%/><img src = "https://github.com/ge35tay/AR_Marker_and_Voxel_Carving/raw/HaifanZhang/OutputData/1.png" width = 50%/>
<center style="font-size:14px;color:#C0C0C0">left: estimation with sigle image, right: estimation with multiple images</center>

You can see more data in `InputData` and `OutputData` folder.

## To do

- Reconstruction 3D model
- Voxel carving.

## Build project

source code and `CMakeLists.txt` are in `Src` folder, OpenCV path is set with absolute path to `C:/3D-Scanning/FinalProject/Lib/opencv/`, you need to change it to your own install folder of opencv.
