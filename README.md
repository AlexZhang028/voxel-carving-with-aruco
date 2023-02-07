# AR marker and voxel carving

## Current progress

- Marker detection ✔
- Camera calibration on sigle image ✔
- Distortion fix ✔
- Camera pose estimation ✔
- Background extraction ✔
- Carving ✔
- Voxel visualization ✔
- Color rendering ✔

## Result

<div align=center>
  <img src=https://github.com/ge35tay/AR_Marker_and_Voxel_Carving/blob/HaifanZhang/InputData/board02-png-22/01.png width = 50%><img src=https://github.com/ge35tay/AR_Marker_and_Voxel_Carving/blob/HaifanZhang/OutputData/board02-png-22/Result_thres3_iter0.png width = 50%>
  <p>left: one of the input images; right: output 3D model (0 iteration, carvingThreshold 0, colorThreshold 3)</p>
</div>

## Build project

### Requirments
- Eigen3 (head only)
- OpenCV 4.7.0_dev

source code and `CMakeLists.txt` are in `Src` folder, OpenCV path is set with absolute path to `../Libs/opencv/`, you need to change it to your own install folder of opencv.

---

## `multiImage.h`

Contains the Class `MultiImage`, which takes an input of file name list and calibrate the camera based on multiple image.

### Initialize

`MultiImage::MultiImage(std::vector<std::string> fileNames, int erodeIter, int dilateIter, cv::Scalar hsv_min, cv::Scalar hsv_max)`

- `fileNames`: complete path of input images
- `erodeIter`: the iteration number of erode when getting silhouettes
- `dialteIter`: the iteration number of dialte when getting silhouettes
- `hsv_min`: the min hsv value when getting silhouettes
- `hsv_max`: the max hsv value when getting silhouettes

After initialize the class, the silhouette and projection matrix are automatically computed and stored in public variables.

### Public variables

`std::vector<cv::Mat> images`: input images.

`std::vector<cv::Mat> cameraPoses`: camera pose [R T; 0 1] of all images.

`std::vector<cv::Mat> silhouettes`: silhouette of input images.

`std::vector<cv::Mat> foregrounds`: foregrounds of input images for debugging, only availble after calling `extractForeground()`.

`std::vector<cv::Mat> p_Matrices`: projection matrix of all images.

### Public methods

`void extractForeground()`: extract the foreground of input images and stors in public variable `foregrounds`.

`void drawArUcoBoard(cv::Mat boardImage)`: stores the image of aruco board in `boardImage`. (only used once)

`cv::Mat getImageWithMarker(int index)`: returns an image and highlights all detected marks on it. (only for debugging)

`cv::Mat getImageWithAxes(int index)`: returns an image and draws world frame onit. (only for debugging)

## `voxel.h`

Contains the Class `Voxel`, which voxelize the space for carving and visulization.

### Initialization

`Voxel(int n_x, int n_y, int n_z, Eigen::Vector3d startPoint, Eigen::Vector3d endPoint, std::vector<cv::Mat> images_input, std::vector<cv::Mat> silhouettes_input, std::vector<cv::Mat> p_Matrices_input)`

- `int n_x`: number of voxels in x direction.
- `int n_y`: number of voxels in y direction.
- `int n_z`: number of voxels in z direction.
- `Eigen::Vector3d startPoint`: start point of bounding box (space to be voxelized)
- `Eigen::Vector3d endPoint)`: end point of bounding box (space to be voxelized)
- `images_input`: input images for carving
- `silhouettes_input`: input silhouettes for carving
- `p_Matrices_input`: input projection matrices for carving
This method discretize a specific area of workspace (bounding box) into given numbers of voxels.

`Voxel::Voxel(std::vector<Eigen::Vector3d> centerPoints_input, double size_x_input, double size_y_input, double size_z_input)` (only for debugging)
  
- `std::vector<Eigen::Vector3d> centerPoints_input`: input center points of voxelized space.
- `double size_x_input`: width of a voxel.
- `double size_y_input`: height of a voxel.
- `double size_z_input`: depth of a voxel.
This method loads a voxelized space with known center points. And are only used for testing and debugging.

### Public variables

`int n_centerPoints`: number of center points (voxels).

`int n_vertices`: number of vertices.

`int n_pointWithColor`: number of center points (voxels) with color.

`int n_faces`: number of faces.

### Public methods

`void writeCenterPoints(std::string fileName, bool color)`

- `std::string fileName`: complete path to save the off file.
- `bool color`: flag to determine if the color information is preserved.
This function write all center points as point cloud in a .off file.

`void writeVertices(std::string fileName)` (only for debugging)

- `std::string fileName`: complete path to save the off file.
Thisfunction write all vertices as point cloud in a .off file and only used for debugging and testing.

`void writeMesh(std::string fileName, bool color)`

- `std::string fileName`: complete path to save the off file.
- `bool color`: flag to determine if the color information is preserved.
This function write the whole mesh (vertices and surface) in a .off file.

`void carve(std::vector<cv::Mat> silhouettes, std::vector<cv::Mat> projectionMatrices, int carvingThreshold)`

- `std::vector<cv::Mat> silhouettes`: all silhouettes used for carving.
- `std::vector<cv::Mat> projectionMatrices)`: all projection matrices used for carving.
- `int carvingThreshold`: controls a voxel should be visible by how many images. if it's set to 0, then the function performs a normal voxel carving.
This function does the carving process with input of silhouettes and projection matrices.

`void colorRender(std::vector<cv::Mat> projectionMatrices, std::vector<cv::Mat> images, int colorThreshold)`

- `std::vector<cv::Mat> projectionMatrices)`: all projection matrices used for carving.
This function does the carving process with input of silhouettes and projection matrices.
- `std::vector<cv::Mat> images`: input images.
- `int colorThreshold`: controls the color consistency of a point in 2 images.
This function retrieves the color information from original input images.

---

## `singleImage.h` (not in use, only use MultiImage)

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
