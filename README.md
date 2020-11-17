# cameraSterioCalibration

## Requirements
* OpenCV
* OpenCV_contrib

### Install OpenCV
```
mkdir opencv && cd opencv

git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git

mkdir build && cd build

cmake -DOPENCV_EXTRA_MODULES_PATH=/full/path/to/opencv_contrib/modules ../opencv

cmake --build .
```
