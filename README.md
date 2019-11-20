# SFND 2D Feature Tracking
[logo]: ./output/report1.jpg
[report2]: ./output/report2.png
[report3]: ./output/report3.png
[report4]: ./output/report4.png
[report5]: ./output/report5.png
<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.

Now let's go for the project rubric points.

## Project rubric points:

### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf.

This readme will be used as the project report.

### 2. MP.1 Data Buffer Optimization:

- Implement a vector for dataBuffer objects whose size does not exceed a limit (e.g. 2 elements). This can be achieved by pushing in new elements on one end and removing elements on the other end. 

I have Implement a ring vector, which push back the image and if the buffer size is bigger than the defined buffer size defined by the code in the variable 
"int dataBufferSize = 2;" I remove the first image.
code :
```
if(dataBuffer.size() > dataBufferSize)
{
	dataBuffer.erase(dataBuffer.begin());
}
```
### 3. MP.2 Keypoint Detection:

- Implement detectors HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT and make them selectable by setting a string accordingly.

These detectors have been implemented in the file "matching2D_Student.cpp", the following functions:

| Detector                          | function               | 
| --------------------------------- |------------------------|
| HARRIS                            | detKeypointsHarris     |
| ShiTomasi                         | detKeypointsShiTomasi  |
| FAST, BRISK, ORB, AKAZE, and SIFT | detKeypointsModern     |

The detector is selectable using the string detectorType, in the function "detKeypointsModern" choose the correct detector from the string


- Remove all keypoints outside of a pre-defined rectangle and only use the keypoints within the rectangle for further processing


This done by using the rectangle bounding box in CV and use the function contains which ask if the rectangle contains the point or not.
here is an example for the image.

![alt text][logo]

### 4. MP.4 Keypoint Descriptors:

- Implement descriptors BRIEF, ORB, FREAK, AKAZE and SIFT and make them selectable by setting a string accordingly.

These descriptors have been implemented in the file "matching2D_Student.cpp", the following function "descKeypoints" is handling this requirement.

- Implement FLANN matching as well as k-nearest neighbor selection. Both methods must be selectable using the respective strings in the main function. 

The function "matchDescriptors" is handling this requirement 

special handling has been done if the descriptor type is HOG, the normalization method "NORM_HAMMING" can't be used in bruat force so a code is done to handle this by make normalize l1 as recommended.


- Use the K-Nearest-Neighbor matching to implement the descriptor distance ratio test, which looks at the ratio of best vs. second-best match to decide whether to keep an associated pair of keypoints.

The function "matchDescriptors" is handling this requirement 


### 5. MP.7 Performance Evaluation 1:

- Count the number of keypoints on the preceding vehicle for all 10 images and take note of the distribution of their neighborhood size. Do this for all the detectors you have implemented. 

The CSV final_result.csv in the output folder has all the informations:


![alt text][report2]

### 6. MP.8 Performance Evaluation 2:
 - Count the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors. In the matching step, the BF approach is used with the descriptor distance ratio set to 0.8.

![alt text][report3]

### 7. MP.8 Performance Evaluation 3:

- Log the time it takes for keypoint detection and descriptor extraction. The results must be entered into a spreadsheet and based on this data, the TOP3 detector / descriptor combinations must be recommended as the best choice for our purpose of detecting keypoints on vehicles.

The information can be found in the final_result.csv

There were some problems for some detectors and discriptors combination for example the AKAZE descriptor works only with AKAZE detector, as well the SIFT doesn't work with ORB,so these combinatios aren't tested.

![alt text][report4]

I removed all the detectors and descriptors that can't be used for real time because it takes long time, and after that I choose the detector descriptor which has much matched keypoints and found the top 3 filters detector / descriptor are :


![alt text][report5]
