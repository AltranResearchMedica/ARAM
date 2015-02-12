ARAM: Augmented Reality for Application on Medical field library
-------------------------------------------------------------------
ARAM is a C++ library for Augmented Reality on Medical Applications based on the extraction and identification of fiducial markers in real time. Based on Opencv, several technologies and approaches are combined to resolve the various problems related to the accuracy, robustness and occlusions.
This library is provided under BSD license. For more details, please contact Dr. Abdelkrim BELHAOUA at support-medica@altran.com.
A contribution has been presented to 12th International Conference on Signal Processing (ICSP, 2014). This work deals with error analysis in augmented reality (AR) systems.
http://ieeexplore.ieee.org/xpl/abstractSimilar.jsp?reload=true&arnumber=7015184

# Introduction

The library deals with the identification of markers in the presence of occlusions without sacrificing the accuracy of the pose camera estimation. These markers are generated using Hamming Code with error correction. 
In order to assess the accuracy of pose estimation, specifically when high levels of accuracy are required on medical field, it is mandatory to carry out a thorough analysis of the errors occurring in the whole process. Consequently, this library deals with the performance analysis of the 3D pose estimation. The suggested analysis focuses on the error estimation affecting the edge detection process.


# Description

The image processing software performs real-time 3D camera pose estimation. Firstly, the camera is carefully calibrated off-line. The extraction of edge information is the first fundamental processing step of the whole pose estimation procedure. The edge or contour point detection relies on well-known gradient-based methods. Following contour point extraction, chains of contour points are determined to form possibly closed contours. This latter are then reduced to a minimal number of vertices according to a specified tolerance. Finally, only the polygons containing the “good” marker are selected to compute the camera pose.

The processing steps of AR marker detection procedure can be summarized as follows:
-	Calibration of the camera (offline),
-	Segmentation of the acquired image and determination of contour point lists,
-	Classification of the edge point lists (Edge linking),
-	Polyline Simplification (Iterative Endpoint Fit)
-	Image Sampling
-	Pose estimation using the calibration parameters.


# Main class

The library main classes are divided as following:

- aram::TagDetector<TagType,ROIDetector> : main ARAM class, used to detect tags in a frame.
- aram::IROIDetector : Interface for ROI Detection
- aram::CannyFittingDetector : currently best IROIDetector implementation 
- aram::AdaptiveThreshDetector : another IROIDetector implementation 
- aram::ITagMatcher : Interface for Tag validation
- aram::LocalThreshTagMatcher : currently best ITagMatcher implementation
- aram::Intrinscic : store camera intrinsic parameters
- aram::Extrinsic : compute extrinsic parameters
- aram::ICoordinate : compute extrinsic parameters, using one or more tags
- aram::MultiTag : currently best ICoordinate implementation


# Compile

## Requirements

- OpenCv >= 2.4.8
- CMake >= 2.8


## Compiler

This library is tested using Microsoft Visual C++ (Edition Pro 2013) and CMake (3.1.0) to generate the solution (.sln or makefile).



# Samples

## camera_data.xml & record.avi

A video, with correct calibration camera parameters, is provided to test samples if you don't have a camera.


## FirstApp

- Check CMakeLists.txt
	'''
	include_directories("C:/Tracking/ARAM/include")
	link_directories("C:/Tracking/ARAM/build/lib/Debug")
	'''
- Use CMake to generate the solution (.sln or makefile)
- Compile

FirstApp circle corners of "good" tag.

## MultiTracking

- Check CMakeLists.txt
	'''
	include_directories("C:/Tracking/ARAM/include")
	link_directories("C:/Tracking/ARAM/build/lib/Debug")
	'''
- Use CMake to generate the solution (.sln or makefile)
- Compile

MultiTracking use ARAM abilities to track a chessboard of tag.

## TrackingOpenGL

- Check CMakeLists.txt
	'''
	include_directories("C:/Tracking/ARAM/include")
	link_directories("C:/Tracking/ARAM/build/lib/Debug")
	'''
- Use CMake to generate the solution (.sln or makefile)
- Compile

TrackingOpenGL uses openGL library to draw boxes on tracked tags.


-------------------------------------------------------------------
# Acknowledgment

Within the framework of the 3D SURG project, Altran Research Medic@ has developed this library under the responsibility of Dr. Jean Pierre RADOUX and Dr. Abdelkrim BELHAOUA. 
Altran Research Medic@ is research program aimed to integrate innovative augmented reality technology in order to improve the medical operating services.

-------------------------------------------------------------------
# Contact

Please report any bugs, comments about error and/or suggest enhancements Dr. Abdelkrim BELHAOUA (support-medica@altran.com)

