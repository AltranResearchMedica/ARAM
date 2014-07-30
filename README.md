ARAM: Augmented Reality for Application on Medical field library
-------------------------------------------------------------------
ARAM is a C++ library for Augmented Reality on Medical Applications based on the extraction and identification of fiducial markers in real time. Based on Opencv, several technologies and approaches are combined to resolve the various problems related to the accuracy, robustness and occlusions. This library is provided under BSD license.


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

![Mono tracking example](https://raw.githubusercontent.com/AltranResearchMedica/ARAM/master/samples/capture/AR_0.png)
![Mono tracking example](https://raw.githubusercontent.com/AltranResearchMedica/ARAM/master/samples/capture/AR_1.png)

# Main class

The library main classes are divided as following:

- aram::TagDetector<TagType,ROIDetector> : main aram class, used to detect tags in a frame.
- aram::IROIDetector : Interface for ROI Detection
- aram::CannyFittingDetector : currently best IROIDetector implementation 
- aram::AdaptiveThreshDetector : another IROIDetector implementation 
- aram::ITagMatcher : Interface for Tag validation
- aram::LocalThreshTagMatcher : currently best ITagMatcher implementation
- aram::Intrinscic : store camera intrinsic parameters
- aram::Extrinsic : compute extrinsic parameters
- aram::ICoordinate : compute extrinsics parameters, using one or more tags
- aram::MultiTag : currently best ICoordinate implementation


# Compile

## Requirements

- OpenCv >= 2.4.8
- CMake >= 2.8


## Compiler

Tested with Microsoft Visual C++ (Express Edition 2010) and gcc 4.6.3

- Use CMake to generate the solution (.sln or makefile)
- Compile


# Samples

## camera_data.xml & record.avi

A video, with correct calibration camera, to test samples if you don't have a camera


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


-------------------------------------------------------------------
# Acknowledgment

This work is contributed by Alexandre KORNMANN in the framework of his final project assignment in Altran Research Medic@ under the responsibility of Dr. Abdelkrim BELHAOUA. Altran Research Medic@ is research program aimed to integrate innovative augmented reality technology in order to improve the medical operating services.

-------------------------------------------------------------------
# Contact

Please report any bugs, comment about error and/or suggest enhancements to support-medica@altran.com

