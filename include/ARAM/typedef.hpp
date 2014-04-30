/*
Copyright (c) 2014, Altran Research Medic@
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:
   This product includes software developed by Altran.
4. Neither the name of Altran nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY ALTRAN RESEARCH MEDIC@ ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ALTRAN RESEARCH MEDIC@ BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/





/**
*
* \file typedef.hpp
* \brief macro export definition
* \author Alexandre Kornmann
* \version 1.0
* \date 11 mars 2014
*
*/

#ifndef _TYPEDEF_HPP_
#define _TYPEDEF_HPP_

#include <opencv2/opencv.hpp>

namespace aram
{	
	class ARAMException;

	class FrameSet;

	//ROIs
	class ROI;
	class IROIDetector;
	class EdgeDetector;
	
	//Tags
	class ITag;
	class HammingTag;
	class StandardTag;

	//Coordinates
	class ICoordinate;
	class Chessboard;

	//Tools
	class BinaryTree;
	class Exporter;
	class Extrinsics;
	class Grid;
	class Intrinscis;


	/**
	* Points in 3D space (world or object coordinates)
	*/
	typedef cv::Point3f Point3D;	
	
	/**
	* Point3D vector
	*/
	typedef std::vector<Point3D> vecPoint3D;

	/**
	* Iterator on Point3D vector
	*/
	typedef vecPoint3D::iterator iteratorPoint3D;
	
	/**
	* Const iterator on Point2D vector
	*/
	typedef vecPoint3D::const_iterator constIteratorPoint3D;

	/**
	* Points in 2D space (image coordinate)
	*/
	typedef cv::Point2f Point2D;

	/**
	* Point2D vector
	*/
	typedef std::vector<Point2D> vecPoint2D;
	
	/**
	* Iterator on Point2D vector
	*/
	typedef vecPoint2D::iterator iteratorPoint2D;

	/**
	* Const iterator on Point2D vector
	*/
	typedef vecPoint2D::const_iterator constIteratorPoint2D;

	/**
	* ITag* vector
	*/
	typedef std::vector<ITag*> vecTag;
	
	/**
	* Iterator on ITag* vector 
	*/
	typedef vecTag::iterator iteratorTag;

	/**
	* ROI* vector
	*/
	typedef std::vector<ROI*> vecROI;
	
	/**
	* Iterator on ROI* vector 
	*/
	typedef vecROI::iterator iteratorROI;


};

#endif