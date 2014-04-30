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
* \file Extrinsics.hpp
* \brief Contains rotation matrix and translation vector
* \author Alexandre Kornmann
* \version 1.0
* \date 10 avril 2014
* 
*/
#ifndef _EXTRINSICS_HPP_
#define _EXTRINSICS_HPP_

//Open CV include
#include <opencv2/opencv.hpp>

//ARAM include
#include <ARAM/export.hpp>
#include <ARAM/typedef.hpp>
#include <ARAM/ARAMException.hpp>

#include <ARAM/tools/Intrinsics.hpp>

namespace aram
{
	/**
	* Contains rotation matrix and translation vector
	*/
	class ARAM_EXPORT Extrinsics
	{
	public :
		/**
		* Compute extrinsics parameters using solvePnP (see OpenCV)
		*
		* \param[in] Intrinsics intr intrinsics parameters for the camera
		* \param[in] vecPoint2D imgPoint
		* \param[in] vecPoint3D objPoint
		*/
		Extrinsics(Intrinsics, vecPoint2D, vecPoint3D);

		
		/**
		* Compute extrinsics parameters using solvePnP (see OpenCV)
		*
		* \param[in] Intrinsics intr intrinsics parameters for the camera
		* \param[in] vecPoint2D imgPoint
		* \param[in] vecPoint3D objPoint
		* \param[out] float & error reprojection error
		*/
		Extrinsics(Intrinsics, vecPoint2D, vecPoint3D, float &);


		/**
		* Store rotation matrix and translation matrix
		*
		* \param[in] Intrinsics intr intrinsics parameters for the camera
		* \param[in] cv::Mat &rmat rotation matrix
		* \param[in] cv::Mat &tvec translation vector
		*/
		Extrinsics(Intrinsics, cv::Mat &, cv::Mat &);


		/**
		* Compute extrinsics parameters using solvePnP (see OpenCV)
		*
		* \param[in] vecPoint2D imgPoint
		* \param[in] vecPoint3D objPoint
		*/
		void compute(vecPoint2D, vecPoint3D);


		/**
		* Get rotation matrix
		*
		* \return const cv::Mat & rotation matrix (3x3)
		*/
		const cv::Mat & rotationMatrix();


		/**
		* Get rotation vector
		*
		* \return const cv::Mat & rotation vector (3x1) in openCV format (see openCV doc)
		*/
		const cv::Mat & rotationVector();


		/**
		* Get translation vector
		*
		* \return const cv::Mat & translation vector (3x1)
		*/
		const cv::Mat & translationVector();

		/**
		* Project a 3d point on image
		*
		* \param[in] Point3D pt point to project (3D space)
		* \return Point2D point in image coordinates
		*/
		Point2D project(Point3D pt);

		/**
		* Project 3d points on image
		*
		* \param[in] vecPoint3D objPoints point to project (3D space)
		* \return vecPoint2D point in image coordinates
		*/
		vecPoint2D project(vecPoint3D objPoints);
		
		
		/**
		* Getter
		*/
		vecPoint3D objPoints();


		/**
		* Getter
		*/
		vecPoint2D imgPoints();

	private :
		cv::Mat _rmat; /**< rotation matrix (3x3)*/
		cv::Mat _rvec; /**< rotation vector (3x1)*/
		cv::Mat _tvec; /**< translation vector (3x1)*/

		Intrinsics _intr;
		
		vecPoint2D _imgPts;
		vecPoint3D _objPts;
	};
};
#endif