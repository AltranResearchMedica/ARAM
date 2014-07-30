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
* \file Extrinsic.hpp
* \brief Contains rotation matrix and translation vector
* \author Alexandre Kornmann
* \version 1.0
* \date 10 avril 2014
* 
*/
#ifndef _EXTRINSIC_HPP_
#define _EXTRINSIC_HPP_

//Open CV include
#include <opencv2/opencv.hpp>

//ARAM include
#include <ARAM/export.hpp>
#include <ARAM/typedef.hpp>
#include <ARAM/ARAMException.hpp>

#include <ARAM/tools/Intrinsic.hpp>

namespace aram
{
	/**
	* Contains rotation matrix and translation vector
	*/
	class ARAM_EXPORT Extrinsic
	{
	public :
		/**
		* Compute extrinsics parameters using solvePnP (see OpenCV)
		*
		* \param[in] Intrinsic intr intrinsics parameters for the camera
		* \param[in] vecPoint2D imgPoint
		* \param[in] vecPoint3D objPoint
		*/
		Extrinsic(Intrinsic intr, vecPoint2D imgPoint, vecPoint3D objPoint);

		
		/**
		* Compute extrinsics parameters using solvePnP (see OpenCV)
		*
		* \param[in] Intrinsic intr intrinsics parameters for the camera
		* \param[in] vecPoint2D imgPoint
		* \param[in] vecPoint3D objPoint
		* \param[out] float &error reprojection error
		*/
		Extrinsic(Intrinsic intr, vecPoint2D imgPoint, vecPoint3D objPoint, float &error);


		/**
		* Store rotation matrix and translation matrix
		*
		* \param[in] Intrinsic intr intrinsics parameters for the camera
		* \param[in] cv::Mat &rmat rotation matrix
		* \param[in] cv::Mat &tvec translation vector
		*/
		Extrinsic(Intrinsic intr, cv::Mat &rmat, cv::Mat &tvec);


		/**
		* Compute reprojection error
		*
		* \return float reprojection error
		*/
		float error();


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
		/**
		* Compute extrinsics parameters using solvePnP (see OpenCV)
		*
		* \param[in] vecPoint2D imgPoint
		* \param[in] vecPoint3D objPoint
		*/
		void compute(vecPoint2D imgPoint, vecPoint3D objPoint);


		cv::Mat m_rmat; /**< rotation matrix (3x3)*/
		cv::Mat m_rvec; /**< rotation vector (3x1)*/
		cv::Mat m_tvec; /**< translation vector (3x1)*/

		Intrinsic m_intr;
		
		vecPoint2D m_imgPts;
		vecPoint3D m_objPts;
	};
};
#endif