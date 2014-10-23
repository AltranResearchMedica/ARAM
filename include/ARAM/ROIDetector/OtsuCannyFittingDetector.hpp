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
* \file OtsuCannyFittingDetector.hpp
* \brief Line fitting for ROIs detection
* \author Alexandre Kornmann
* \version 1.0
* \date 05 aout 2014
*
* Detect a potential tag (ROI), based on line fitting algorithm, using Otsu algorithm to find canny threshold value
*
*/

#ifndef _OTSUCANNYFITTINGDETECTOR_HPP_
#define _OTSUCANNYFITTINGDETECTOR_HPP_


//ARAM include
#include <ARAM/export.hpp>
#include <ARAM/typedef.hpp>
#include <ARAM/ARAMException.hpp>

#include <ARAM/ROIDetector/IROIDetector.hpp>
#include <ARAM/tools/contourClosing.hpp>


namespace aram
{

	/** 
	* Detect a potential tag (ROI), based on line fitting algorithm, using Otsu algorithm to find canny threshold value
	*/
	class ARAM_EXPORT OtsuCannyFittingDetector : public IROIDetector
	{
	public :
		/**
		* Constructor
		*
		* \param[in] FrameSet *fs FrameSet contains all current frame created by the library
		*/
		OtsuCannyFittingDetector(FrameSet* fs);


		/**
		* Find roi
		*
		* \param[in,out] vecROI *rois vector of ROIs
		*/
		void findROI(vecROI *rois);


		/**
		* epsilon getter
		*
		* \return float epsilon
		*/
		float epsilon() const;


		/**
		* epsilon setter
		*
		* \param[in] float 
		*/
		void epsilon(float e);


		/**
		* minPerimeter getter
		*
		* \return double min perimeter
		*/
		double minPerimeter() const;


		/**
		* minPerimeter setter
		*
		* \param[in] double p minimum perimeter value to be a ROI
		*/
		void minPerimeter(double p);


		/**
		* minArea getter
		*
		* \return double min area
		*/
		double minArea() const;


		/**
		* minArea setter
		*
		* \param[in] double a minimum area value to be a ROI
		*/
		void minArea(double a);


		/**
		* sigmaX getter
		*
		* \return double sigma X for gaussian blur (see openCV doc)
		*/
		double sigmaX() const;


		/**
		* sigmaX setter
		*
		* \param[in] double sX sigma X for gaussian blur (see openCV doc)
		*/
		void sigmaX(double sX);


		/**
		* sigmaY getter
		*
		* \return double sigma Y for gaussian blur (see openCV doc)
		*/
		double sigmaY() const;


		/**
		* sigmaY setter
		*
		* \param[in] double sy sigma Y for gaussian blur (see openCV doc)
		*/
		void sigmaY(double sY);


        /**
		* blurSize getter
		*
		* \return cv::Size size of gaussian blur kernel
		*/
		cv::Size blurSize() const;


        /**
		* blurSize setter
		*
		* \param[in] cv::Size s size of gaussian blur kernel
		*/
		void blurSize(cv::Size s);


        /**
		* sobelSize getter
		*
		* \return int low threshold parameter for Canny operator
		*/
		int sobelSize() const;


        /**
		* sobelSize setter
		*
		* \param[in] int s kernel size for Sobel operator (n x n)
		*/
		void sobelSize(int s);


        /**
		* reps getter
		*
		* \return double sufficient accuracy for the radius (distance between the coordinate origin and the line)
		*/
		double reps() const;


        /**
		* reps setter
		*
		* \param[in] int r sufficient accuracy for the radius (distance between the coordinate origin and the line)
		*/
		void reps(double r);


        /**
		* aeps getter
		*
		* \return double sufficient accuracy for the angle
		*/
		double aeps() const;


        /**
		* aeps setter
		*
		* \param[in] double a sufficient accuracy for the angle
		*/
		void aeps(double a);


	private :		
		/**
		* Compute intersection between two lines
		* \param[in] Line first line (format : (u,v,x0,y0), see openCV doc, fitLine)
		* \param[in] Line second line (format : (u,v,x0,y0), see openCV doc, fitLine)
		* \return Point2D intersection point
		*/
		Point2D Intersect(Line a, Line b);


		/**
		* Compute orthogonal distance between a line and a point
		*
		* \param[in] cv::Point point
		* \param[int] Line line
		* \return float distance between line and point
		*/
		float orthoDistance(cv::Point, Line);


		float m_epsilon; /**< approxPolyDP factor for epsilon parameter (see openCV doc) */

		double m_minPerimeter; /**< minimum perimeter value to be a ROI (trivial condition for tag detection) in pixel*/
		double m_minArea; /**< minimum area value to be a ROI (trivial condition for tag detection) in pixel*/
		double m_sigmaX; /**< Gaussian blur parameter */
		double m_sigmaY; /**< Gaussian blur parameter */
		cv::Size m_blurSize; /**< Gaussian blur kernel size */
		
		int m_lowThreshold; /**< low threshold value for Canny hysteresis */
		int m_highThreshold; /**< high threshold value for Canny hysteresis */
		int m_sobelSize; /**< sobel kernel size */

		double m_reps; /**< sufficient accuracy for the radius (distance between the coordinate origin and the line) (fitLine parameter) */
		double m_aeps; /**< Sufficient accuracy for the angle (fitLine parameter) */
		int m_distType; /**< Distance used by the M-estimator (see openCV doc) (fitLine parameter) */
	};
};

#endif
