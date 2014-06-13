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
* \file LineFitting.hpp
* \brief Line fitting for ROIs detection
* \author Alexandre Kornmann
* \version 1.0
* \date 16 mai 2014
*
* Detect a potential tag (ROI), based on line fitting algorithm
*
*/

#ifndef _LINEFITTING_HPP_
#define _LINEFITTING_HPP_


//ARAM include
#include <ARAM/export.hpp>
#include <ARAM/ROIDetector/IROIDetector.hpp>

//openCV include
#include <opencv2/opencv.hpp>

namespace aram
{
	/** 
	* Detection of ROI based on lien fitting.
	*/
	class ARAM_EXPORT LineFitting : public IROIDetector
	{
	public :
		/**
		* Constructor
		*/
		LineFitting();


		/**
		* Find roi
		*
		* \param[in,out] vecROI *rois vector of ROIs
		* \param[in] vecTag *tags vector of tags (always empty, useless since 0.1)
		* \param[in] FrameSet *fs set of frame, contains currentFrame (call fs->load("currentFrame"); to get the current frame), you can use this set to store results of operations like threshold, canny, ... 
		*/
		void findROI(vecROI *, vecTag *, FrameSet *);

		
	private :		
		/**
		* Compute intersection between two lines
		* \param[in] Line first line (format : (u,v,x0,y0), see openCV doc, fitLine)
		* \param[in] Line second line (format : (u,v,x0,y0), see openCV doc, fitLine)
		* \return cv::Point intersection point
		*/
		cv::Point Intersect(Line, Line);


		/**
		* Naive border closure for Canny correction, remove 1 pixel hole
		*
		* \param[in] const cv::Mat &in CV_U8C1 binary matrix with border to close
		* \param[out] cv::mat &out CV_U8C1 binary matrix with border closed (must be different from in, to avoid side effects)
		*/
		void naiveClosure(const cv::Mat &in, cv::Mat &out);


		/**
		* Compute orthoganl distance between a line and a point
		*
		* \param[in] cv::Point point
		* \param[int] Line line
		* \return float distance between line and point
		*/
		float orthoDistance(cv::Point, Line);


		float _factEpsilon; /**< approxPolyDP factor for epsilon parameter (see openCV doc) */

		double _minPerimeter; /**< minimum perimeter value to be a ROI (trivial condition for tag detection) in pixel*/
		double _minArea; /**< minimum area value to be a ROI (trivial condition for tag detection) in pixel*/
		double _sigmaX; /**< Gaussian blur parameter */
		double _sigmaY; /**< Gaussian blur parameter */
		cv::Size _blurSize; /**< Gaussian blur kernel size */
		
		int _lowThreshold; /**< low threshold value for Canny hysteresis */
		int _highThreshold; /**< high threshold value for Canny hysteresis */
		int _sobelSize; /**< sobel kernel size */

		double _reps; /**< sufficient accuracy for the radius (distance between the coordinate origin and the line) (fitLine parameter) */
		double _aeps; /**< Sufficient accuracy for the angle (fitLine parameter) */
		int _distType; /**< Distance used by the M-estimator (see openCV doc) (fitLine parameter) */
	};
};

#endif
