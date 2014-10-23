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
* \file AdaptiveThreshDetector.hpp
* \brief Edge based detection
* \author Alexandre Kornmann
* \version 1.0
* \date 13 mars 2014
*
* Detection of ROI based on edge finding, using Canny operator to detect contour. The same detector can be used with threshold operation to detect contour. Threshold is faster, but isn't robust against illumination issues.
*
*/

#ifndef _ADAPTIVETRESHDETECTOR_HPP_
#define _ADAPTIVETRESHDETECTOR_HPP_

//ARAM include
#include <ARAM/export.hpp>
#include <ARAM/ROIDetector/IROIDetector.hpp>

//openCV include
#include <opencv2/opencv.hpp>

namespace aram
{
	/** 
	* Detection of ROI based on edge finding, using Canny operator to detect contour. The same detector can be used with threshold operation to detect contour. Threshold is faster, but isn't robust against illumination issues.
	*/
	class ARAM_EXPORT AdaptiveThreshDetector : public IROIDetector
	{
	public :
		/**
		* Constructor
		*
		* \param[in] FrameSet *fs FrameSet contains all current frame created by the library
		*/
		AdaptiveThreshDetector(FrameSet *fs);


		/**
		* Find roi
		*
		* \param[in,out] vecROI *rois vector of ROIs
		*/
		void findROI(vecROI *rois);


		void operator()(vecROI *rois);


		/**
		* epsilon getter
		*
		* \return float epsilon
		*/
		float epsilon() const;


		/**
		* epsilon setter
		*
		* \param[in] float epsilon
		*/
		void epsilon(float e);


		/**
		* blockSize getter
		*
		* \return int blockSize
		*/
		int blockSize() const;


		/**
		* blockSize setter
		*
		* \param[in] int n threshold kernel size (n x n)
		*/
		void blockSize(int n);


		/**
		* constant getter
		*
		* \return double constant
		*/
		double adaptiveConstant() const;


		/**
		* constant setter
		*
		* \param[in] double c adaptive threshold parameter (see openCV doc)
		*/
		void adaptiveConstant(double c);


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


	private :
		float m_epsilon; /**< approxPolyDP factor for epsilon parameter (see openCV doc) */
		int m_blockSize; /**< Adaptive threshold kernel size */
		double m_constant; /**< Adaptive threshold parameter (see openCV doc) */
		double m_minPerimeter; /**< minimum perimeter value to be a ROI (trivial condition for tag detection) in pixel*/
	};
};

#endif
