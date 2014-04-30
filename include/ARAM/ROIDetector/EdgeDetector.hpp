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
* \file EdgeDetector.hpp
* \brief Edge based detection
* \author Alexandre Kornmann
* \version 1.0
* \date 13 mars 2014
*
* Detect a potential tag, based on an edge finding algorithm
*
*/

#ifndef _EDGEDETECTOR_HPP_
#define _EDGEDETECTOR_HPP_


//ARAM include
#include <ARAM/export.hpp>
#include <ARAM/ROIDetector/IROIDetector.hpp>

//openCV include
#include <opencv2/opencv.hpp>

namespace aram
{
	/** 
	* Detection of ROI based on edge finding, using Canny operator to detect contour. The same detector can be used with threshold operation to detect contour. Threshold is faster, but isn't robust against  issues.
	*/
	class ARAM_EXPORT EdgeDetector : public IROIDetector
	{
	public :
		/**
		* Find roi
		*
		* \param[in,out] vecROI *rois vector of ROIs
		* \param[in] vecTag *tags vector of tags (always empty, useless since 0.1)
		* \param[in] FrameSet *fs set of frame, contains currentFrame (call fs->load("currentFrame"); to get the current frame), you can use this set to store results of operations like threshold, canny, ... 
		*/
		void findROI(vecROI *, vecTag *, FrameSet *);
	};
};

#endif
