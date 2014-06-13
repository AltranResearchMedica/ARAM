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
* \file ITag.hpp
* \brief Interface for tag, determine if potential tag is or not a tag
* \author Alexandre Kornmann
* \version 1.0
* \date 13 mars 2014
*
*/

#ifndef _ITAG_HPP_
#define _ITAG_HPP_

//std include
#include <string>

//ARAM include
#include <ARAM/export.hpp>
#include <ARAM/typedef.hpp>
#include <ARAM/ARAMException.hpp>
#include <ARAM/FrameSet.hpp>
#include <ARAM/ROIDetector/ROI.hpp>

#include <ARAM/tools/Extrinsics.hpp>
#include <ARAM/tools/Intrinsics.hpp>

//openCV include
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace aram
{
	/** 
	* Interface for tag, determine if potential tag is or not a tag
	*/
	class ARAM_EXPORT ITag
	{
	public :
		/**
		* Construct a Tag based on a ROI
		*
		* \param[in] ROI & Region of interest for this tag (contains corners)
		*/
		ITag(ROI &);
		

		/**
		* check tag validity
		*
		* \param[in,out] vecROI *rois vector of ROIs
		* \param[in] vecTag *tags vector of tags (always empty, useless since 0.1)
		* \param[in] FrameSet *fs set of frame, contains currentFrame (call fs->load("currentFrame"); to get the current frame), you can use this set to store results of operations like threshold, canny, ... 
		*/
		virtual bool checkTag(vecROI *, vecTag *, FrameSet *)=0;


		/**
		* Compute extrinsics parameter associeted with this tag
		* 
		* \param[in] Intrinsics & intrinsics parameters
		* \param[in] float size tag size (in user define unit, for example mm)
		* \return Extrinsics & rotation matrix
		*/
		virtual Extrinsics extrinsics(Intrinsics &, float)=0;


		/**
		* Unique id for this marker
		*
		* \return int id of this marker
		*/
		virtual int id()=0;


		/**
		* Rotate n times corners list (clock wise)
		*/
		void rotate(int n);


		/**
		* Getter
		* 
		* \return vecPoint2D ROI corners
		*/
		const vecPoint2D corners() const;


		/**
		* Setter
		*
		* \param[in] vecPoint2D new corners
		*/
		void corners(vecPoint2D);

	private :
		vecPoint2D _corners; /**< ROI which contains tag */
	};
};

#endif

