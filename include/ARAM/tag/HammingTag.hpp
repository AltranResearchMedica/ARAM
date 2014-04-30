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
* \file HammingTag.hpp
* \brief Hamming tag
* \author Alexandre Kornmann
* \version 1.0
* \date 13 mars 2014
*
*/

#ifndef _HAMMINGTAG_HPP_
#define _HAMMINGTAG_HPP_

//std include
#include <string>

//ARAM Include
#include <ARAM/export.hpp>
#include <ARAM/tag/ITag.hpp>
#include <ARAM/tools/BinaryTree.hpp>

//openCV include
#include <opencv2/opencv.hpp>

namespace aram
{
	/** 
	* Tag with hamming correction. After sampling in ROI, it's use dictionnary of tag to find the nearest tag (the shortest hamming distance beetween sampling and dictionnary) to determine if this ROI is a tag or not.
	* Use adaptative threshold before sampling.
	*/
	class ARAM_EXPORT HammingTag : public ITag
	{
	public :
		/**
		* Constructor
		*
		* \param[in] ROI & region of interest where is the tag
		*/
		HammingTag(ROI &);
		
		
		/**
		* Check tag validity by sampling ROIs.
		*
		* \param[in,out] vecROI *rois vector of ROIs
		* \param[in] vecTag *tags vector of tags (always empty, useless since 0.1)
		* \param[in] FrameSet *fs set of frame, contains currentFrame (call fs->load("currentFrame"); to get the current frame), you can use this set to store results of operations like threshold, canny, ... 
		*/
		bool checkTag(vecROI *, vecTag *, FrameSet *);

		/**
		* Compute extrinsics parameter associeted with this tag
		* 
		* \param[in] Intrinsics & intrinsics parameters
		* \param[in] float size tag size (in user define unit, for example mm)
		* \return Extrinsics & rotation matrix
		*/
		Extrinsics extrinsics(Intrinsics &, float);


		/**
		* Unique id for this marker
		*
		* \return int id of this marker
		*/
		int id();



	private :
		/**
		* Rotate CV_8UC1 matrix (clock wise)
		*
		* \param[in] cv::Mat & matrix to rotate, contains  
		* \param[out] cv::Mat & matrix after rotations  
		*/
		void rotate(cv::Mat &, cv::Mat &);

		int _tagSize; /**< tag size size in "bits number" (border include)*/
		int _scale; /**< value of perspective scale */

		int _id; /**< Tag id */
	};
};

#endif