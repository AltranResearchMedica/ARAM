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
* \file ITagMatcher.hpp
* \brief Interface for tag, determine if potential tag is or not a tag
* \author Alexandre Kornmann
* \version 1.0
* \date 13 mars 2014
*
*/

#ifndef _ITAGMATCHER_HPP_
#define _ITAGMATCHER_HPP_

//ARAM include
#include <ARAM/export.hpp>
#include <ARAM/typedef.hpp>
#include <ARAM/ARAMException.hpp>

#include <ARAM/FrameSet.hpp>
#include <ARAM/ROIDetector/ROI.hpp>
#include <ARAM/tools/Extrinsic.hpp>
#include <ARAM/tools/Intrinsic.hpp>


namespace aram
{
	/** 
	* Interface for tag, determine if potential tag is or not a tag
	*/
	class ARAM_EXPORT ITagMatcher
	{
	public :
		/**
		* Constructor
		*
		* \param[in] FrameSet *fs FrameSet contains all current frame created by the library
		*/
		ITagMatcher(FrameSet *fs);
		

		/**
		* check tag validity
		*
		* \param[in,out] ROI *roi Region of interest to check
		*/
		virtual bool checkTag(ROI *roi)=0;


	protected : 
		/**
		* Save a frame
		* \param[in] std::string name unique name to store a frame
		* \param[in] const cv::Mat &mat frame to store
		*/
		void save(std::string name, cv::Mat &mat);

		
		/**
		* Load a frame, throw ARAMException if this frame doesn't exist
		* \param[in] std::string name unique name to load
		*/
		cv::Mat & load(std::string name);

	
		/**
		* Test if a frame name is used
		* \param[in] std::string name frame name
		* \return bool true if name is found
		*/
		bool exist(std::string name);
		
		
		/**
		* Rotate CV_8UC1 matrix (clock wise)
		*
		* \param[in] cv::Mat & matrix to rotate  
		* \param[out] cv::Mat & matrix after rotation
		*/
		void rotate(cv::Mat &, cv::Mat &);


	private :
		FrameSet *p_fs; /** < FrameSet contains all current frame created by the library */
	};
};

#endif

