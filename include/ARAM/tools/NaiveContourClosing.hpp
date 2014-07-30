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
* \file NaiveContourClosing.hpp
* \brief Contours closing by a trivial method.
* \author Alexandre Kornmann
* \version 1.0
* \date 24 juin 2014
*
* Naive closure
*/

#ifndef _NAIVECONTOURCLOSING_HPP_
#define _NAIVECONTOURCLOSING_HPP_


//ARAM include
#include <ARAM/export.hpp>
#include <ARAM/typedef.hpp>
#include <ARAM/ARAMException.hpp>

namespace aram
{
	/**
	* Naive closure inspect neighbours in a 3x3 region, and check if two contours pixels are already detected and aligned with current pixel
	*/
	class NaiveContourClosing
	{
	public:
		/**
		* close all hole in contours (1 pixel retrieve maximum)
		*
		* \param[in,out] binary image (CV_8U1C) to close
		*/
		void close(cv::Mat &src);


	private:
		/**
		* check 3x3 mask
		*
		* 3 | 2 | 1
		* ---------
		* 4 | 8 | 0
		* ---------
		* 5 | 6 | 7
		*
		* \param[in] 3x3 mask (0->7) values
		* \return true if a pixel 8 is a contour
		*/
		bool checkEdge(int data[8]);
	};
};
#endif