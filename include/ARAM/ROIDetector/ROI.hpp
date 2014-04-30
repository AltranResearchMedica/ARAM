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
* \file ROI.hpp
* \brief Region of interest
* \author Alexandre Kornmann
* \version 1.3
* \date 14 mars 2014
*
*/

#ifndef _ROI_HPP_
#define _ROI_HPP_

//std include
#include <string>

//ARAM include
#include <ARAM/export.hpp>
#include <ARAM/typedef.hpp>
#include <ARAM/ARAMException.hpp>

//openCV include
#include <opencv2/core/core.hpp>

namespace aram
{
	/** 
	* Region of interest
	*/
	class ARAM_EXPORT ROI
	{
	public :
		/**
		* Push a vector of Point2D, clear current vector
		*
		* \param[in] vecPoint2D & new vector to store
		*/
		void corners(const vecPoint2D &);


		/**
		* Push a Point2D
		*
		* \param[in] const Point2D & new Point2f to store
		*/
		void corners(const Point2D &);


		/**
		* Get corners positions
		*
		* \return vecPoint2D corners positions (if 4 valid points store, throw an exception if not)
		*/
		vecPoint2D corners() const;


	private :
		vecPoint2D _corners; /**< corners positions */

	};
};

#endif