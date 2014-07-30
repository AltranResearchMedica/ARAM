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

//ARAM include
#include <ARAM/export.hpp>
#include <ARAM/typedef.hpp>
#include <ARAM/ARAMException.hpp>

#include <ARAM/tools/Extrinsic.hpp>


namespace aram
{
	/** 
	* Region of interest
	*/
	class ARAM_EXPORT ROI
	{
	public :
		/**
		* Constructor
		*/
		ROI();

		/**
		* Push a vector of Point2D, clear current vector
		*
		* \param[in] vecPoint2D &pts new vector to store
		*/
		void corners(const vecPoint2D &pts);


		/**
		* Push a Point2D
		*
		* \param[in] const Point2D &pt new Point2f to store
		*/
		void corners(const Point2D &pt);


		/**
		* Get corners positions
		*
		* \return vecPoint2D corners positions (if 4 valid points store, throw an exception if not)
		*/
		vecPoint2D & corners();
		
		
		/**
		* Rotate n times corners list (clock wise)
		* 
		* \param[in] int n number of rotation
		*/
		void rotate(int n);
		

		/**
		* Compute extrinsic parameter associeted with this tag
		* 
		* \param[in] Intrinsic & intrinsic parameters
		* \param[in] float size tag size (in user define unit, for example millimeters)
		* \return Extrinsic & rotation matrix
		*/
		Extrinsic extrinsic(const Intrinsic &intr, float size);


		/**
		* Getter
		* Unique id for this marker
		*
		* \return int id of this marker
		*/
		int id() const;


		/**
		* Setter
		* Unique id for this marker
		*
		* \param[in] int id of this marker
		*/
		void id(int i);


	private :
		vecPoint2D m_corners; /**< corners positions */

		int m_id; /**< Tag id (-1 if it's not a tag) */
	};
};

#endif