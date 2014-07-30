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
* \file MultiTag.hpp
* \author Alexandre Kornmann
* \version 1.0
* \date 11 avril 2014
*
*/

#ifndef _MULTITAG_HPP_
#define _MULTITAG_HPP_

//openCV include
#include <opencv2/opencv.hpp>

//ARAM include
#include <ARAM/export.hpp>
#include <ARAM/typedef.hpp>
#include <ARAM/ARAMException.hpp>

#include <ARAM/coordinate/ICoordinate.hpp>


namespace aram
{
	/**
	* Information about tag in real world coordinates
	*/
	struct ARAM_EXPORT TagInfo
	{
		/**
		* Constructor
		*
		* \param[in] int i tag id
		* \param[in] Point2D o origin coordinate (bottom left corner)
		* \param[in] float s size of tag (always use the same unit !)
		*/
		TagInfo(int,Point2D,float);


		int id; /**< tag id */
		float size; /**< size in user defind unit */
		Point2D origin; /**< origin of tag in user define unit */
	};


	/*
	* Compute extrinsics parameters using a grid of tags
	* This method is efficient to deal with occlusion problems.
	By using tags with kowned positions in real world coordinates, you can compute other tags position by knowing only one of them, and then compute camera pose.
	*/
	class ARAM_EXPORT MultiTag : public ICoordinate
	{
	public :
		/**
		* Constructor
		*/
		MultiTag();
		
		/**
		* Compute extrinsics parameters and reprojection error
		* 
		* \param[in] iteratorTag begin iterator on begin of valid tag list
		* \param[in] iteratorTag end iterator on end of valid tag list
		* \param[in] Intrinsic intr Intrinsic parameters
		* \return Extrinsic extrinsics parameters
		*/
		Extrinsic compute(iteratorTag, iteratorTag, Intrinsic);

		/**
		* Add a TagTnfo to MultiTag
		* \param[in] TagInfo t TagInfo to add
		*/
		void addTagInfo(TagInfo);


		/**
		* Get a TagInfo by id
		* \param[in] int id tag id to get
		* \return TagInfo tag informations
		*/
		TagInfo getTagInfo(int);

		
	private :
		std::vector<TagInfo> _tags; /**<  store information abaout tags position in real world coordinates */
	};
};

#endif