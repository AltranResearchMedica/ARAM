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
* \file Exporter.hpp
* \brief Tools for info export
* \author Alexandre Kornmann
* \version 1.0
* \date 15 avril 2014
* 
*/
#ifndef _EXPORTER_HPP_
#define _EXPORTER_HPP_

//std include
#include <iostream>
#include <fstream>

//Open CV include
#include <opencv2/opencv.hpp>

//ARAM include
#include <ARAM/export.hpp>
#include <ARAM/typedef.hpp>
#include <ARAM/ARAMException.hpp>


namespace aram
{
	/**
	* Tools for info export
	*/
	class ARAM_EXPORT Exporter
	{
	public :
		/**
		* Constuctor
		*/
		Exporter();


		/**
		* ++ operator overload
		*/
		Exporter& operator++();
					
		
		/**
		* Save time value
		*
		* \param[in] float time time value
		* \param[in] const std::string &file file name
		*/
		void timer(float, const std::string &);


		/**
		* Save occurence value
		*
		* \param[in] int occ occurence value
		* \param[in] const std::string &file file name
		*/
		void occurence(int, const std::string &);


		/**
		* Save error value (norm)
		*
		* \param[in] float err error value
		* \param[in] const std::string &file file name
		*/
		void error(float, const std::string &);

			
		/**
		* Save error value (two points)
		*
		* \param[in] float x1
		* \param[in] float y1
		* \param[in] float x2
		* \param[in] float y2
		* \param[in] const std::string &file file name
		*/
		void error(float, float, float, float, const std::string &);
				

		/**
		* Save frame in a file (.png)
		*
		* \param[in] const cv::Mat frame img to capture
		* \param[in] const std::string &file file name
		*/
		void frame(cv::Mat &, const std::string &);

	private :
		/**
		* Write in a file
		*
		* \param[in] const std::string &text content to write
		* \param[in] const std::string &file file name
		*/
		void write(const std::string &, const std::string &);


		int _count; /** < Frame counter */
	};
};
#endif
