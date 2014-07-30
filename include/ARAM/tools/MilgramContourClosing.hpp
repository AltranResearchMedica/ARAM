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
* \file MilgramContourClosing.hpp
* \brief Contours closing using Milgram algorithm
* \author Alexandre Kornmann
* \version 1.0
* \date 23 juillet 2014
*
* Milgram closure
*/

#ifndef _MILGRAMCONTOURCLOSING_HPP_
#define _MILGRAMCONTOURCLOSING_HPP_


//ARAM include
#include <ARAM/export.hpp>
#include <ARAM/typedef.hpp>
#include <ARAM/ARAMException.hpp>


namespace aram
{
	/**
	* Closing operation for binary image, based on Milgram algorithm
	* M. Milgram and J.-P. Cocquerez, Contour closing with local operator, 1986, Traitement du Signal, volume 3, numero 6
	*/
	class MilgramContourClosing
	{
	public :
		/**
		* Close a binary image using Milgram algorithm
		*
		* \param[in,out] cv::Mat &src binary image to close
		*/
		void close(cv::Mat &src);


		/**
		* Pattern singleton implementation, return an unique instance of MilgramContourClosing
		*
		* \return unique instance of MilgramContourClosing
		*/
		static MilgramContourClosing & getInstance();


	private :
		/**
		* Constructor
		*/
		MilgramContourClosing(int limit=5, int iter=1);


		/**
		* Assignement operator
		*/
		MilgramContourClosing& operator=(const MilgramContourClosing&);


		/**
		* Copy constructor
		*/
		MilgramContourClosing(const MilgramContourClosing&);

	
		/**
		* Look Up Table creation
		*/
		void initLUT();


		/**
		* Check if a point is a saddle point, for LUT creation (isSelle sub function)
		*
		* \param[in] std::vector<int> &data mask of 8 neighbourgs pixels
		*/
		bool checkSelle(int a, int b, int c, int d, std::vector<int> &data);
		
		/**
		* Check if a point is a saddle point, for LUT creation
		*
		* \param[in] std::vector<int> &data mask of 8 neighbourgs pixels
		*/
		bool isSelle(std::vector<int> &data);
		

		/**
		* Compute A value (as describe in Milgram paper) : only use for LUT creation
		*
		* \param[in] std::vector<int> &data mask of 8 neighbourgs pixels
		*/
		int computeA(std::vector<int> &data);

		
		/**
		* Update contours in canny reading an updated distance map
		*
		* \param[in] cv::Mat& canny canny image to update
		* \param[out] cv::Mat& dist distance map
		*/
		void updateCanny(cv::Mat &canny, cv::Mat &dist);
		
		
		/**
		* Find saddle point in distance map, using LUT, and setting it to zero
		*
		* \param[in, out] cv::Mat& dist distance map
		*/
		bool updateSelle(cv::Mat &dist);


		/**
		* Initialize distance map, from a binary image
		*
		* \param[in] cv::Mat& in binary image
		* \param[out] cv::Mat& dist distance map
		*/
		void initDistanceMap(cv::Mat& in, cv::Mat& dist);


		/**
		* update chamfer distance map (double scanning process)
		*
		* \param[in,out] distance map to update
		*/
		void updateDistanceMap(cv::Mat &dist);


		static MilgramContourClosing s_instance; /**< unique instance of TagDictionnary */

		std::vector<bool> *p_LUT; /**< Look up table */

		int m_limit; /**< if a saddle point is far away than m_limit, he's ignored */ 
		int m_iter; /**< Number of iteration of the process */
	};
};

#endif