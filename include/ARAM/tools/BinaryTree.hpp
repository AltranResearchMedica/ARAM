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
* \file BinaryTree.hpp
* \author Alexandre Kornmann
* \version 1.0
* \date 01 avril 2014
*
*/

#ifndef _BINARYTREE_HPP_
#define _BINARYTREE_HPP_

//std include
#include <bitset>

//ARAM include
#include <ARAM/export.hpp>
#include <ARAM/typedef.hpp>
#include <ARAM/ARAMException.hpp>

//openCV include
#include <opencv2/opencv.hpp>

namespace aram
{
	/**
	* Binary tree's node
	*/
	class Node
	{
	public :
		/**
		* Cconstructor
		*
		* \param[in] int node value (0/1)
		*/
		Node(int);
		
		
		Node *_left; /**< left soon (0 branch)*/
		Node *_right; /**< right soon (1 branch) */
		int _value; /**< node value (0/1) */
	};


	/**
	* Binary tree structure for search operations, implements pattern singleton
	*/
	class BinaryTree
	{
	public :
		/**
		* Search a value in tree
		*
		* \param[in] cv::Mat CV_8UC1 square matrix 9x9
		* \return bool true if found
		*/
		bool search(cv::Mat &);


		/**
		* Search a value in tree
		*
		* \param[in] cv::Mat CV_8UC1 square matrix 9x9
		* \param int hamming distance tolerance
		* \return tag id if found, -1 if not
		*/
		int hammingSearch(cv::Mat &, int);
		
		
		/**
		* Pattern singleton implementation, return an unique instance of BinaryTree
		*
		* \return unique instance of BinaryTree
		*/
		static BinaryTree & getInstance();

	private :
		/**
		* Insert a new value in tree
		*
		* \param[in] std::bitset<81> contains marker in linear form
		*/
		void insert(std::bitset<81>);

		
		/**
		* Compute hamming distance between two bitset
		*
		* \param[in] std::bitset<81> a first bitset for comparaison
		* \param[in] std::bitset<81> b second bitset for comparaison
		* \return int hamming distance beetween two bitset
		*/
		int hammingDistance(std::bitset<81>, std::bitset<81>);

		/**
		* Build tree
		*/
		void read();
		
		
		/**
		* Constructor
		*/
		BinaryTree();


		/**
		* Assignement operator
		*/
		BinaryTree& operator=(const BinaryTree&);


		/**
		* Copy constructor
		*/
		BinaryTree(const BinaryTree&);

	
		static BinaryTree _instance; /**< unique instance of BinaryTree */

		Node *_root; /**< root node, for binary tree search */
		std::vector< std::bitset<81> > _sets; /**< bitsets storage, for hamming search */
	};
};
#endif