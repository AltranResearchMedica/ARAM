#ifndef _TAGGENERATOR_HPP_
#define _TAGGENERATOR_HPP_


#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <cstdlib>
#include <cmath>
#include <sstream>

class TagGenerator
{
public :
	/**
	* \brief generate a dictionnary with n tag, every tag contains sxs bits
	* 
	* \param int n number of tags to generate
	* \param int s size of tag to generate
	* \param int s size of border
	* \param int r number of retry before reduce hamming distance
	* \param int minDist initial value for hamming distance
	* \param int XMLFilename name of XML file
	*
	*/
	TagGenerator(int n, int s, int b, int r, int minDist, std::string XMLFilename);

private :
	/**
	* \brief generate a dictionnary with n tag, every tag contains sxs bits
	* 
	* \param int minDist initial value for hamming distance
	* \param int XMLFilename name of XML file
	*
	*/
	void generate(int minDist, std::string XMLFilename);

	/**
	* \brief Comput hamming distance beetween two tags
	* \param cv::Mat m first matrix
	* \param cv::Mat n second amtrix
	*
	*/
	int hammingDistance(cv::Mat m, cv::Mat n);

	/**
	* \brief Compute hamming distance to dictionnary
	* 
	* \param cv::Mat m matrix to compare with all existing tags
	*
	*/
	int hammingDistance(cv::Mat m);

	/**
	* \brief generate randomly a tag 
	* 
	* \param inout cv::Mat bits output matrix
	*
	*/
	void generateTag(cv::Mat &bits);

	/**
	* \brief Rotate a tag sxs
	* 
	* \param in cv::Mat &in matrix to rotate
	* 
	* \return rotated matrix
	*/
	cv::Mat rotate(cv::Mat &in);

private :
	int _tagNumber; /**< number of tags to generate*/
	int _tagSize; /**< size of tags to generate (in pixel) */
	int _borderSize; /** size of the border (in pixel) */
	int _retry;

	std::vector<cv::Mat> _tagList; /**< List of tag already generate */
};

#endif