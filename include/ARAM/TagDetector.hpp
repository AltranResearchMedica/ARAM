/**
*
* \file TagDetector.hpp
* \brief Main class of ARAM library
* \author Alexandre Kornmann
* \version 1.0
* \date 11 mars 2014
*
*/

#ifndef _TAGDETECTOR_HPP_
#define _TAGDETECTOR_HPP_


//std include 
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <ios>

//ARAM include
#include <ARAM/export.hpp>
#include <ARAM/typedef.hpp>
#include <ARAM/ARAMException.hpp>

#include <ARAM/FrameSet.hpp>
#include <ARAM/ROIDetector/IROIDetector.hpp>

#include <ARAM/tag/ITag.hpp>

#include <ARAM/tools/Exporter.hpp>

//openCV include
#include <opencv2/opencv.hpp>

namespace aram
{		
	/** 
	* Handle detection operation
	*/
	template <typename ROIDetector, typename TagType>
	class ARAM_EXPORT TagDetector
	{

	public :		
		/**
		* Constructor
		* Instantiate the ROIs detectector and a frame set storage
		*/
		TagDetector():
		_r(ROIDetector()),
		_fs(new FrameSet())
		{
			reset();
		}
				
		
		/**
		* Destructor
		* Free memory for ROIs and tags previously allocated, using reset function
		*/
		~TagDetector()
		{
			reset();
		}


		/**
		* Reset ROIs list and Tags list, free dynamically allocated ROIs and tags
		*/
		void reset()
		{
			iteratorTag itTag;
			iteratorROI itRoi;


			for(itRoi=_rois.begin();itRoi!=_rois.end();++itRoi)
			{
				delete *itRoi;
			}
			_rois.clear();
			
			for(itTag=_tags.begin();itTag!=_tags.end();++itTag)
			{
				delete *itTag;
			}
			_tags.clear();

			_fs->reset();
		}

		/**
		* Main method, take an image and return a list of tag found in this image
		*
		* \param[in] cv::Mat & input image 
		* \return std::vector<TagType> a list of tag found in cv::Mat, can be empty 
		*/
		void detect(const cv::Mat &frame)
		{
			reset();
			
			float freq = (float) cv::getTickFrequency()/1000.0f;
			
			int64 globaltimer, tagstimer, roistimer;

			globaltimer = cv::getTickCount();
			// current frame copy
			_fs->save("currentFrame",frame);
			
			roistimer = cv::getTickCount();
			// ROI detection
			_r.findROI(&_rois,&_tags,_fs);
			e.timer((float) ((cv::getTickCount()-roistimer)/freq), "rois");
			
			tagstimer = cv::getTickCount();
			// Tag computing
			iteratorROI it;
			for(it=_rois.begin();it!=_rois.end();++it)
			{
				TagType *t = new TagType(*(*it));
				
				if(t->checkTag(&_rois,&_tags,_fs)) _tags.push_back(t);
				else
				{
					delete t;
				}
			}

			e.timer((float) ((cv::getTickCount()-tagstimer)/freq), "tags");
			e.timer((float) ((cv::getTickCount()-globaltimer)/freq), "global");
			
			return;
		}

		/**
		* Begin of valid tags list
		*
		* \return iterator to the begin of tag list
		*/
		iteratorTag begin()
		{
			return _tags.begin();
		}


		/**
		* End of valid tags list
		*
		* \return iterator to the end of tag list
		*/
		iteratorTag end()
		{
			return _tags.end();
		}

		/**
		* Getter
		*/
		FrameSet * frameSet()
		{
			return _fs;
		}
	private :
		ROIDetector _r; /**< Region of interest detection method */

		FrameSet *_fs; /**< Store a set of all frame used for this detection */

		vecROI _rois; /**< Store ROIs found */
		vecTag _tags; /**< Store tags found */

		Exporter e;
	};
};

#endif