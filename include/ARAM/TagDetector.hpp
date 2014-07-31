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

//ARAM include
#include <ARAM/export.hpp>
#include <ARAM/typedef.hpp>
#include <ARAM/ARAMException.hpp>

#include <ARAM/FrameSet.hpp>
#include <ARAM/ROIDetector/IROIDetector.hpp>
#include <ARAM/tag/ITagMatcher.hpp>

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
		* Instantiate the ROIs detector and a frame set storage
		*/
		TagDetector()
		{
			p_fs = new FrameSet();
			p_roiDetect = new ROIDetector(p_fs);
			p_tagMatch = new TagType(p_fs);
			reset();
		}
				
		
		/**
		* Destructor
		* Free memory for ROIs and tags previously allocated, using reset function
		*/
		~TagDetector()
		{
			reset();
			delete p_tagMatch;
			delete p_roiDetect;
			delete p_fs;
		}


		/**
		* Reset ROIs list and Tags list, free dynamically allocated ROIs and tags
		*/
		void reset()
		{
			// clean ROIs
			iteratorROI itRoi;
			for(itRoi=m_rois.begin();itRoi!=m_rois.end();++itRoi)
			{
				delete *itRoi;
			}
			m_rois.clear();
			
			// clean tags
			// do not release tag vector content, because it's already done with roi
			m_tags.clear();
			
			// clean frameSet
			p_fs->reset();
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

			// current frame copy
			p_fs->save("currentFrame",frame);

			// ROI detection
			p_roiDetect->findROI(&m_rois);

			// Tag computing
			iteratorROI it;
			for(it=m_rois.begin();it!=m_rois.end();++it)
			{
				if(p_tagMatch->checkTag(*it)) m_tags.push_back(*it);
			}
			
			return;
		}

		/**
		* Begin of valid tags list
		*
		* \return iterator to the begin of tag list
		*/
		iteratorTag begin()
		{
			return m_tags.begin();
		}


		/**
		* End of valid tags list
		*
		* \return iterator to the end of tag list
		*/
		iteratorTag end()
		{
			return m_tags.end();
		}

		/**
		* Getter
		* 
		* \return FrameSet * current frame set
		*/
		FrameSet * frameSet() const
		{
			return p_fs;
		}


		/**
		* Getter
		* 
		* \return vecROI * current ROI vector
		*/
		vecROI * ROI() const
		{
			return &m_rois;
		}


		/**
		* Getter
		* 
		* \return vecTag * current tags vector
		*/
		vecTag * tag()
		{
			return &m_tags;
		}


        /**
        * Getter
        *
        * \return IROIDetector * current detector
        */
        IROIDetector *detector()
        {
            return p_roiDetect;
        }


        /**
        * Getter
        *
        * \return ITagMatcher * current sampler
        */
        ITagMatcher *sampler()
        {
            return p_tagMatch;
        }



	private :
		ROIDetector *p_roiDetect; /**< Region of interest detection method */
		TagType *p_tagMatch;
		FrameSet *p_fs; /**< Store a set of all frame used for this detection */

		vecROI m_rois; /**< Store ROIs found */
		vecTag m_tags; /**< Store tags found */
	};
};

#endif
