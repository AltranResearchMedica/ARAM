#include <ARAM/ROIDetector/EdgeDetector.hpp>


namespace aram
{
	void EdgeDetector::findROI(vecROI *rois, vecTag *, FrameSet *fs)
	{
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
	
		cv::Mat currentFrame, grayscale, binary, canny, findContour;
		currentFrame = fs->load("currentFrame");

		cv::Canny(currentFrame,canny,33,156);
		fs->save("canny",canny);
		
		cv::findContours(canny, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));
		

		std::vector<cv::Point> approxCurve;

		for(unsigned int i=0;i<contours.size();++i)
		{
			double d = cv::arcLength(contours[i],true);

			//PERIMETER VALUE ???
			if(d>40.0)
			{
				//EPSILON VALUE ???
				cv::approxPolyDP(contours[i], approxCurve, double(contours[i].size())*0.1, true);

				contours[i] = approxCurve;
				if(contours[i].size()==4)
				{
					if(cv::isContourConvex(contours[i]))
					{
						std::vector<cv::Point>::const_iterator it;
						std::vector<Point2D> corn;
						for(it=contours[i].begin();it!=contours[i].end();++it)
						{
						  Point2D pt;
						  pt.x = (*it).x;
						  pt.y = (*it).y;
						  corn.push_back(pt);
						}

						ROI *r = new ROI;
						r->corners(corn);
						rois->push_back(r);
					}
				}
			}
		}

		return;

	}
};
