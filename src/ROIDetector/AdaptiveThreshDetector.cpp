#include <ARAM/ROIDetector/AdaptiveThreshDetector.hpp>


namespace aram
{
	AdaptiveThreshDetector::AdaptiveThreshDetector(FrameSet *fs):IROIDetector(fs),m_epsilon(0.05f),m_blockSize(7),m_constant(7.0),m_minPerimeter(40.0)
	{

	}

	void AdaptiveThreshDetector::findROI(vecROI *rois)
	{
		cv::Mat currentFrame, grayscale, threshold, findContour;
		currentFrame = load("currentFrame");

		// frame to grayscale
		cv::cvtColor(currentFrame, grayscale, CV_BGR2GRAY);
		
#ifdef EXPORT_FRAME
		save("grayscale",grayscale);
#endif

		// grayscale to threshold
		cv::adaptiveThreshold(grayscale, threshold, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, m_blockSize, m_constant);
		
#ifdef EXPORT_FRAME
		threshold = 255*threshold;
		save("adaptiveThreshold",threshold);
#endif

		// compute all contour in thresholded frame
		// use CV_RETR_EXTERNAL to avoid double contour finding
		// use CV_CHAIN_APPROX_SIMPLE to compress contours (we don't need every contours points in this case)
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(threshold, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));

#ifdef EXPORT_FRAME
		cv::Mat drawingRois = cv::Mat::zeros(threshold.size(), CV_8UC3);
		cv::Mat drawingContour = cv::Mat::zeros(threshold.size(), CV_8UC3);


		for(unsigned int i=0; i<contours.size(); i++)
		{
			cv::Scalar color = cv::Scalar(0, 100, 200);
			drawContours(drawingContour, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
		}

		save("Contours",drawingContour);
#endif	

		std::vector<cv::Point> approxCurve;
		std::vector<std::vector<cv::Point> > approxCurves;

		for(unsigned int i=0; i<contours.size(); ++i)
		{
			double d = cv::arcLength(contours[i],true);

			if(d>m_minPerimeter)
			{
				cv::approxPolyDP(contours[i], approxCurve, double(contours[i].size())*m_epsilon, true);
				
				approxCurves.push_back(approxCurve);

				//contours[i] = approxCurve;
				if(approxCurve.size()==4)
				{
					if(cv::isContourConvex(approxCurve))
					{
						// build new ROI
						ROI *r = new ROI;

						std::vector<cv::Point>::const_iterator it;
						for(it=approxCurve.begin(); it!=approxCurve.end(); ++it)
						{
						  Point2D pt;
						  pt.x = (float) (*it).x;
						  pt.y = (float) (*it).y;
						  r->corners(pt);
						}
						
						// add ROI in ROIs list
						rois->push_back(r);

#ifdef EXPORT_FRAME
						std::vector<Point2D> corners = r->corners();
						for(unsigned int i=0; i<corners.size(); i++)
						{
							cv::Scalar color = cv::Scalar(50, 200, 255);
							cv::line(drawingRois, corners[i], corners[(i+1)%4], color, 2);
						}

#endif
					}
				}
			}
		}
		
#ifdef EXPORT_FRAME
		save("Region of interest",drawingRois);
#endif

		return;
	}


	// setter/getter

	float AdaptiveThreshDetector::epsilon() const
	{
		return m_epsilon;
	}

	void AdaptiveThreshDetector::epsilon(float e)
	{
		// 0.0 < e < 1.0
		m_epsilon = (e<0.0f) ? 0.0f : ((e>1.0f) ? 1.0f : e);
		return;
	}

	int AdaptiveThreshDetector::blockSize() const
	{
		return m_blockSize;
	}

	void AdaptiveThreshDetector::blockSize(int n)
	{
		// n = 3, 5, 7, 9, and so on
		m_blockSize = (n<3) ? 3 : ((n%2==0) ? n+1 : n);
		return;
	}

	double AdaptiveThreshDetector::adaptiveConstant() const
	{
		return m_constant;
	}

	void AdaptiveThreshDetector::adaptiveConstant(double c)
	{
		m_constant = c;
	}

	double AdaptiveThreshDetector::minPerimeter() const
	{
		return m_minPerimeter;
	}

	void AdaptiveThreshDetector::minPerimeter(double p)
	{
		m_minPerimeter = (p<0.0) ? 0.0 : p;
	}
};
