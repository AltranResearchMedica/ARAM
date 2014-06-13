#include <ARAM/ROIDetector/EdgeDetector.hpp>


namespace aram
{
	EdgeDetector::EdgeDetector()
	{
		_factEpsilon = 0.05f;
		_blockSize = 7;
		_constant = 7.0;
		_minPerimeter = 40.0;
	}

	void EdgeDetector::findROI(vecROI *rois, vecTag *, FrameSet *fs)
	{
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
	
		
		cv::Mat currentFrame, grayscale, threshold, findContour;
		currentFrame = fs->load("currentFrame");

		cv::cvtColor(currentFrame, grayscale, CV_BGR2GRAY);
		cv::adaptiveThreshold(grayscale, threshold, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, _blockSize, _constant);
				
		cv::findContours(threshold, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0,0));
		

		std::vector<cv::Point> approxCurve;
		std::vector<std::vector<cv::Point> > approxCurves, initialCurves;

		for(unsigned int i=0;i<contours.size();++i)
		{
			double d = cv::arcLength(contours[i],true);

			if(d>_minPerimeter)
			{
				cv::approxPolyDP(contours[i], approxCurve, double(contours[i].size())*_factEpsilon, true);
				
				initialCurves.push_back(contours[i]);
				approxCurves.push_back(approxCurve);

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
						  pt.x = (float) (*it).x;
						  pt.y = (float) (*it).y;
						  corn.push_back(pt);

						}

						ROI *r = new ROI;
						r->corners(corn);
						rois->push_back(r);
					}
				}
			}
		}
#ifdef DEBUG	
		cv::RNG rng(12345);

		cv::Mat drawing1 = cv::Mat::zeros(threshold.size(),CV_8UC3);
		for(unsigned int i=0;i<approxCurves.size();i++)
		{
			cv::Scalar color = cv::Scalar(rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255));
			unsigned int j;
			for(j=0;j<approxCurves[i].size()-1;j++)
			{
				cv::line(drawing1,approxCurves[i][j],approxCurves[i][j+1],color,1);
			}
			cv::line(drawing1,approxCurves[i][j],approxCurves[i][0],color,1);
		}
		fs->save("approxCurve",drawing1);

		cv::Mat drawing2 = cv::Mat::zeros(threshold.size(),CV_8UC3);
		for(unsigned int i=0;i<initialCurves.size();i++)
		{
			unsigned int j;
			cv::Scalar color = cv::Scalar(rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255));
			for(j=0;j<initialCurves[i].size()-1;j++)
			{
				cv::line(drawing2,initialCurves[i][j],initialCurves[i][j+1],color,1);
			}
			cv::line(drawing2,initialCurves[i][j],initialCurves[i][0],color,1);
		}
		fs->save("initialCurves",drawing2);
		
		
		std::cout << "Initial curves : " << initialCurves.size() << std::endl;
		std::cout << "Approx curves : " << approxCurves.size() << std::endl;
		
		cv::Mat drawing3 = cv::Mat::zeros(threshold.size(),CV_8UC3);
		for(unsigned int i=0;i<rois->size();i++)
		{
			cv::Scalar color = cv::Scalar(rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255));
			vecPoint2D cor = rois->at(i)->corners();
			for(unsigned int j=0;j<cor.size();j++)
			{
				cv::line(drawing3,cor[j],cor[(j+1)%4],color,1);
			}
		}
		fs->save("roi",drawing3);
#endif
		return;

	}
};
