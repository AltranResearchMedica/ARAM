#include <ARAM/ROIDetector/OtsuCannyFittingDetector.hpp>

namespace aram
{
	OtsuCannyFittingDetector::OtsuCannyFittingDetector(FrameSet* fs):
	IROIDetector(fs),
	m_sigmaX(std::sqrt(2.0)), m_sigmaY(std::sqrt(2.0)), m_blurSize(cv::Size(15,15)),
	m_lowThreshold(0), m_highThreshold(0), m_sobelSize(3),
	m_epsilon(0.05f), m_minPerimeter(40.0), m_minArea(200.0),
	m_reps(0.01), m_aeps(0.01), m_distType(CV_DIST_L2)
	{			
	}

	void OtsuCannyFittingDetector::findROI(vecROI *rois)
	{	
		cv::Mat currentFrame, grayscale, otsu, blur, canny, closure, findContour;
			
		// Current frame
		currentFrame = load("currentFrame");
		// Grayscale
		cv::cvtColor(currentFrame, grayscale, CV_BGR2GRAY);
		// Otsu
		double t = cv::threshold(grayscale, otsu, 0, 255, cv::THRESH_BINARY_INV | CV_THRESH_OTSU);
		
		// Gaussian blur
		cv::GaussianBlur(grayscale, blur, m_blurSize, m_sigmaX, m_sigmaY);

		m_lowThreshold = (int) (t/2.0);
		m_highThreshold = (int) t;
		// Canny
		cv::Canny(blur, canny, m_lowThreshold, m_highThreshold, m_sobelSize);

		save("Otsu", otsu);

#ifdef EXPORT_FRAME
		cv::Mat saveCanny = 255*canny;
		save("Edge detection",saveCanny);
#endif
		// Milgram closure
		close(canny,ARAM_MilgramContourClosing);
		
#ifdef EXPORT_FRAME
		cv::Mat saveClosure = 255*canny;
		save("Contour closing",saveClosure);
#endif

		// findContours out values
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;

		// compute all contour in binary frame
		// use CV_RETR_EXTERNAL to avoid double contour finding
		// use CV_CHAIN_APPROX_NONE to keep all contours points (we need every contours points in this case)
		cv::findContours(canny, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0,0));
		
#ifdef EXPORT_FRAME
		cv::Mat drawingRois = cv::Mat::zeros(canny.size(), CV_8UC3);
		cv::Mat drawingContour = cv::Mat::zeros(canny.size(), CV_8UC3);


		for(unsigned int i=0; i<contours.size(); i++)
		{
			cv::Scalar color = cv::Scalar(0, 100, 200);
			drawContours(drawingContour, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
		}

		save("Edge linking",drawingContour);
#endif
		
		// list of all valid approximated curves
		std::vector<std::vector<cv::Point> > approxCurves;
		// list of all valid intial curves (before approxPolyDP)
		std::vector<std::vector<cv::Point> > initialCurves;
		
		// approxPolyDP out values
		std::vector<cv::Point> curve;
				
		// for each contours found
		for(unsigned int i=0;i<contours.size();++i)
		{
			double perim = cv::arcLength(contours[i],true);

			// Perimeter filter
			if(perim>m_minPerimeter)
			{
				double area = cv::contourArea(contours[i]);
				if(area>m_minArea)
				{				
					// approxPolyDP : Ramer–Douglas–Peucker algorithm
					cv::approxPolyDP(contours[i], curve, double(contours[i].size())*m_epsilon, true);

					// 4 corners = quadrangle
					if(curve.size()==4)
					{
						// convexity filter
						if(cv::isContourConvex(curve))
						{
							// add to valid lists
							approxCurves.push_back(curve);
							initialCurves.push_back(contours[i]);
						}
					}
				}
			}
		}

		// 4 lines
		std::vector<std::vector<cv::Point> > lines;
		// list of line after fitting
		std::vector<Line> lineFitted;
		lines.resize(4);
		lineFitted.resize(4);
		
		int line = 0;
		// for each valid curve
		for(unsigned int i=0;i<initialCurves.size();i++)
		{
			lines.clear();
			lineFitted.clear();
			lines.resize(4);
			lineFitted.resize(4);

			line = 0;

			// for every points in initial curve
			for(unsigned int j=0;j<initialCurves[i].size();j++)
			{
				// current point
				cv::Point currPoint = initialCurves[i][j];

				// add current point to current line
				lines[line].push_back(currPoint);

				// if current point is a corner
				if(currPoint==approxCurves[i][0] || currPoint==approxCurves[i][1] || currPoint==approxCurves[i][2] || currPoint==approxCurves[i][3])
				{
					// use next line
					line = (line+1)%4;
					// add current point to next line
					lines[line].push_back(currPoint);
				}
			}
			// for every 4 lines
			for(unsigned int l=0;l<lines.size();l++)
			{
				fitLine(lines[l], lineFitted[l], m_distType, 0, m_reps, m_aeps);
			}
			
			
			// build ROI
			ROI *r = new ROI;
			for(unsigned int l=0;l<lineFitted.size();l++)
			{
				Point2D inter = Intersect(lineFitted[l],lineFitted[(l+1)%4]);
				r->corners(inter);
			}

			// add ROI to ROI list in detector
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

#ifdef EXPORT_FRAME
		save("Line fitting",drawingRois);
#endif

		return;
	}

	Point2D OtsuCannyFittingDetector::Intersect(Line a, Line b)
	{
		float u0 = a[0];
		float v0 = a[1];
		float x0 = a[2];
		float y0 = a[3];
		
		float u1 = b[0];
		float v1 = b[1];
		float x1 = b[2];
		float y1 = b[3];

		float len = 100.0;

		float x10 = x0;
		float x11 = x0+len*u0; 
		float y10 = y0;
		float y11 = y0+len*v0; 

		float x20 = x1;
		float x21 = x1+len*u1; 
		float y20 = y1;
		float y21 = y1+len*v1; 

		float denom = (x10-x11)*(y20-y21)-(y10-y11)*(x20-x21);

		Point2D intersect(
								(((x10*y11-y10*x11)*(x20-x21)-(x10-x11)*(x20*y21-y20*x21))/denom),
								(((x10*y11-y10*x11)*(y20-y21)-(y10-y11)*(x20*y21-y20*x21))/denom)
							);

		return intersect;
	}

	float OtsuCannyFittingDetector::orthoDistance(cv::Point pt, Line l)
	{
		float len = 100.0;
		float mu = l[2];
		float mv = l[3];
		float mpu = l[2]+len*l[0];
		float mpv = l[3]+len*l[1];
				
		float a = mv-mpv;
		float b = mpu-mu;
		float c = -(b*mv+a*mu);

		float res = std::abs(a*pt.x+b*pt.y+c);
		res /= std::sqrt(a*a+b*b);

		return res;
	}



	// setter / getter
	float OtsuCannyFittingDetector::epsilon() const
	{
		return m_epsilon;
	}

	void OtsuCannyFittingDetector::epsilon(float e)
	{
		// 0.0 < e < 1.0
		m_epsilon = (e<0.0f) ? 0.0f : ((e>1.0f) ? 1.0f : e);
		return;
	}

	double OtsuCannyFittingDetector::minPerimeter() const
	{
		return m_minPerimeter;
	}

	void OtsuCannyFittingDetector::minPerimeter(double p)
	{
		m_minPerimeter = (p<0.0) ? 0.0 : p;
		return;
	}

	double OtsuCannyFittingDetector::minArea() const
	{
		return m_minArea;
	}

	void OtsuCannyFittingDetector::minArea(double a)
	{
		m_minArea = (a<0.0) ? 0.0 : a;
		return;
	}

	double OtsuCannyFittingDetector::sigmaX() const
	{
		return m_sigmaX;
	}

	void OtsuCannyFittingDetector::sigmaX(double sX)
	{
		m_sigmaX = (sX<0.0) ? 0.0 : sX;
		return;
	}

	double OtsuCannyFittingDetector::sigmaY() const
	{
		return m_sigmaY;
	}

	void OtsuCannyFittingDetector::sigmaY(double sY)
	{
		m_sigmaY = (sY<0.0) ? 0.0 : sY;
		return;
	}

	cv::Size OtsuCannyFittingDetector::blurSize() const
	{
		return m_blurSize;
	}

	void OtsuCannyFittingDetector::blurSize(cv::Size s)
	{
		// width and height can be different, but must be positive and odd
		m_blurSize.height = (s.height<0) ? 0 : ((s.height%2==0) ? s.height+1 : s.height);
		m_blurSize.width = (s.width<0) ? 0 : ((s.width%2==0) ? s.width+1 : s.width);
		return;
	}

	int OtsuCannyFittingDetector::sobelSize() const
	{
		return m_sobelSize;
	}

	void OtsuCannyFittingDetector::sobelSize(int s)
	{
		m_sobelSize = (s==3||s==5||s==7) ? s : 3;
		return;
	}

	double OtsuCannyFittingDetector::reps() const
	{
		return m_reps;
	}

	void OtsuCannyFittingDetector::reps(double r)
	{
		m_reps = (r<0.0) ? 0.0 : r;
		return;
	}

	double OtsuCannyFittingDetector::aeps() const
	{
		return m_aeps;
	}

	void OtsuCannyFittingDetector::aeps(double a)
	{
		m_aeps = (a<0.0) ? 0.0 : a;
		return;
	}
};