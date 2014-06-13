#include <ARAM/ROIDetector/LineFitting.hpp>


namespace aram
{
	LineFitting::LineFitting()
	{			
		_sigmaX = std::sqrt(2.0);
		_sigmaY = std::sqrt(2.0);
		_blurSize = cv::Size(15,15);
		
		_lowThreshold = 63;
		_highThreshold = 105;
		_sobelSize = 3;
		
		_factEpsilon = 0.05f;
		_minPerimeter = 40.0;
		_minArea = 200.0;

		_reps = 0.01;
		_aeps = 0.01;
		_distType = CV_DIST_L2;
	}

	void LineFitting::findROI(vecROI *rois, vecTag *, FrameSet *fs)
	{	
		cv::Mat currentFrame, grayscale, blur, canny, closure, findContour;
			
		// Current frame
		currentFrame = fs->load("currentFrame");
		// Grayscale
		cv::cvtColor(currentFrame, grayscale, CV_BGR2GRAY);
		// Gaussian blur
		cv::GaussianBlur(grayscale, blur, _blurSize, _sigmaX, _sigmaY);
		// Canny
		cv::Canny(blur, canny, _lowThreshold, _highThreshold, _sobelSize);
		// Naive closure
		naiveClosure(canny,closure);

		// findContours out values
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;

		// findContours
		cv::findContours(closure, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0,0));
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
			if(perim>_minPerimeter)
			{
				double area = cv::contourArea(contours[i]);
				if(area>_minArea)
				{				
					// approxPolyDP : Ramer–Douglas–Peucker algorithm
					cv::approxPolyDP(contours[i], curve, double(contours[i].size())*_factEpsilon, true);

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
		
		// for each valid curve
		for(unsigned int i=0;i<initialCurves.size();i++)
		{
			lines.clear();
			lines.resize(4);
			lineFitted.clear();
			lineFitted.resize(4);

			int line = 0;

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
				fitLine(lines[l], lineFitted[l], _distType, 0, _reps, _aeps);
			}
			
			// build ROI
			ROI *r = new ROI;
			for(unsigned int l=0;l<lineFitted.size();l++)
			{
				cv::Point inter = Intersect(lineFitted[l],lineFitted[(l+1)%4]);
				r->corners(inter);
			}

			// add ROI to ROI list in detector
			rois->push_back(r);
			
		}
		
		return;
	}

	cv::Point LineFitting::Intersect(Line a, Line b)
	{
		double u0 = a[0];
		double v0 = a[1];
		double x0 = a[2];
		double y0 = a[3];
		
		double u1 = b[0];
		double v1 = b[1];
		double x1 = b[2];
		double y1 = b[3];

		double len = 100.0;

		double x10 = x0;
		double x11 = x0+len*u0; 
		double y10 = y0;
		double y11 = y0+len*v0; 

		double x20 = x1;
		double x21 = x1+len*u1; 
		double y20 = y1;
		double y21 = y1+len*v1; 

		double denom = (x10-x11)*(y20-y21)-(y10-y11)*(x20-x21);

		cv::Point intersect(
								(int) (((x10*y11-y10*x11)*(x20-x21)-(x10-x11)*(x20*y21-y20*x21))/denom),
								(int) (((x10*y11-y10*x11)*(y20-y21)-(y10-y11)*(x20*y21-y20*x21))/denom)
							);

		return intersect;
	}

	float LineFitting::orthoDistance(cv::Point pt, Line l)
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
	
	void LineFitting::naiveClosure(const cv::Mat &in, cv::Mat &out)
	{	
		out = cv::Mat::zeros(in.size(), in.type());

		// use 1 and rows-1/cols-1 to avoid border issue
		for(int i=1; i<in.rows-1; i++)
		{
			for(int j=1; j<in.cols-1; j++)
			{
				int pix4 = in.at<uchar>(i,j);

				if(pix4==0)
				{
					// look around
					//
					// 0 | 1 | 2
					// ---------
					// 3 | 4 | 5
					// ---------
					// 6 | 7 | 8
					//

					int pix0 = in.at<uchar>(i-1,j-1);
					int pix1 = in.at<uchar>(i-1,j);
					int pix2 = in.at<uchar>(i-1,j+1);
					int pix3 = in.at<uchar>(i,j-1);
					int pix5 = in.at<uchar>(i,j+1);
					int pix6 = in.at<uchar>(i+1,j-1);
					int pix7 = in.at<uchar>(i+1,j);
					int pix8 = in.at<uchar>(i+1,j+1);

					int mean = pix0+pix1+pix2+pix3+pix4+pix5+pix6+pix7+pix8;
					mean/=9;

					bool res = false;
					if(pix0!=0 && (pix5!=0||pix7!=0||pix8!=0)) res = true;
					if(pix1!=0 && (pix6!=0||pix7!=0||pix8!=0)) res = true;
					if(pix2!=0 && (pix3!=0||pix6!=0||pix7!=0)) res = true;
					if(pix3!=0 && (pix5!=0||pix5!=0||pix8!=0)) res = true;
					if(pix5!=0 && (pix0!=0||pix3!=0||pix6!=0)) res = true;
					if(pix6!=0 && (pix1!=0||pix2!=0||pix5!=0)) res = true;
					if(pix7!=0 && (pix3!=0||pix1!=0||pix2!=0)) res = true;
					if(pix8!=0 && (pix0!=0||pix1!=0||pix3!=0)) res = true;

					if(res) out.at<uchar>(i,j) = mean;
				}
				else out.at<uchar>(i,j) = in.at<uchar>(i,j);
			}
		}
	
		return;
	}
};
