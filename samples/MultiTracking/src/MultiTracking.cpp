#include <ARAM/TagDetector.hpp> // Main ARAM class
#include <ARAM/tag/LocalThreshTagMatcher.hpp> // Tag validator
#include <ARAM/ROIDetector/CannyFittingDetector.hpp> // Region of interest detection
#include <ARAM/coordinate/MultiTag.hpp> // Multi tracking

#include <opencv2/opencv.hpp> // OpenCV data structure

#include <exception> //std::exception

void drawPyramide(cv::Mat &mat, float size, aram::Point3D origin, cv::Scalar color, aram::Extrinsic extr)
{
    size *= 0.5;
    // Project 3D world coordinate -> 2D image coordinate
    aram::Point3D a(-size,-size,0.0);
    aram::Point3D b(-size,size,0.0);
    aram::Point3D c(size,size,0.0);
    aram::Point3D d(size,-size,0.0);
    aram::Point3D e(0.0,0.0,-size);

    a+=origin;
    b+=origin;
    c+=origin;
    d+=origin;
    e+=origin;

    aram::Point2D ap = extr.project(a);
    aram::Point2D bp = extr.project(b);
    aram::Point2D cp = extr.project(c);
    aram::Point2D dp = extr.project(d);
    aram::Point2D ep = extr.project(e);


    cv::line(mat,ap,bp,color,2);
    cv::line(mat,bp,cp,color,2);
    cv::line(mat,cp,dp,color,2);
    cv::line(mat,dp,ap,color,2);
    cv::line(mat,ap,ep,color,2);
    cv::line(mat,bp,ep,color,2);
    cv::line(mat,cp,ep,color,2);
    cv::line(mat,dp,ep,color,2);

    return;
}

int main(int argc, char** argv)
{
	try
	{
		// Detection parameters :
		// -> Region of interest detection
		// -> Tag validator
		typedef aram::TagDetector<aram::CannyFittingDetector,aram::LocalThreshTagMatcher> myDetector;
		
		// Tag detector instanciation
		myDetector *detector = new myDetector();
		
		// Intrinsics parameters
		aram::Intrinsic intr("C:\\camera_data.xml");
		
		aram::MultiTag mt;
		float m_size = 28.0;
		float m_delta = 14.0;

		aram::TagInfo t0(0,aram::Point2D(0.0,m_delta+m_size),m_size);
		aram::TagInfo t1(1,aram::Point2D(0.0,0.0),m_size);
		aram::TagInfo t2(2,aram::Point2D(m_delta+m_size,0.0),m_size);
		aram::TagInfo t3(3,aram::Point2D(m_delta+m_size,m_delta+m_size),m_size);

		mt.addTagInfo(t0);
		mt.addTagInfo(t1);
		mt.addTagInfo(t2);
		mt.addTagInfo(t3);

		
		// Video input (see openCV doc)
		cv::VideoCapture cap(0); // use default video (usually your webcam)
		if(!cap.isOpened()) throw std::exception();
		
		cv::Mat frame;

		// Main loop
		while(true)
       	{
			// next frame from video input 
			cap >> frame;
						
			// Tag detection
			detector->detect(frame);

			// Tag list iterator
			aram::iteratorTag it;
			
			for(it=detector->begin();it!=detector->end();++it)
			{
				aram::vecPoint2D corners = (*it)->corners();

				for(unsigned int i=0;i<corners.size();++i)
				{
					cv::line(frame,corners[i%4],corners[(i+1)%4],cv::Scalar(100,150,150),2);
				}
			}

			// If any tags was detected
			if(detector->begin()!=detector->end())
			{
				// Get extrinsics parameters
				aram::Extrinsic e = mt.compute(detector->begin(),detector->end(),intr);
				drawPyramide(frame,m_size*2+m_delta,aram::Point3D(m_size+m_delta/2,m_size+m_delta/2,0),cv::Scalar(0,255,0),e);
			}
	
			// render
			cv::imshow("render", frame);
			// GUI refresh (see openCV doc)
			if(cv::waitKey(10)>=0) break;
		}
	}
	catch(std::exception &)
	{
	}

	return 0;
}