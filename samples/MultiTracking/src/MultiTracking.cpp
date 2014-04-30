#include <ARAM/TagDetector.hpp> // Main ARAM class
#include <ARAM/tag/HammingTag.hpp> // Tag validator
#include <ARAM/ROIDetector/EdgeDetector.hpp> // Region of interest detection

#include <ARAM/tools/Grid.hpp> // Grid an TagInfo
#include <ARAM/coordinate/Chessboard.hpp> // Multi tracking Extrinsics compute

#include <opencv2/opencv.hpp> // OpenCV data structure

#include <exception> //std::exception

int main(int argc, char** argv)
{
	try
	{
		float size = 142.0;
		float delta = 654.0;

		aram::Grid g;
		aram::TagInfo t1(21,aram::Point2D(0.0,0.0),size);
		aram::TagInfo t2(22,aram::Point2D(0.0,delta),size);
		aram::TagInfo t3(19,aram::Point2D(delta,delta),size);
		aram::TagInfo t4(20,aram::Point2D(delta,0.0),size);
		
		g.addTagInfo(t1);
		g.addTagInfo(t2);
		g.addTagInfo(t3);
		g.addTagInfo(t4);
		aram::Chessboard *coord = new aram::Chessboard(g);


		// Detection parameters :
		// -> Region of interest detection
		// -> Tag validator USE HAMMINGTAG FOR MULTI TRACKING !
		typedef aram::TagDetector<aram::EdgeDetector,aram::HammingTag> myDetector;
		
		// Tag detector instanciation
		myDetector *detector = new myDetector();
		
		// Intrinsics parameters
		aram::Intrinsics intr("C:\\camera_data.xml");
		
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
						
			// Intrinsics parameters
			aram::Intrinsics intr("C:\\camera_data.xml");

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
				aram::Extrinsics e = coord->compute(detector->begin(),detector->end(),intr);

				// Project 3D world coordinate -> 2D image coordinate
				aram::Point2D o = e.project(aram::Point3D(0.0,0.0,0.0));
				aram::Point2D x = e.project(aram::Point3D(delta,0.0,0.0));
				aram::Point2D y = e.project(aram::Point3D(0.0,delta,0.0));
				aram::Point2D z = e.project(aram::Point3D(0.0,0.0,delta/2.0));

				// draw axis
				cv::line(frame,o,x,cv::Scalar(200,0,0),2);
				cv::line(frame,o,y,cv::Scalar(0,200,0),2);
				cv::line(frame,o,z,cv::Scalar(0,0,200),2);
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