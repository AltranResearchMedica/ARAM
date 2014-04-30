#include <ARAM/TagDetector.hpp> // Main ARAM class
#include <ARAM/tag/StandardTag.hpp> // Tag validator
#include <ARAM/ROIDetector/EdgeDetector.hpp> // Region of interest detection

#include <opencv2/opencv.hpp> // OpenCV data structure

#include <exception> //std::exception

int main(int argc, char** argv)
{
	try
	{
		// Detection parameters :
		// -> Region of interest detection
		// -> Tag validator
		typedef aram::TagDetector<aram::EdgeDetector,aram::StandardTag> myDetector;
		
		// Tag detector instanciation
		myDetector *detector = new myDetector();
		
		// Intrinsics parameters
		aram::Intrinsics intr("C:\\camera_data.xml");
		
		// Video input (see openCV doc)
		
		// use default video input (usually your webcam)
		cv::VideoCapture cap(0);
		
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
			// Loop over valid tag in current frame
			for(it=detector->begin();it!=detector->end();++it)
			{
				aram::vecPoint2D imgPoint = (*it)->corners();
				for(unsigned int i=0;i<imgPoint.size();++i)
				{
					cv::circle(frame,imgPoint[i],3,cv::Scalar(200,50,50),3);
				}
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
