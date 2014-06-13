#include <ARAM/tag/HarrisCornerTag.hpp>

namespace aram
{
	HarrisCornerTag::HarrisCornerTag(ROI &r):ITag(r),_tagSize(9),_scale(8),_id(-1)
	{
	}

	bool HarrisCornerTag::checkTag(vecROI *rois, vecTag *tags, FrameSet *fs)
	{
		cv::Mat out;
		vecPoint2D src = corners();
		vecPoint2D dst;
		dst.resize(4);

		float s = (float) _tagSize*_scale;
		dst[0] = Point2D(0.0, 0.0);
		dst[1] = Point2D(0.0, s);
		dst[2] = Point2D(s, s);
		dst[3] = Point2D(s, 0.0);

		cv::Mat pers = cv::getPerspectiveTransform(src, dst);
		
		cv::Mat otsu;
		
		cv::Mat currentFrame = fs->load("currentFrame");
		cv::Size size(_tagSize*_scale,_tagSize*_scale);
		cv::warpPerspective(currentFrame, out, pers, size, cv::INTER_NEAREST);
		cv::cvtColor(out,out,CV_BGR2GRAY);
		cv::threshold(out, otsu, 0, 255, cv::THRESH_BINARY_INV | CV_THRESH_OTSU);
	
		cv::Mat bits = cv::Mat::ones(_tagSize,_tagSize,CV_8UC1);

		//compute matrix of 0/1
		int swidth = out.rows/_tagSize;
		for(int y=0;y<_tagSize;y++)
		{
			for(int x=0;x<_tagSize;x++)
			{
				int Xstart=(x)*(swidth);
				int Ystart=(y)*(swidth);
				cv::Mat square = otsu(cv::Rect(Xstart,Ystart,swidth,swidth));
				int nZ = cv::countNonZero(square);
				
				if(nZ>(swidth*swidth)/2) bits.at<uchar>(y,x) = 0;
			}
		}
		

		//try to find if this tag is in our dictonnary
		BinaryTree &bt = BinaryTree::getInstance();

		int res = -1;
		int nrot = 0;

		nrot = 0;
		while(res==-1&&nrot<4)
		{
			rotate(bits, bits);
			res = bt.hammingSearch(bits,10);
			nrot++;
		}

		ITag::rotate(nrot-1);
	
		_id = res;

		if(res==-1)
		{
			return false;
		}
		else
		{
			harrisCorner(fs);
			return true;
		}
	}

	void HarrisCornerTag::harrisCorner(FrameSet *fs)
	{
		vecPoint2D cor = corners();
		vecPoint2D newCor;
		newCor.resize(4);
		
		for(int c=0;c<cor.size();++c)
		{
			int window = 10;
			aram::Point2D currCorner = cor[c];
			
			cv::Mat img = fs->load("currentFrame");
			
			int a = currCorner.x-window;
			int b = currCorner.y-window;

			cv::Mat roi = img(cv::Rect(a,b,2*window,2*window));
			cv::Mat dst = cv::Mat::zeros(roi.size(), CV_32FC1);
			cv::cvtColor(roi,roi,CV_BGR2GRAY);
			int blockSize = 2;
			int apertureSize = 3;
			double k = 0.04;
			cv::cornerHarris(roi, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
			
			cv::Mat dst_norm,dst_norm_scaled;
			cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
			cv::convertScaleAbs(dst_norm, dst_norm_scaled);
			

			int maxX = 0;
			int maxY = 0;
			/// Drawing a circle around corners
			for(int j=0;j<dst_norm.rows;j++)
			{
				for(int i=0;i<dst_norm.cols;i++)
				{
					if(dst_norm.at<float>(j,i)>230)
					{
						maxX = j;
						maxY = j;
					}
				}
			}

			newCor[c] = Point2D(currCorner.x-window+maxX,currCorner.y-window+maxY);
			std::cout << currCorner << "->" << newCor[c] << std::endl;
		}

		corners(newCor);

		return;
	}

	Extrinsics HarrisCornerTag::extrinsics(Intrinsics &intr, float size)
	{
		if(!intr.valid()) throw ARAMException(__LINE__, __FILE__, "LocalThreshTag::extrinsics", "Wrong intrinsics parameters");

		vecPoint3D objPoints;
			
		Point3D p1(0.0, 0.0, 0.0);
		Point3D p2(0.0, size, 0.0);
		Point3D p3(size, size, 0.0);
		Point3D p4(size, 0.0, 0.0);
			
		objPoints.push_back(p1);
		objPoints.push_back(p2);
		objPoints.push_back(p3);
		objPoints.push_back(p4);

		return Extrinsics(intr,corners(),objPoints);
	}

	int HarrisCornerTag::id()
	{
		return _id;
	}

	void HarrisCornerTag::rotate(cv::Mat &in, cv::Mat &out)
	{
		cv::Mat res = in.clone();
		for(int i=0;i<_tagSize;++i)
		{
			for (int j=0;j<_tagSize;++j)
			{
				res.at<uchar>(i,j)=in.at<uchar>(_tagSize-j-1,i);
			}
		}

		out = res.clone();
		return;
	}
};