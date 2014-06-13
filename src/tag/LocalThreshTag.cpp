#include <ARAM/tag/LocalThreshTag.hpp>

namespace aram
{
	LocalThreshTag::LocalThreshTag(ROI &r):ITag(r),_tagSize(9),_scale(8),_id(-1)
	{
	}

	bool LocalThreshTag::checkTag(vecROI *, vecTag *, FrameSet *fs)
	{
		vecPoint2D src = corners();
		vecPoint2D dst;
		dst.resize(4);

		float s = (float) _tagSize*_scale;
		dst[0] = Point2D(0.0, 0.0);
		dst[1] = Point2D(0.0, s);
		dst[2] = Point2D(s, s);
		dst[3] = Point2D(s, 0.0);
		
		cv::Mat pers = cv::getPerspectiveTransform(src, dst);
		
		cv::Mat currentFrame, out, otsu, gray;

		if(!fs->exist("tresh"))
		{

			currentFrame = fs->load("currentFrame");
			cv::cvtColor(currentFrame,gray,CV_BGR2GRAY);
			cv::threshold(gray, otsu, 0, 255, cv::THRESH_BINARY_INV | CV_THRESH_OTSU);

			fs->save("thresh",otsu);
		}
		else
		{
			otsu = fs->load("thresh");
		}
		
		cv::Size size(_tagSize*_scale,_tagSize*_scale);
		cv::warpPerspective(otsu, out, pers, size, cv::INTER_NEAREST);


		cv::Mat bits = cv::Mat::ones(_tagSize,_tagSize,CV_8UC1);

		//compute matrix of 0/1
		int swidth = out.rows/_tagSize;
		for(int y=0;y<_tagSize;y++)
		{
			for(int x=0;x<_tagSize;x++)
			{
				int Xstart=(x)*(swidth);
				int Ystart=(y)*(swidth);
				cv::Mat square = out(cv::Rect(Xstart,Ystart,swidth,swidth));
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

		if(res==-1) return false;
		else return true;
	}

	Extrinsics LocalThreshTag::extrinsics(Intrinsics &intr, float size)
	{
		if(!intr.valid()) throw ARAMException(__LINE__, __FILE__, "LocalThreshTag::extrinsics", "Wrong intrinsics parameters");

		vecPoint3D objPoints;
		

		Point3D p1(0.0, size, 0.0);
		Point3D p2(size, size, 0.0);
		Point3D p3(size, 0.0, 0.0);
		Point3D p4(0.0, 0.0, 0.0);

		objPoints.push_back(p1);
		objPoints.push_back(p2);
		objPoints.push_back(p3);
		objPoints.push_back(p4);

		return Extrinsics(intr,corners(),objPoints);
	}

	int LocalThreshTag::id()
	{
		return _id;
	}

	void LocalThreshTag::rotate(cv::Mat &in, cv::Mat &out)
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