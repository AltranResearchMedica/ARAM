#include <ARAM/tag/HammingTagMatcher.hpp>

namespace aram
{
	HammingTagMatcher::HammingTagMatcher(FrameSet *fs):ITagMatcher(fs),m_tagSize(7),m_scale(8)
	{
	}

	bool HammingTagMatcher::checkTag(ROI* roi)
	{
		cv::Mat out;
		vecPoint2D src = roi->corners();
		vecPoint2D dst;
		dst.resize(4);

		float s = (float) m_tagSize*m_scale;
		dst[0] = Point2D(0.0, 0.0);
		dst[1] = Point2D(0.0, s);
		dst[2] = Point2D(s, s);
		dst[3] = Point2D(s, 0.0);

		cv::Mat pers = cv::getPerspectiveTransform(src, dst);

		cv::Mat currentFrame, grayscale, binary;

		if(!exist("tresh"))
		{
			int blockSize = (int) (cv::norm(src[0]-src[1])/(double)m_tagSize);
			if(blockSize<3) blockSize = 3;
			if(blockSize%2==0) blockSize++;

			currentFrame = load("currentFrame");
			cv::cvtColor(currentFrame, grayscale,CV_BGR2GRAY);

			//cv::adaptiveThreshold(grayscale,binary,255.0,cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, blockSize+2, 0.5);
			cv::threshold(grayscale, binary, 128, 255, 1);

			save("thresh",binary);
		}
		else
		{
			binary = load("thresh");
		}
		
		cv::Size size(m_tagSize*m_scale,m_tagSize*m_scale);
		cv::warpPerspective(binary, out, pers, size, cv::INTER_NEAREST);
		

		cv::Mat bits = cv::Mat::ones(m_tagSize,m_tagSize,CV_8UC1);

		//compute matrix of 0/1
		int swidth = out.rows/m_tagSize;
		for(int y=0;y<m_tagSize;y++)
		{
			for(int x=0;x<m_tagSize;x++)
			{
				int Xstart=(x)*(swidth);
				int Ystart=(y)*(swidth);
				cv::Mat square = out(cv::Rect(Xstart,Ystart,swidth,swidth));
				int nZ = cv::countNonZero(square);
				
				if(nZ>(swidth*swidth)/2) bits.at<uchar>(y,x) = 0;
			}
		}
		
		//try to find if this tag is in our dictonnary
		TagDictionnary *bt = TagDictionnary::getInstance();

		int res = -1;
		int nrot = 0;

		nrot = 0;
		while(res==-1&&nrot<4)
		{
			rotate(bits, bits);
			res = bt->hammingSearch(bits);
			nrot++;
		}

		roi->rotate(nrot+2);
		roi->id(res);

		return (res!=-1);
	}
};
