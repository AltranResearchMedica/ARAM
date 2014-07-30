#include <ARAM/tag/ITagMatcher.hpp>

namespace aram
{	
	ITagMatcher::ITagMatcher(FrameSet *fs):p_fs(fs)
	{
	}

	void ITagMatcher::save(std::string name, cv::Mat &mat)
	{
		p_fs->save(name,mat);
		return;
	}

	bool ITagMatcher::exist(std::string name)
	{
		return p_fs->exist(name);
	}

	cv::Mat & ITagMatcher::load(std::string name)
	{
		return p_fs->load(name);
	}

	void ITagMatcher::rotate(cv::Mat &in, cv::Mat &out)
	{
		CV_Assert(in.cols==in.rows);

		cv::Mat res = in.clone();
		for(int i=0;i<in.cols;++i)
		{
			for (int j=0;j<in.rows;++j)
			{
				res.at<uchar>(i,j)=in.at<uchar>(in.cols-j-1,i);
			}
		}

		out = res.clone();
		return;
	}
};