#include <ARAM/FrameSet.hpp>

namespace aram
{
	void FrameSet::save(std::string name, const cv::Mat &mat)
	{	
		m_frameMap.insert(std::pair<std::string,cv::Mat>(name,mat));
		
		return;
	}
	
	cv::Mat & FrameSet::load(std::string name)
	{
		std::map<std::string,cv::Mat>::iterator it = m_frameMap.find(name);
		if(it==m_frameMap.end()) throw ARAMException(__LINE__, __FILE__, "FrameSet::load", "Wrong name");

		return (it->second);
	}
	
	bool FrameSet::exist(std::string name)
	{
		std::map<std::string,cv::Mat>::iterator it = m_frameMap.find(name);
		return (it!=m_frameMap.end());
	}

	void FrameSet::reset()
	{
		m_frameMap.clear();

		return;
	}
	
	iteratorFrameSet FrameSet::begin()
	{
		return m_frameMap.begin();
	}
		
	iteratorFrameSet FrameSet::end()
	{
		return m_frameMap.end();
	}
};