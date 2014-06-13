#include <ARAM/FrameSet.hpp>

namespace aram
{
	void FrameSet::save(std::string name, const cv::Mat &mat)
	{	
		_frameMap.insert(std::pair<std::string,cv::Mat>(name,mat));
		
		return;
	}
	
	cv::Mat FrameSet::load(std::string name)
	{
		std::map<std::string,cv::Mat>::iterator it = _frameMap.find(name);
		if(it==_frameMap.end()) throw ARAMException(__LINE__, __FILE__, "FrameSet::load", "Wrong name");

		return (it->second);
	}
	
	bool FrameSet::exist(std::string name)
	{
		std::map<std::string,cv::Mat>::iterator it = _frameMap.find(name);
		return (it!=_frameMap.end());
	}

	void FrameSet::reset()
	{
		_frameMap.clear();

		return;
	}
	
	std::map<std::string,cv::Mat>::iterator FrameSet::begin()
	{
		return _frameMap.begin();
	}
		
	std::map<std::string,cv::Mat>::iterator FrameSet::end()
	{
		return _frameMap.end();
	}
};