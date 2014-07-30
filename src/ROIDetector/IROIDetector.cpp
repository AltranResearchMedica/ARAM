#include <ARAM/ROIDetector/IROIDetector.hpp>

namespace aram
{
	IROIDetector::IROIDetector(FrameSet *fs):p_fs(fs)
	{
	}

	void IROIDetector::save(std::string name, cv::Mat &mat)
	{
		p_fs->save(name,mat);
		return;
	}

	cv::Mat & IROIDetector::load(std::string name)
	{
		return p_fs->load(name);
	}

	bool IROIDetector::exist(std::string name)
	{
		return p_fs->exist(name);
	}

};
