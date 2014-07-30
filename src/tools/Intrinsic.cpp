#include <ARAM/tools/Intrinsic.hpp>

namespace aram
{
	Intrinsic::Intrinsic(const std::string file)
	{
		load(file);
	}

	void Intrinsic::load(const std::string file)
	{
		cv::FileStorage fs(file,cv::FileStorage::READ);
				
		CV_Assert(fs.isOpened());

		fs["Camera_Matrix"] >> m_cameraMatrix;
		fs["Distortion_Coefficients"] >> m_distorsionCoefficient;
		fs.release();
		
		CV_Assert(!m_cameraMatrix.empty());
		CV_Assert(!m_distorsionCoefficient.empty());
		
		return;
	}

	const cv::Mat & Intrinsic::cameraMatrix()
	{
		return m_cameraMatrix;
	}

	const cv::Mat & Intrinsic::distorsionCoefficient()
	{
		return m_distorsionCoefficient;
	}

	bool Intrinsic::valid() const
	{
		return !m_distorsionCoefficient.empty()&&!m_cameraMatrix.empty();
	}	
};