#include <ARAM/tools/Intrinsics.hpp>

namespace aram
{

	Intrinsics::Intrinsics(const std::string file)
	{
		load(file);
	}

	void Intrinsics::load(const std::string file)
	{
		cv::FileStorage fs(file,cv::FileStorage::READ);
		
		if(!fs.isOpened()) throw ARAMException(__LINE__, __FILE__, "Intrinsics::Intrinsics", "Failed to open intrinsics parameters");

		fs["Camera_Matrix"] >> _cameraMatrix;
		fs["Distortion_Coefficients"] >> _distorsionCoefficient;
		fs.release();
		
		if(_cameraMatrix.empty()) throw ARAMException(__LINE__, __FILE__, "Intrinsics::Intrinsics", "Failed to read camera matrix");
		if(_distorsionCoefficient.empty()) throw ARAMException(__LINE__, __FILE__, "Intrinsics::Intrinsics", "Failed to read distortions coefficients");
	}

	const cv::Mat & Intrinsics::cameraMatrix()
	{
		return _cameraMatrix;
	}

	const cv::Mat & Intrinsics::distorsionCoefficient()
	{
		return _distorsionCoefficient;
	}

	bool Intrinsics::valid()
	{
		return !_distorsionCoefficient.empty()&&!_cameraMatrix.empty();
	}
};