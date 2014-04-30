#include <ARAM/tools/Extrinsics.hpp>

namespace aram
{
	Extrinsics::Extrinsics(Intrinsics intr, vecPoint2D imgPoints, vecPoint3D objPoints):_intr(intr)
	{
		compute(imgPoints,objPoints);
	}
	
	Extrinsics::Extrinsics(Intrinsics intr, vecPoint2D imgPoints, vecPoint3D objPoints, float &error):_intr(intr)
	{
		compute(imgPoints,objPoints);

		for(unsigned int i=0;i<objPoints.size();++i)
		{
			Point2D proj = project(objPoints[i]);
			error += (float) cv::norm(proj-imgPoints[i]);
		}
		error/=objPoints.size();
	}

	Extrinsics::Extrinsics(Intrinsics intr, cv::Mat &rmat, cv::Mat &tvec):_intr(intr)
	{
		_rmat = rmat.clone();
		_tvec = tvec.clone();
	}
	
	void Extrinsics::compute(vecPoint2D imgPoints, vecPoint3D objPoints)
	{
		_objPts = objPoints;
		_imgPts = imgPoints;

		
		cv::solvePnP(objPoints, imgPoints, _intr.cameraMatrix(), _intr.distorsionCoefficient(), _rvec, _tvec);
		
		float mean = 0.0f;
		for(unsigned int i=0;i<objPoints.size();++i)
		{
			Point2D proj = project(objPoints[i]);
			mean += (float) cv::norm(proj-imgPoints[i]);
		}
		mean/=objPoints.size();

		cv::Rodrigues(_rvec,_rmat);
		
	}

	const cv::Mat & Extrinsics::rotationMatrix()
	{
		if(_rvec.empty() && _rmat.empty()) ARAMException(__LINE__, __FILE__, "Extrinsics::rotationVector", "Rotation matrix and vector are empty");

		if(!_rvec.empty() && _rmat.empty()) cv::Rodrigues(_rvec,_rmat);

		return _rmat;
	}

	const cv::Mat & Extrinsics::rotationVector()
	{
		if(_rvec.empty() && _rmat.empty()) ARAMException(__LINE__, __FILE__, "Extrinsics::rotationVector", "Rotation matrix and vector are empty");
		
		if(_rvec.empty() && !_rmat.empty()) cv::Rodrigues(_rmat,_rvec);

		return _rvec;
	}

	const cv::Mat & Extrinsics::translationVector()
	{
		if(_tvec.empty()) ARAMException(__LINE__, __FILE__, "Extrinsics::translationVector", "Translation vector is empty");
		return _tvec;
	}

	Point2D Extrinsics::project(Point3D pt)
	{
		if(!_intr.valid()) throw ARAMException(__LINE__, __FILE__, "Extrinsics::project", "Wrong intrinsics parameters");
	
		if(_tvec.empty()||_rvec.empty()) throw ARAMException(__LINE__, __FILE__, "Extrinsics::project", "Wrong extrinsics parameters");

		vecPoint3D objPoints;
		objPoints.push_back(pt);
		
		vecPoint2D imgPoints;
		cv::projectPoints(objPoints, _rvec, _tvec, _intr.cameraMatrix(), _intr.distorsionCoefficient(), imgPoints);

		return imgPoints[0];
	}

	vecPoint2D Extrinsics::project(vecPoint3D objPoints)
	{
		if(!_intr.valid()) throw ARAMException(__LINE__, __FILE__, "Extrinsics::project", "Wrong intrinsics parameters");
	
		if(_tvec.empty()||_rvec.empty()) throw ARAMException(__LINE__, __FILE__, "Extrinsics::project", "Wrong extrinsics parameters");

		vecPoint2D imgPoints;

		cv::projectPoints(objPoints, _rvec, _tvec, _intr.cameraMatrix(), _intr.distorsionCoefficient(), imgPoints);

		return imgPoints;
	}

	vecPoint3D Extrinsics::objPoints()
	{
		return _objPts;
	}

	vecPoint2D Extrinsics::imgPoints()
	{
		return _imgPts;
	}
};