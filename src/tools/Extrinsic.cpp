#include <ARAM/tools/Extrinsic.hpp>

namespace aram
{
	Extrinsic::Extrinsic(Intrinsic intr, vecPoint2D imgPoints, vecPoint3D objPoints):m_intr(intr)
	{
		m_rvec = cv::Mat::zeros(3,1,CV_64FC1);
		m_tvec = cv::Mat::zeros(3,1,CV_64FC1);
		compute(imgPoints,objPoints);
	}
	
	Extrinsic::Extrinsic(Intrinsic intr, vecPoint2D imgPoints, vecPoint3D objPoints, float &error):m_intr(intr)
	{
		compute(imgPoints,objPoints);

		for(unsigned int i=0;i<objPoints.size();++i)
		{
			Point2D proj = project(objPoints[i]);
			error += (float) cv::norm(proj-imgPoints[i]);
		}
		error/=objPoints.size();
	}

	Extrinsic::Extrinsic(Intrinsic intr, cv::Mat &rmat, cv::Mat &tvec):m_intr(intr)
	{
		m_rmat = rmat.clone();
		m_tvec = tvec.clone();
	}
	
	void Extrinsic::compute(vecPoint2D iP, vecPoint3D oP)
	{
		m_objPts = oP;
		m_imgPts = iP;

		// WARNING : Flag CV_P3P and CV_EPNP return unstable results
		cv::solvePnP(m_objPts, m_imgPts, m_intr.cameraMatrix(), m_intr.distorsionCoefficient(), m_rvec, m_tvec, false);

		cv::Rodrigues(m_rvec,m_rmat);
	}

	float Extrinsic::error()
	{
		CV_Assert(m_imgPts.size()==m_objPts.size());
		
		vecPoint2D projPoints = project(m_objPts);

		float err = 0.0;
		for(unsigned int i=0;i<m_imgPts.size();++i)
		{
			err+=std::sqrt((m_imgPts[i].x-projPoints[i].x)*(m_imgPts[i].x-projPoints[i].x)+(m_imgPts[i].y-projPoints[i].y)*(m_imgPts[i].y-projPoints[i].y));
		}

		err/= (float) m_imgPts.size();

		return err;
	}

	const cv::Mat & Extrinsic::rotationMatrix()
	{		
		CV_Assert(!m_rvec.empty() || !m_rmat.empty());

		if(!m_rvec.empty() && m_rmat.empty()) cv::Rodrigues(m_rvec,m_rmat);

		return m_rmat;
	}

	const cv::Mat & Extrinsic::rotationVector()
	{
		CV_Assert(!m_rvec.empty() || !m_rmat.empty());
		
		if(m_rvec.empty() && !m_rmat.empty()) cv::Rodrigues(m_rmat,m_rvec);

		return m_rvec;
	}

	const cv::Mat & Extrinsic::translationVector()
	{
		CV_Assert(!m_rmat.empty());
		
		return m_tvec;
	}

	Point2D Extrinsic::project(Point3D pt)
	{
		CV_Assert(m_intr.valid());
		CV_Assert(!m_tvec.empty()&&!m_rvec.empty());
		
		vecPoint3D objPoints;
		objPoints.push_back(pt);
		
		vecPoint2D imgPoints;
		cv::projectPoints(objPoints, m_rvec, m_tvec, m_intr.cameraMatrix(), m_intr.distorsionCoefficient(), imgPoints);

		return imgPoints[0];
	}

	vecPoint2D Extrinsic::project(vecPoint3D objPoints)
	{
		CV_Assert(m_intr.valid());
		CV_Assert(!m_tvec.empty()&&!m_rvec.empty());
		
		vecPoint2D imgPoints;

		cv::projectPoints(objPoints, m_rvec, m_tvec, m_intr.cameraMatrix(), m_intr.distorsionCoefficient(), imgPoints);

		return imgPoints;
	}

	vecPoint3D Extrinsic::objPoints()
	{
		return m_objPts;
	}

	vecPoint2D Extrinsic::imgPoints()
	{
		return m_imgPts;
	}
};