#include <ARAM/ROIDetector/ROI.hpp>

namespace aram
{
	ROI::ROI():m_id(-1)
	{
	}

	void ROI::corners(const vecPoint2D &pts)
	{
		m_corners.clear();

		constIteratorPoint2D it;

		for(it=pts.begin();it!=pts.end();++it) corners(*it);
		
		return;
	}

	void ROI::corners(const Point2D &pt)
	{
		CV_Assert(m_corners.size()<4);

		m_corners.push_back(pt);
		
		return;
	}
	
	vecPoint2D & ROI::corners()
	{
		CV_Assert(m_corners.size()==4);

		return m_corners;
	}
		
	void ROI::rotate(int n)
	{
		std::rotate(m_corners.begin(),m_corners.begin()+n%m_corners.size(),m_corners.end());
		return;
	}

	Extrinsic ROI::extrinsic(const Intrinsic &intr, float size)
	{
		CV_Assert(m_id!=-1);
		CV_Assert(intr.valid());
		
		vecPoint3D objPoints;

		float s = size/2.0f;
		objPoints.push_back(Point3D(-s, -s, 0.0));
		objPoints.push_back(Point3D(-s,s,0.0));
		objPoints.push_back(Point3D(s,s,0.0));
		objPoints.push_back(Point3D(s,-s,0.0));
		

		return Extrinsic(intr,corners(),objPoints);
	}


	int ROI::id() const
	{
		return m_id;
	}

	void ROI::id(int i)
	{
		m_id = i;
		return;
	}

};