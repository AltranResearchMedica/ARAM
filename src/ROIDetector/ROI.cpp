#include <ARAM/ROIDetector/ROI.hpp>

namespace aram
{
	void ROI::corners(const vecPoint2D &pts)
	{
		_corners.clear();

		constIteratorPoint2D it;

		for(it=pts.begin();it!=pts.end();++it) corners(*it);
		
		return;
	}

	void ROI::corners(const Point2D &pt)
	{
		if(_corners.size()>=4) throw ARAMException(__LINE__, __FILE__, "ROI::corners", "Too many corners");

		_corners.push_back(pt);
		
		return;
	}
	
	vecPoint2D ROI::corners() const
	{
		if(_corners.size()!=4) throw ARAMException(__LINE__, __FILE__, "ROI::corners", "Bad ROI content");
		
		return _corners;
	}

};