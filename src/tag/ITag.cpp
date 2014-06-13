#include <ARAM/tag/ITag.hpp>

namespace aram
{
	ITag::ITag(ROI &r)
	{
		_corners = r.corners();
	}

	const std::vector<Point2D> ITag::corners() const
	{
		return _corners;
	}
		
	void ITag::rotate(int n)
	{
		std::rotate(_corners.begin(),_corners.begin()+n,_corners.end());
		return;
	}
	
	void ITag::corners(vecPoint2D c)
	{
		_corners = c;
		return;
	}
};