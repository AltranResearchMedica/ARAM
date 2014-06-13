#include <ARAM/coordinate/Chessboard.hpp>

namespace aram
{
	Chessboard::Chessboard(Grid g):_grid(g)
	{
	}

	Extrinsics Chessboard::compute(iteratorTag begin, iteratorTag end, Intrinsics intr)
	{		
		iteratorTag it;
		
		vecPoint3D objPoints;
		vecPoint2D imgPoints;
		
		
		for(it=begin;it!=end;++it)
		{
		//if((*it)->id()==0)
		{
			try
			{
				TagInfo t = _grid.getTagInfo((*it)->id());
								
				float x0 = t.origin.x;
				float y0 = t.origin.y;
				float s = t.size;
				
				objPoints.push_back(Point3D(x0,y0,0.0f));
				objPoints.push_back(Point3D(x0,y0+s,0.0f));
				objPoints.push_back(Point3D(x0+s,y0+s,0.0f));
				objPoints.push_back(Point3D(x0+s,y0,0.0f));


				vecPoint2D corners = (*it)->corners();
				
				imgPoints.push_back(corners[0]);
				imgPoints.push_back(corners[1]);
				imgPoints.push_back(corners[2]);
				imgPoints.push_back(corners[3]);
			}
			catch(std::exception &)
			{
			}
		}
		}
		
		return Extrinsics(intr,imgPoints,objPoints);
	}
};