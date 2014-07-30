#include <ARAM/coordinate/MultiTag.hpp>

namespace aram
{
	MultiTag::MultiTag()
	{
	}

	Extrinsic MultiTag::compute(iteratorTag begin, iteratorTag end, Intrinsic intr)
	{		
		iteratorTag it;
		
		vecPoint3D objPoints;
		vecPoint2D imgPoints;
		
		
		for(it=begin;it!=end;++it)
		{
			try
			{
				TagInfo t = getTagInfo((*it)->id());
								
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
		
		return Extrinsic(intr,imgPoints,objPoints);
	}

	TagInfo::TagInfo(int i, Point2D o, float s)
	{
		id = i;
		origin = o;
		size = s;
	}

	void MultiTag::addTagInfo(TagInfo t)
	{
		_tags.push_back(t);
		return;
	}

	TagInfo MultiTag::getTagInfo(int sId)
	{
		TagInfo t(-1,Point2D(0.0,0.0),0.0);

		bool exist = false;
		std::vector<TagInfo>::iterator it;
		for(it=_tags.begin();it!=_tags.end();++it)
		{
			if(it->id==sId)
			{
				t=*it;
				exist = true;
			}
		}

		if(!exist)
		{
			std::stringstream ss;
			ss << "Tag id not found (" << sId << ")";
			throw ARAMException(__LINE__,__FILE__, "MultiTag::getTagInfo", ss.str());
		}
		return t;
	}
};