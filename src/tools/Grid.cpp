#include <ARAM/tools/Grid.hpp>

namespace aram
{
	TagInfo::TagInfo(int i, Point2D o, float s)
	{
		id = i;
		origin = o;
		size = s;
	}

	void Grid::addTagInfo(TagInfo t)
	{
		_tags.push_back(t);
		return;
	}

	TagInfo Grid::getTagInfo(int sId)
	{
		TagInfo t(0,Point2D(0.0,0.0),0.0);

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
			throw ARAMException(__LINE__,__FILE__, "Grid::getTagInfo", ss.str());
		}
		return t;
	}
};