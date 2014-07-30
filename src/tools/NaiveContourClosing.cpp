#include <ARAM/tools/NaiveContourClosing.hpp>

namespace aram
{
	bool NaiveContourClosing::checkEdge(int data[8])
	{
		//
		// 3 | 2 | 1
		// ---------
		// 4 | 8 | 0
		// ---------
		// 5 | 6 | 7
		//
		if(data[0]!=0 && (data[3]!=0||data[4]!=0||data[5]!=0)) return true;
		if(data[1]!=0 && (data[4]!=0||data[5]!=0||data[6]!=0)) return true;
		if(data[2]!=0 && (data[5]!=0||data[6]!=0||data[7]!=0)) return true;
		if(data[3]!=0 && (data[6]!=0||data[7]!=0||data[0]!=0)) return true;

		return false;
	}

	void NaiveContourClosing::close(cv::Mat &src)
	{	
		std::vector<std::pair<uchar*,int> > closingPoint;

		int nCols = src.cols;
		int nRows = src.rows;
		int i=0, j=0;

		uchar *pm1, *p, *pp1;
		p = src.ptr<uchar>(0);
		pp1 = src.ptr<uchar>(1);
	
		int data[8];

		for(i=1; i<nRows-1; i++)
		{
			pm1 = p;
			p = pp1;
			pp1 = src.ptr<uchar>(i+1);

			for(j=1; j<nCols-1; j++)
			{
				//
				// 3 | 2 | 1
				// ---------
				// 4 | 8 | 0
				// ---------
				// 5 | 6 | 7
				//
			
				data[0] = p[j+1];
				data[1] = pm1[j+1];
				data[2] = pm1[j];
				data[3] = pm1[j-1];
				data[4] = p[j-1];
				data[5] = pp1[j-1];
				data[6] = pp1[j];
				data[7] = pp1[j+1];
			
				if(checkEdge(data)) closingPoint.push_back(std::make_pair(p,j));
			}
		}

		// add contour to canny
		for(unsigned int k=0;k<closingPoint.size();k++)
		{
			closingPoint[k].first[closingPoint[k].second] = 255;
		}

		return;
	}
};