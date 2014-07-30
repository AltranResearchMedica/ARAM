#include <ARAM/tools/MilgramContourClosing.hpp>

namespace aram
{
	MilgramContourClosing MilgramContourClosing::s_instance=MilgramContourClosing();

	MilgramContourClosing& MilgramContourClosing::operator=(const MilgramContourClosing&)
	{
		return s_instance;
	}

	MilgramContourClosing::MilgramContourClosing(const MilgramContourClosing&)
	{
	}

	MilgramContourClosing & MilgramContourClosing::getInstance()
	{
		return s_instance;
	}

	int min(int a, int b, int c, int d, int e)
	{
		int min = a;
		if(b<min) min = b;
		if(c<min) min = c;
		if(d<min) min = d;
		if(e<min) min = e;

		return min;
	}

	MilgramContourClosing::MilgramContourClosing(int limit, int iter):m_limit(limit),m_iter(iter)
	{
		initLUT();
	}


	void MilgramContourClosing::initLUT()
	{
		p_LUT = new std::vector<bool>;

		p_LUT->resize(65536); // = 2^16
	
		std::vector<int> data, datab;
		data.resize(9);
		datab.resize(9);
	
		data[8] = 0;

		int count = 0, selle = 0;
		for(int a=-1;a<2;a++)
		{
			for(int b=-1;b<2;b++)
			{
				for(int c=-1;c<2;c++)
				{
					for(int d=-1;d<2;d++)
					{
						for(int e=-1;e<2;e++)
						{
							for(int f=-1;f<2;f++)
							{
								for(int g=-1;g<2;g++)
								{
									for(int h=-1;h<2;h++)
									{
										data[0] = a;
										data[1] = b;
										data[2] = c;
										data[3] = d;
										data[4] = e;
										data[5] = f;
										data[6] = g;
										data[7] = h;

										int res = computeA(data);

										datab[0] = data[0]+3;
										datab[1] = data[1]+3;
										datab[2] = data[2]+3;
										datab[3] = data[3]+3;
										datab[4] = data[4]+3;
										datab[5] = data[5]+3;
										datab[6] = data[6]+3;									
										datab[7] = data[7]+3;							
										datab[8] = data[8]+3;

										p_LUT->at(res) = isSelle(datab);
									}
								}
							}
						}
					}
				}
			}
		}

		return;
	}


	void MilgramContourClosing::close(cv::Mat &src)
	{
		CV_Assert(src.channels() == 1);
		
		cv::Mat dist;
		initDistanceMap(src,dist);


		bool saddlePointFound = true;
		int i=0;
		while(i<m_iter&&saddlePointFound)
		{
			updateDistanceMap(dist);
			saddlePointFound = updateSelle(dist);
			i++;
		}

		updateCanny(src,dist);


		return;
	}

	void MilgramContourClosing::updateDistanceMap(cv::Mat &dist)
	{	
		// CV_Assert(dist.channels() == 1);

		// i ->line
		// j -> columns
		// update map of chamfer distance
	
		int nRows = dist.rows;
		int nCols = dist.cols;

		int i=0,j=0;

		// first scanning
		//
		// mask :
		// Position | j-1 |  j  | j+1
		//    i-1   | +3  | +2  | +3
		//    i     | +2  | +0  |  X	
	
		uchar *pm1, *p;
	
		p = dist.ptr<uchar>(0);
	
		// first row compute

		// first col 
		// p[0] = p[0];
		for(j=1; j<nCols; ++j)
		{
			p[j] = (p[j]<p[j-1]+2) ? p[j]:p[j-1]+2;
		}

		// row 1 to n-1
		for(i=1; i<nRows; i++)
		{
			// row shiftting
			pm1 = p;
			p = dist.ptr<uchar>(i);
        
			// first col
			p[0] = min(p[0],pm1[0]+2,pm1[1]+3,255,255);
			for(j=1; j<nCols-1; j++)
			{
				//d(i,j)=MIN{d(i,j), d(i,j-1)+2, d(i-1,j-l)+3, d(i-1,j)+2, d(i-l,j+l)+3}
				p[j] = min(p[j],p[j-1]+2,pm1[j-1]+3,pm1[j]+2,pm1[j+1]+3);
			}

			// last col
			p[j] = min(p[j],p[j-1]+2,pm1[j-1]+3,pm1[j]+2,255);
		}


		// second scanning
		//
		// mask :
		//	   X   |  0  | +2  |   i
		//    +3   | +2  | +3  |  i+1
		//    j-1  |  j  | j+1 |  Position

		i = nRows-1;
		j = nCols-1;
	
		// last row 
		p = dist.ptr<uchar>(nRows-1);
	
		// last col
		// p[nCols-1] = p[nCols-1];

		for(j=nCols-2; j>=0; j--)
		{
			p[j] = (p[j]<p[j+1]+2) ? p[j]:p[j+1]+2;
		}

		for(i=nRows-2; i>=0; i--)
		{
			// row shiftting
			pm1 = p;
			p = dist.ptr<uchar>(i);

			// last col
			p[nCols-1] = min(p[nCols-1],pm1[nCols-1]+2,pm1[nCols-2]+3,255,255);

			for(j=nCols-2; j>=1; j--)
			{
				p[j] = min(p[j], p[j+1]+2, pm1[j-1]+3, pm1[j]+2, pm1[j+1]+3);
			}

			// first col
			p[j] = min(p[j],p[j+1]+2,pm1[j]+2,pm1[j+1]+3,255);
		}

		return;
	}

	void MilgramContourClosing::initDistanceMap(cv::Mat& in, cv::Mat& dist)
	{
		// accept only char type matrices
		// CV_Assert(in.depth() != sizeof(uchar));
		// CV_Assert(in.channels() == 1);
	
		int nRows = in.rows;
		int nCols = in.cols;

		dist = cv::Mat::ones(in.size(),CV_8UC1);

		uchar *pin, *pout;
		for(int i=0; i<nRows; ++i)
		{
			pout = dist.ptr<uchar>(i);
			pin = in.ptr<uchar>(i);
			for(int j=0; j<nCols; ++j)
			{
				pout[j] = (pin[j]!=0) ? 0:255;
			}
		}

		return;
	}


	void MilgramContourClosing::updateCanny(cv::Mat &canny, cv::Mat &dist)
	{
		// accept only char type matrices
		// CV_Assert(in.depth() != sizeof(uchar));
		// CV_Assert(in.channels() == 1);
	
		int nRows = canny.rows;
		int nCols = canny.cols;

		uchar *pin, *pout;
		for(int i=0; i<nRows; ++i)
		{
			pout = canny.ptr<uchar>(i);
			pin = dist.ptr<uchar>(i);
			for(int j=0; j<nCols; ++j)
			{
				pout[j] = (pin[j]==0) ? 255:0;
			}
		}

		return;
	}

	int MilgramContourClosing::computeA(std::vector<int> &data)
	{
		// CV_Assert(data.size() == 9);

		int d8 = data[8];
	
		int a = 0;
		for(int i=7;i>=0;--i)
		{
			bool Z = ((data[i]-d8) == 0);
			bool N = ((data[i]-d8) < 0);

			bool y = N;
			bool x = !(N|Z);
		
			a = a << 2;
			a += x*2 +y;
		}

		return a;
	}

	bool MilgramContourClosing::updateSelle(cv::Mat &dist)
	{
		std::vector<std::pair<uchar*,int> > saddlePoint;

		bool selleFound = false;
		std::vector<int> data;
		data.resize(9);

		int nCols = dist.cols;
		int nRows = dist.rows;
		int i=0, j=0;

		uchar *pm1, *p, *pp1;
		p = dist.ptr<uchar>(0);
		pp1 = dist.ptr<uchar>(1);

		for(i=1; i<nRows-1; i++)
		{
			pm1 = p;
			p = pp1;
			pp1 = dist.ptr<uchar>(i+1);

			for(j=1; j<nCols-1; j++)
			{
				//
				// 3 | 2 | 1
				// ---------
				// 4 | 8 | 0
				// ---------
				// 5 | 6 | 7
				//
			
				data[8] = p[j];

				if(data[8]>1&&data[8]<m_limit)
				{
					data[0] = p[j+1];
					data[1] = pm1[j+1];
					data[2] = pm1[j];
					data[3] = pm1[j-1];
					data[4] = p[j-1];
					data[5] = pp1[j-1];
					data[6] = pp1[j];
					data[7] = pp1[j+1];

					int a = computeA(data);

					if(p_LUT->at(a))
					{
						saddlePoint.push_back(std::make_pair(p,j));
						selleFound = true;
					}
				}
			}
		}

		for(unsigned int k=0;k<saddlePoint.size();k++)
		{
			saddlePoint[k].first[saddlePoint[k].second] = 0;
		}

		return selleFound;
	}



	bool MilgramContourClosing::checkSelle(int a, int b, int c, int d, std::vector<int> &data)
	{		
		int strict = 0;
		int large = 0;
		if(data[a]<data[8]) strict++;
		if(data[a]<=data[8]) large++;
									
		if(data[b]<data[8]) strict++;
		if(data[b]<=data[8]) large++;

		if(data[c]>data[8]) strict++;
		if(data[c]>=data[8]) large++;
									
		if(data[d]>data[8]) strict++;
		if(data[d]>=data[8]) large++;
							
		if(strict>=3&&large==4)
		{
			return true;
		}
	
		return false;
	}

	bool MilgramContourClosing::isSelle(std::vector<int> &data)
	{
		//CV_Assert(data.size()==9);
	
		bool res = false;


		int a=1, b=5, c=3, d=7;
		for(int rot = 0; rot<8 && !res; rot++)
		{
			int ap = (a+rot)%8;
			int bp = (b+rot)%8;
			int cp = (c+rot)%8;
			int dp = (d+rot)%8;
		
			res = res||checkSelle(ap,bp,cp,dp,data);
			res = res||checkSelle(bp,ap,cp,dp,data);
			res = res||checkSelle(ap,bp,dp,cp,data);
			res = res||checkSelle(bp,ap,dp,cp,data);
		}
	
		a=3, b=6, c=5, d=0;
		for(int rot = 0; rot<8 && !res; rot++)
		{
			int ap = (a+rot)%8;
			int bp = (b+rot)%8;
			int cp = (c+rot)%8;
			int dp = (d+rot)%8;
		
			res = res||checkSelle(ap,bp,cp,dp,data);
			res = res||checkSelle(bp,ap,cp,dp,data);
			res = res||checkSelle(ap,bp,dp,cp,data);
			res = res||checkSelle(bp,ap,dp,cp,data);
		

			res = res||checkSelle(cp,dp,ap,bp,data);
			res = res||checkSelle(cp,dp,bp,ap,data);
			res = res||checkSelle(dp,cp,ap,bp,data);
			res = res||checkSelle(dp,cp,bp,ap,data);
		}
	
	
		return res;
	}
};