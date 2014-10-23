#include "../include/TagGenerator.hpp"


TagGenerator::TagGenerator(int n, int s, int b, int r, int minDist, std::string XMLFilename) :_tagNumber(n), _tagSize(s), _borderSize(b), _retry(r)
{
	generate(minDist, XMLFilename);
}

void TagGenerator::generate(int minDist, std::string XMLFilename)
{
	cv::FileStorage fs(XMLFilename, cv::FileStorage::WRITE);

	int targetDist = minDist;
	int scale = 20;
	int retry = 0;
	
	int tagInd = 0;
	while(_tagList.size()<_tagNumber || targetDist<0)
	{
		cv::Mat bits;
		generateTag(bits);
		
		int hamm = hammingDistance(bits);
		if (hamm>targetDist)
		{
			retry = 0;
			
			int totalSize = _tagSize+2*_borderSize;
			cv::Mat bitsBorder = cv::Mat::zeros(totalSize,totalSize,CV_8UC1);

			for(int i=0;i<totalSize;++i)
			{
				bitsBorder.at<uchar>(i,0) = 0;
				bitsBorder.at<uchar>(0,1) = 0;
				bitsBorder.at<uchar>(i,totalSize-1) = 0;
				bitsBorder.at<uchar>(totalSize-1,i) = 0;
			}

			for(int row=0;row<_tagSize;++row)
			{
				for(int col=0;col<_tagSize;++col)
				{
					bitsBorder.at<uchar>(row+_borderSize,col+_borderSize) = bits.at<uchar>(row,col);
				}
			}

			_tagList.push_back(bitsBorder.clone());
			std::cout << bitsBorder << std::endl;

			std::stringstream tagStreamName;
			tagStreamName << "tag" << tagInd;
			
			fs << tagStreamName.str();
			fs << bitsBorder;


			cv::Mat resized = cv::Mat::zeros(totalSize*scale,totalSize*scale,CV_8UC1);
		
			for(int row=0;row<totalSize*scale;++row)
			{
				for(int col=0;col<totalSize*scale;++col)
				{
					resized.at<uchar>(row,col) = bitsBorder.at<uchar>(row/scale,col/scale);
				}
			}
		
			resized.convertTo(resized, CV_32FC1, 255.0);
		
			std::stringstream ss;
			ss << tagInd;
			std::string tagname = "tag"+ss.str()+".png";
			cv::imwrite(tagname, resized);

			tagInd++;
		}
		else
		{
			retry++;

			if (retry > _retry) targetDist--;
		}
	}

	fs << "size" << _tagSize;
	fs << "border" << _borderSize;
	fs << "hamming" << targetDist;
		
	return;
}

void TagGenerator::generateTag(cv::Mat &bits)
{
	bits = cv::Mat::zeros(_tagSize,_tagSize,CV_8UC1);

	for(int row=0;row<_tagSize;++row)
	{
		for(int col=0;col<_tagSize;++col)
		{
			double d = rand()/(double)RAND_MAX;
			int v = 0;
			if(d>0.5) v = 1;

			bits.at<uchar>(row,col) = v;
		}
	}

	return;
}

int TagGenerator::hammingDistance(cv::Mat m)
{
	int dist = _tagSize*_tagSize;

	cv::Mat r1 = m;
	cv::Mat r2 = rotate(r1);
	cv::Mat r3 = rotate(r2);
	cv::Mat r4 = rotate(r3);
	
	//compute self-rotation hamming distance
	dist = std::min(dist,hammingDistance(r1,r2));
	dist = std::min(dist,hammingDistance(r1,r3));
	dist = std::min(dist,hammingDistance(r1,r4));
	
	dist = std::min(dist,hammingDistance(r2,r3));
	dist = std::min(dist,hammingDistance(r2,r4));
	
	dist = std::min(dist,hammingDistance(r3,r4));

	//compute other element hamming distance
	std::vector<cv::Mat>::iterator it;
	for(it=_tagList.begin();it!=_tagList.end();++it)
	{
		dist = std::min(dist,hammingDistance(r1,*it));
		dist = std::min(dist,hammingDistance(r2,*it));
		dist = std::min(dist,hammingDistance(r3,*it));
		dist = std::min(dist,hammingDistance(r4,*it));
	}

	return dist;
}

int TagGenerator::hammingDistance(cv::Mat m, cv::Mat n)
{
	int dist = 0;
	
	for(int row=0;row<_tagSize;row++)
	{
		for(int col=0;col<_tagSize;col++)
		{
			if(m.at<uchar>(row,col)!=n.at<uchar>(row,col)) dist++;
		}
	}

	return dist;
}

cv::Mat TagGenerator::rotate(cv::Mat &in)
{
	cv::Mat out = in.clone();
    for(int i=0;i<_tagSize;++i)
    {
        for (int j=0;j<_tagSize;++j)
        {
            out.at<uchar>(i,j)=in.at<uchar>(_tagSize-j-1,i);
        }
    }

    return out;
}