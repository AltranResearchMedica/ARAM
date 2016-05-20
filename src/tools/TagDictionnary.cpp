#include <ARAM/tools/TagDictionnary.hpp>
#include <bitset>

namespace aram
{
	Node::Node(int v):m_value(v)
	{
		p_left = NULL;
		p_right = NULL;
	}

	bool TagDictionnary::binaryTreeSearch(cv::Mat &bits)
	{
		bool res = true;
		Node *parentNode = p_root;

		for (int row = 0; row<bits.rows && res; ++row)
		{
			for (int col = 0; col<bits.cols && res; ++col)
			{
				int v = bits.at<uchar>(row, col);
				switch (v)
				{
				case 0:
					if (parentNode->p_left == NULL) res = false;

					parentNode = parentNode->p_left;
					break;

				case 1:
					if (parentNode->p_right == NULL) res = false;

					parentNode = parentNode->p_right;
					break;

				default:
					throw ARAMException(__LINE__, __FILE__, "TagDictionnary::insert", "Wrong value");
					break;
				}
			}
		}

		return res;
	}

	int TagDictionnary::hammingSearch(cv::Mat &tag)
	{
		int min = (m_tagSize + 2 * m_borderSize)*(m_tagSize + 2 * m_borderSize);
		int indMin = m_sets.size();
		for (unsigned int i = 0; i<m_sets.size(); ++i)
		{
			int hammDist = hammingDistance(m_sets[i], tag);
			if (hammDist<min)
			{
				min = hammDist;
				indMin = i;
			}
		}
		
		return (min < m_hammingDistance / 2) ? indMin : -1;
	}

	TagDictionnary * TagDictionnary::getInstance()
	{
		if (s_instance == NULL)
		{
			s_instance = new TagDictionnary("generatedMarker.xml");
		}

		return s_instance;
	}

	TagDictionnary *TagDictionnary::s_instance = NULL; 

	TagDictionnary::TagDictionnary(std::string dictionnaryFileName)
	{
		p_root = new Node(-1);
		p_root->p_left = NULL;
		p_root->p_right = NULL;

		read(dictionnaryFileName);
	}

	TagDictionnary& TagDictionnary::operator=(const TagDictionnary&)
	{
		return *s_instance;
	}

	TagDictionnary::TagDictionnary(const TagDictionnary&)
	{
	}

	void TagDictionnary::insert(cv::Mat m)
	{
		Node *parentNode = p_root;

		std::bitset<81> bits;
		for (int i = 0; i<m.rows; ++i)
		{
			for (int j = 0; j<m.cols; ++j)
			{
				if (m.at<uchar>(i, j) == 0) bits[i * m.rows + j] = 0;
				else bits[i * m.rows + j] = 1;
			}
		}

		for(int pos=0;pos<m.rows*m.cols;++pos)
		{
			int v = bits[pos];
			
			switch(v)
			{
			case 0:
				if(parentNode->p_left==NULL) parentNode->p_left = new Node(0);
				parentNode = parentNode->p_left;
				break;
			case 1:
				if(parentNode->p_right==NULL) parentNode->p_right = new Node(1);
				parentNode = parentNode->p_right;
				break;

			default :
				throw ARAMException(__LINE__, __FILE__, "TagDictionnary::insert", "Wrong value");
				break;
			}
		}

		m_sets.push_back(m);
	}

	int TagDictionnary::hammingDistance(cv::Mat m, cv::Mat n)
	{
		// Add assert to check matrix size
		int dist = 0;

		for (int row = 0; row<m_tagSize+2*m_borderSize; row++)
		{
			for (int col = 0; col<m_tagSize + 2 * m_borderSize; col++)
			{
				if (m.at<uchar>(row, col) != n.at<uchar>(row, col)) dist++;
			}
		}

		return dist;
	}

	void TagDictionnary::read(std::string dictionnaryFilename)
	{
		cv::FileStorage fs;
		if (!fs.open(dictionnaryFilename, cv::FileStorage::READ))
		{
			throw ARAMException(__LINE__, __FILE__, "TagDictionnary::read", "Unable to open generatedMarker.xml");
		}
		
		m_borderSize = (int)fs["border"];
		m_tagSize = (int)fs["size"];
		m_hammingDistance = (int)fs["hamming"];
		
		int tagId = 0;
		bool nextStep = false;
		do
		{
			nextStep = false;
			cv::Mat newTag;
			std::stringstream tagIdStream;
			tagIdStream << "tag" << tagId;
			fs[tagIdStream.str()] >> newTag;
			if (!newTag.empty())
			{
				insert(newTag);
				tagId++;
				nextStep = true;
			}
		}
		while (nextStep);

		return;
	}
};
