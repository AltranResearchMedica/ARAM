#include <ARAM/tools/contourClosing.hpp>

namespace aram
{
	/**
	* Wrapper for closing operation
	*/
	void close(cv::Mat &toClose, CLOSURE_TYPE type)
	{
		switch(type)
		{
		case ARAM_NAIVE:
			NaiveContourClosing nc;
			nc.close(toClose);
			break;

		case ARAM_MilgramContourClosing:
		default:
			MilgramContourClosing &mc = MilgramContourClosing::getInstance();
			mc.close(toClose);
			break;
		}

		return;
	}
};
