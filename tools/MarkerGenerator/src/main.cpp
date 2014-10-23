#include "../include/TagGenerator.hpp"

#include <algorithm>

char* getCmdOption(char ** begin, char ** end, const std::string & option)
{
	char ** itr = std::find(begin, end, option);
	if (itr != end && ++itr != end)
	{
		return *itr;
	}
	return 0;
}

bool cmdOptionExists(char** begin, char** end, const std::string& option)
{
	return std::find(begin, end, option) != end;
}

int main(int argc, char **argv)
{
	int tagSize = 7; // Inside matrix size
	int borderSize = 1; // border size
	int hammingDistance = 10; // initial value for hamming distance search
	int dicSize = 50; // number of tag to generate
	int retry = 250; // number of retry before decrease hamming distance

	std::string XMLFilename = "generatedMarker.xml";

	char * tS = getCmdOption(argv, argv + argc, "-t");
	if (tS) tagSize = atoi(tS);

	char * bS = getCmdOption(argv, argv + argc, "-b");
	if (bS) borderSize = atoi(bS);

	char * dS = getCmdOption(argv, argv + argc, "-d");
	if (dS) dicSize = atoi(dS);

	char * hD = getCmdOption(argv, argv + argc, "-h");
	if (hD) hammingDistance = atoi(hD);

	char * r = getCmdOption(argv, argv + argc, "-r");
	if (r) retry = atoi(r);

	
	TagGenerator tg(dicSize,tagSize,borderSize,retry,hammingDistance, XMLFilename);

	return 0;
}