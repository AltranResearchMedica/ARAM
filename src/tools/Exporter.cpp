#include <ARAM/tools/Exporter.hpp>

namespace aram
{
        Exporter::Exporter():_count(0)
	{
	}

	Exporter& Exporter::operator++()
	{
		_count++;
		return *this;
	}
	
	void Exporter::write(const std::string &text, const std::string &file)
	{
	  std::ofstream f(file.c_str(),std::ios_base::out|std::ios_base::app);
		f << text << std::endl;
		f.close();
		
		return;
	}

	void Exporter::timer(float time, const std::string &file)
	{
		std::stringstream ss;
		ss << time;

		write(ss.str(),file);

		return;
	}

	void Exporter::occurence(int occ, const std::string &file)
	{
		std::stringstream ss;
		ss << occ;

		write(ss.str(),file);

		return;
	}

	void Exporter::error(float err, const std::string &file)
	{
		std::stringstream ss;
		ss << err;

		write(ss.str(),file);

		return;
	}

	void Exporter::error(float x1, float y1, float x2, float y2, const std::string &file)
	{
		std::stringstream ss;
		ss << x1 << " " << y1 << " " << x2 << " " << y2;

		write(ss.str(),file);

		return;
	}
				
	void Exporter::frame(cv::Mat &frame, const std::string &file)
	{			
		std::stringstream ss;
		ss << _count;

		std::string path = file + ss.str() + ".png";
			
		cv::imwrite(path,frame);

		return;
	}
};
