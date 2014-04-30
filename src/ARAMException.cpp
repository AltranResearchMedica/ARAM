#include <ARAM/ARAMException.hpp>

#include <iostream>
namespace aram
{
	ARAMException::ARAMException(int l, std::string file, std::string func, std::string m):_line(l),_file(file),_func(func),_msg(m)
	{
		std::stringstream ss;
		ss << "[" <<_file << " (line " <<_line << ")] " << _func << " : " << _msg << std::endl; 
		std::cerr << ss.str() << std::endl;
	}

  const char* ARAMException::what() const throw()
	{
		return "";
	}
};
