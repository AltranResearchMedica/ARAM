#include <ARAM/ARAMException.hpp>

#include <iostream>
namespace aram
{
	ARAMException::ARAMException(int l, std::string file, std::string func, std::string m):m_line(l),m_file(file),m_func(func),m_msg(m)
	{
		std::stringstream ss;
		ss << "[" <<m_file << " (line " <<m_line << ")] " << m_func << " : " << m_msg << std::endl; 
		std::cerr << ss.str() << std::endl;
	}

	const char* ARAMException::what() const throw()
	{
		return "";
	}
};
