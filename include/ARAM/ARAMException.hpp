/**
*
* \file ARAMException.hpp
* \brief ARAM's exceptions
* \author Alexandre Kornmann
* \version 1.0
* \date 13 mars 2014
*
*/

#ifndef _ARAMEXCEPTION_HPP_
#define _ARAMEXCEPTION_HPP_

//std include
#include <exception>
#include <sstream>
#include <string>

//ARAM include
#include <ARAM/export.hpp>
#include <ARAM/typedef.hpp>

namespace aram
{
	/** 
	* ARAM's exceptions
	*/
	class ARAM_EXPORT ARAMException : public std::exception
	{
	public :
		/**
		* Constructor
		* 
		* \param[in] int line number
		* \param[in] std::string file name
		* \param[in] std::string function/method name
		* \param[in] std::string custom error message
		*/
		ARAMException(int, std::string, std::string, std::string);

		~ARAMException() throw()
		  {
		  };


		/**
		*  Implements std::exception::what
		*/
		const char* what() const throw();

	private :
		int m_line; /**< line number */
		std::string m_file; /**< file name */
		std::string m_func; /**< function name */
		std::string m_msg; /**< exception value */

		std::string m_res; /**< string to return */
	};
};

#endif
