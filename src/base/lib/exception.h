/*!
 * @file exception.h
 * @brief Exception declarations.
 *
 * @author Piotr Trojanek <piotr.trojanek@gmail.com>
 * @author Tomasz Winiarski <tomrobotics@gmail.com>
 * @author tkornuta <tkornuta@ia.pw.edu.com>
 *
 * @ingroup LIB
 */

#ifndef __TRANSFORMER_ERROR_H
#define __TRANSFORMER_ERROR_H

#include <stdint.h>
#include <boost/exception/all.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <boost/thread/thread_time.hpp>
#include <boost/date_time/posix_time/time_formatters.hpp>

namespace mrrocpp {
namespace lib {

// TODO: Move to the mrrocpp::lib::exception namespace.
/*!
 * Classes of errors in the MRROC++ framework.
 */
typedef enum _ERROR_CLASS_T
{
	NEW_MESSAGE, SYSTEM_ERROR, FATAL_ERROR, NON_FATAL_ERROR
} error_class_t;

}
}

namespace mrrocpp {
namespace lib {
namespace exception {

//! A single line description of error.
typedef boost::error_info <struct mrrocpp_error_description_, char const *> mrrocpp_error_description;

//! Time when error was detected.
typedef boost::error_info <struct timestamp, boost::system_time> mrrocpp_error_time;

//! Convert exception's timestamp to human-readable string
inline std::string to_string(mrrocpp_error_time const & e)
{
	return boost::posix_time::to_simple_string(e.value());
}

/*!
 * \brief Base class for all system exceptions/errors.
 * \author tkornuta
 * \date 12.05.2011
 */
template <error_class_t ercl>
class mrrocpp_error : virtual public std::exception, virtual public boost::exception
{
public:
	/*!
	 * Class of the error.
	 */
	const error_class_t error_class;

	/*!
	 * Constructor.
	 */
	mrrocpp_error() :
		error_class(ercl)
	{
		// Add it to diagnostic information.
		*this << mrrocpp_error_time(boost::get_system_time());
	}

	/*!
	 * Destructor.
	 */
	~mrrocpp_error() throw ()
	{
	}

	/*!
	 * Returns diagnostic information.
	 */
	virtual const char* what() const throw ()
	{
		return diagnostic_information_what(*this);
	}
};

/*!
 * \brief Base class for all system errors.
 * \author tkornuta
 * \date 12.05.2011
 */
typedef mrrocpp_error <SYSTEM_ERROR> mrrocpp_system_error;
/*!
 * \brief Base class for all fatal errors.
 * \author tkornuta
 * \date 12.05.2011
 */
typedef mrrocpp_error <FATAL_ERROR> mrrocpp_fatal_error;

/*!
 * \brief Base class for all non fatal errors.
 * \author tkornuta
 * \date 12.05.2011
 */
typedef mrrocpp_error <NON_FATAL_ERROR> mrrocpp_non_fatal_error;

/*!
 * Macro for registration of MRROC++ system errors.
 *
 * \param CLASS_NAME Name of the error (class name).
 * \param DESCRIPTION Description added to the mrrocpp_error_description error info field.
 *
 * \author tkornuta
 * \date 12.05.2011
 */
#define REGISTER_SYSTEM_ERROR(CLASS_NAME, DESCRIPTION) \
struct CLASS_NAME : virtual mrrocpp::lib::exception::mrrocpp_system_error \
{ \
	CLASS_NAME() { *this << mrrocpp::lib::exception::mrrocpp_error_description(DESCRIPTION); } \
	~CLASS_NAME() throw () { } \
};

/*!
 * Macro for registration of MRROC++ fatal errors.
 *
 * \param CLASS_NAME Name of the error (class name).
 * \param DESCRIPTION Description added to the mrrocpp_error_description error info field.
 *
 * \author tkornuta
 * \date 12.05.2011
 */
#define REGISTER_FATAL_ERROR(CLASS_NAME, DESCRIPTION) \
struct CLASS_NAME : virtual mrrocpp::lib::exception::mrrocpp_fatal_error \
{ \
	CLASS_NAME() { *this << mrrocpp::lib::exception::mrrocpp_error_description(DESCRIPTION); } \
	~CLASS_NAME() throw () { } \
};

/*!
 * Macro for registration of MRROC++ non fatal errors.
 *
 * \param CLASS_NAME Name of the error (class name).
 * \param DESCRIPTION Description added to the mrrocpp_error_description error info field.
 *
 * \author tkornuta
 * \date 12.05.2011
 */
#define REGISTER_NON_FATAL_ERROR(CLASS_NAME, DESCRIPTION) \
struct CLASS_NAME : virtual mrrocpp::lib::exception::mrrocpp_non_fatal_error \
{ \
	CLASS_NAME() { *this << mrrocpp::lib::exception::mrrocpp_error_description(DESCRIPTION); } \
	~CLASS_NAME() throw () { } \
};

/*!
 * Macro for handling MRROC++ system errors.
 *
 * \param ERROR Exception derived from the mrrocpp_system_error classes.
 *
 * \author tkornuta
 * \date 27.10.2011
 */

#define HANDLE_MRROCPP_SYSTEM_ERROR(ERROR) \
	std::cout<< ERROR.what() << std::endl; \
	msg->message(ERROR);

/*!
 * Macro for handling MRROC++ fatal errors.
 *
 * \param ERROR Exception derived from the mrrocpp_fatal_error classes.
 *
 * \author tkornuta
 * \date 27.10.2011
 */

#define HANDLE_MRROCPP_FATAL_ERROR(ERROR) \
	std::cout<< ERROR.what() << std::endl; \
	msg->message(ERROR);

/*!
 * Macro for handling MRROC++ non-fatal errors.
 *
 * \param ERROR Exception derived from the mrrocpp_non_fatal_error classes.
 *
 * \author tkornuta
 * \date 27.10.2011
 */

#define HANDLE_MRROCPP_NON_FATAL_ERROR(ERROR) \
	std::cout<< ERROR.what() << std::endl; \
	msg->message(ERROR);


/********************************** OLD MRROC++ ERRORS **********************************/

/**
 * System error (inter-process communication, filesystem, etc.)
 */
class System_error
{
};

/**
 * Fatal exception in framework or application
 */
class Fatal_error
{
public:
	//! Servo error number (1)
	const uint64_t error0;

	//! Servo error number (2)
	const uint64_t error1;

	/**
	 * Constructor
	 * @param err_no_0 servo error number (1)
	 * @param err_no_1 servo error number (2)
	 * @return
	 */
	Fatal_error(uint64_t err_no_0, uint64_t err_no_1);
};

/**
 * Non-fatal errors (type 1)
 * @author Tomasz Winiarski <tomrobotics@gmail.com>
 */
class NonFatal_error_1
{
public:
	//! Error in coordinate calculations
	const uint64_t error;

	/**
	 * Constructor
	 * @param err_no error value
	 */
	NonFatal_error_1(uint64_t err_no);
};

/**
 * Non-fatal errors (type 2)
 * @author Tomasz Winiarski <tomrobotics@gmail.com>
 */
class NonFatal_error_2
{
public:
	//! Error in coordinate calculations
	const uint64_t error;

	/**
	 * Constructor
	 * @param err_no error value
	 */
	NonFatal_error_2(uint64_t err_no);
};

/**
 * Non-fatal errors (type 3)
 * @author Tomasz Winiarski <tomrobotics@gmail.com>
 */
class NonFatal_error_3
{
public:
	//! Error in coordinate calculations
	const uint64_t error;

	/**
	 * Constructor
	 * @param err_no error value
	 */
	NonFatal_error_3(uint64_t err_no);
};

/**
 * Non-fatal errors (type 4)
 * @author Tomasz Winiarski <tomrobotics@gmail.com>
 */
class NonFatal_error_4
{
public:
	//! Error in coordinate calculations
	const uint64_t error;

	/**
	 * Constructor
	 * @param err_no error value
	 */
	NonFatal_error_4(uint64_t err_no);
};

} // namespace exception
} // namespace common
} // namespace mrrocpp

#endif
