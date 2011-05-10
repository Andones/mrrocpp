/*!
 * @file sr_edp.cc
 * @brief System reporter class for EDP - definitions.
 *
 * @author Piotr Trojanek <piotr.trojanek@gmail.com>
 *
 * @ingroup LIB
 */

#ifndef SR_EDP_H_
#define SR_EDP_H_

#include "base/lib/sr/srlib.h"

namespace mrrocpp {
namespace lib {

//! SR class for use in EDP
class sr_edp : public sr
{
protected:
	//! Interpret the status code into a text message.
	virtual void interpret(char * description, error_class_t message_type, uint64_t error_code0, uint64_t error_code1);

public:
	/**
	 * Constructor
	 * @param process_type reporting process type
	 * @param process_name reporting process name
	 * @param sr_channel_name name of the SR communication channel
	 * @param _multi_thread flag for selecting multi-threaded variant
	 */
	sr_edp(process_type_t process_type, const std::string & process_name, const std::string & sr_channel_name);

	//! Sends a message to SR adequate for given non fatal error.
	virtual void error_message(const mrrocpp::lib::exception::mrrocpp_non_fatal_error & _e);

	//! Sends a message to SR adequate for given fatal error.
	virtual void error_message(const mrrocpp::lib::exception::mrrocpp_fatal_error & _e);

	//! Sends a message to SR adequate for given system error.
	virtual void error_message(const mrrocpp::lib::exception::mrrocpp_system_error & _e);

/*	void message(error_class_t message_type, uint64_t error_code);
	void message(error_class_t message_type, uint64_t error_code0, uint64_t error_code1);
	void message(error_class_t message_type, uint64_t error_code, const std::string & text);
	void message(const std::string & text);
	void message(error_class_t message_type, const std::string & text);*/

};

} // namespace lib
} // namespace mrrocpp

#endif /* SR_EDP_H_ */
