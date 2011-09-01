/**
 * \file logger.h
 * \brief Logging utilities.
 * \bug Not multi-thread safe
 *
 * \author Mateusz Boryń <mateusz.boryn@gmail.com>
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include <deque>
#include <boost/thread/mutex.hpp>

#include "log_message.h"

#include "base/lib/mrmath/homog_matrix.h"	// TODO: remove

namespace logger {

/** Is log enabled*/
extern bool log_enabled, log_dbg_enabled;

/**
 * Print message to the console only if logEnabled is set to true.
 * @param fmt printf-like format
 */
void log(const char *fmt, ...)
// Check if arguments follow printf-like format (see GCC documentation).
// 1 - number of argument with string format, 2 - first variable argument to check
__attribute__ ((format (printf, 1, 2)))
;

/**
 * Print Homog_matrix.
 * @param hm
 */
void log(const mrrocpp::lib::Homog_matrix & hm);

/**
 * Print message to the console only if logDbgEnabled is set to true.
 * @param fmt printf-like format
 */
void log_dbg(const char *fmt, ...)
// Check if arguments follow printf-like format (see GCC documentation).
// 1 - number of argument with string format, 2 - first variable argument to check
__attribute__ ((format (printf, 1, 2)))
;

/**
 * Print Homog_matrix.
 * @param hm
 */
void log_dbg(const mrrocpp::lib::Homog_matrix & hm);


class logger_client {
public:
	logger_client(int max_queue_size);
	~logger_client();

	void log(const log_message& msg);

	void operator()();
protected:

private:
	void connect();
	void disconnect();

	const int max_queue_size;
	std::deque<const log_message*> queue;
	uint32_t current_message_number;
	boost::mutex queue_mutex;
	boost::mutex notify_mutex;
	bool terminate;
};

} // namespace logger

#endif /* LOGGER_H_ */
