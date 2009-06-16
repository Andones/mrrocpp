// ------------------------------------------------------------------------
// Plik:				mis_fun.h
// System:		QNX/MRROC++   v. 6.3
// Opis:			miscelangeous functions
// Modyfikacja:
// Jej autor:
// Data:			2006
// ------------------------------------------------------------------------

#ifndef __MIS_FUN_H
#define __MIS_FUN_H

#include <pthread.h>
#include <string.h>

#include "lib/impconst.h"

namespace mrrocpp {
namespace lib {

// setting of thread priority
void set_thread_priority(pthread_t thread, int sched_priority_l);

// by Y
inline void copy_frame(lib::frame_tab destination_frame, const lib::frame_tab source_frame)
{
	memcpy(destination_frame, source_frame, sizeof(lib::frame_tab));
	/*
	for (int   column = 0; column < 4; column++)
		for (int row = 0; row < 3; row++)
			destination_frame[column][row] = source_frame[column][row];
	*/
}

} // namespace lib
} // namespace mrrocpp

#endif
