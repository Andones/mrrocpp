#include "mp/mp_delay_ms_condition.h"

#include "unistd.h"

namespace mrrocpp {
namespace mp {
namespace generator {

// condition to wait for desired time in ms

delay_ms_condition::delay_ms_condition(task::mp_task& _mp_task, int _ms_delay): base (_mp_task)
{
	local_timer = new timer();
	configure(_ms_delay);
}

delay_ms_condition::~delay_ms_condition()
{
	delete local_timer;
}

void delay_ms_condition::configure (int _ms_delay)
{
	ms_delay = _ms_delay;
}

bool delay_ms_condition::first_step ()
{
	local_timer->timer_start();
	return true;
}

bool delay_ms_condition::next_step ()
{
	local_timer->timer_stop();
	local_timer->get_time(&sec);
	if (1000*sec > (float) ms_delay)
		return false;
	delay (20);
	local_timer->timer_stop();
	local_timer->get_time(&sec);
	if (1000*sec > (float) ms_delay)
		return false;
	return true;
}

} // namespace generator
} // namespace mp
} // namespace mrrocpp

