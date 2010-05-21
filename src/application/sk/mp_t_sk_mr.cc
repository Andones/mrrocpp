// -------------------------------------------------------------------------
//                              task/mp_t_haptic.cc
//
// MP task for two robot haptic device
//
// -------------------------------------------------------------------------

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "mp/mp.h"
#include "ecp_mp_t_sk_mr.h"
#include "mp_t_sk_mr.h"
#include "lib/mrmath/mrmath.h"

#include <iostream>
#include <string>
#include <sstream>

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config) {
	return new sk_mr(_config);
}

sk_mr::sk_mr(lib::configurator &_config) :
	task(_config) {
}

void sk_mr::main_task_algorithm(void) {

	sr_ecp_msg->message("New sk_mr series");

	set_next_ecps_state((int) ecp_mp::task::ECP_GEN_BIAS_EDP_FORCE, (int) 5,
			"", 0, 1, lib::ROBOT_IRP6_ON_TRACK);

	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(
			1, 1, lib::ROBOT_IRP6_ON_TRACK, lib::ROBOT_IRP6_ON_TRACK);

	set_next_ecps_state((int) ecp_mp::task::ECP_GEN_TFF_NOSE_RUN, (int) 5, "",
			0, 1, lib::ROBOT_IRP6_ON_TRACK);

	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(
			1, 1, lib::ROBOT_IRP6_ON_TRACK, lib::ROBOT_IRP6_ON_TRACK);

	set_next_ecps_state((int) ecp_mp::task::ECP_GEN_EDGE_FOLLOW_FORCE, (int) 5,
			"", 0, 1, lib::ROBOT_IRP6_ON_TRACK);

	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(
			1, 1, lib::ROBOT_IRP6_ON_TRACK, lib::ROBOT_IRP6_ON_TRACK);

	sr_ecp_msg->message("END");

}

} // namespace task
} // namespace mp
} // namespace mrrocpp
