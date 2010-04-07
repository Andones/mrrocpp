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
#include "ecp_mp_t_swarmitfix.h"
#include "mp_t_swarmitfix.h"

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task (lib::configurator &_config)
{
	return new swarmitfix(_config);
}

swarmitfix::swarmitfix(lib::configurator &_config) : task(_config)
{
}


void swarmitfix::main_task_algorithm(void)
{


   	sr_ecp_msg->message("New swarmitfix series");


   	// wlaczenie generatora transparentnego w obu robotach
   	set_next_ecps_state ((int) ecp_mp::task::ECP_GEN_TRANSPARENT, (int) 0, "", 1, lib::ROBOT_SPKM);
   	set_next_ecps_state ((int) ecp_mp::task::ECP_GEN_TRANSPARENT, (int) 0, "", 1, lib::ROBOT_SMB);
   	set_next_ecps_state ((int) ecp_mp::task::ECP_GEN_TRANSPARENT, (int) 0, "", 1, lib::ROBOT_SHEAD);

   	sr_ecp_msg->message("1");
	send_end_motion_to_ecps (1, lib::ROBOT_SPKM);
	sr_ecp_msg->message("2");
	set_next_ecps_state ((int) ecp_mp::task::ECP_GEN_SLEEP, (int) 5, "", 1, lib::ROBOT_SPKM);
	sr_ecp_msg->message("3");
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
			(1, 1, lib::ROBOT_SPKM, lib::ROBOT_SPKM);


	sr_ecp_msg->message("4");

   	send_end_motion_to_ecps (2, lib::ROBOT_SMB, lib::ROBOT_SHEAD);


}


} // namespace task
} // namespace mp
} // namespace mrrocpp
