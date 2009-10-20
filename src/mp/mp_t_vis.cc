// ------------------------------------------------------------------------
//
//                      MASTER PROCESS (MP) - main()
//
// ------------------------------------------------------------------------

#include <stdio.h>
#include <unistd.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/srlib.h"
#include "mp/mp.h"
#include "mp/mp_t_vis.h"
#include "mp/mp_g_vis.h"
#include "mp/mp_g_force.h"
#include "ecp_mp/ecp_mp_s_vis.h"
#include "ecp_mp/ecp_mp_s_schunk.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"

#include <boost/foreach.hpp>

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task (lib::configurator &_config)
{
	return new vis(_config);
}

vis::vis(lib::configurator &_config) : task(_config)
{
	// Powolanie czujnikow
	sensor_m[lib::SENSOR_FORCE_ON_TRACK] =
		new ecp_mp::sensor::schunk (lib::SENSOR_FORCE_ON_TRACK, "[vsp_force_irp6ot]", *this);

	sensor_m[lib::SENSOR_CAMERA_SA] =
		new ecp_mp::sensor::vis (lib::SENSOR_CAMERA_SA, "[vsp_vis]", *this);

	// Konfiguracja wszystkich czujnikow
	BOOST_FOREACH(ecp_mp::sensor_item_t & sensor_item, sensor_m) {
		sensor_item.second->to_vsp.parameters=1; // biasowanie czujnika
		sensor_item.second->configure_sensor();
	}
}

void vis::main_task_algorithm(void)
{
	generator::seven_eye eyegen(*this, 4);
	eyegen.robot_m[lib::ROBOT_IRP6_ON_TRACK] = robot_m[lib::ROBOT_IRP6_ON_TRACK];
	eyegen.sensor_m[lib::SENSOR_CAMERA_SA] = sensor_m[lib::SENSOR_CAMERA_SA];

	sr_ecp_msg->message("New loop");

	//seven_eye eyegen(*this, 4);
	//eyegen.robot_m = robot_m;
	//eyegen.sensor_m = sensor_m;
	//po cholere biasujemy jeszcze raz te czujniki i co to w ogole oznacza???
	BOOST_FOREACH(ecp_mp::sensor_item_t & sensor_item, sensor_m) {
		sensor_item.second->to_vsp.parameters=1; // biasowanie czujnika
		sensor_item.second->configure_sensor();
	}

	set_next_ecps_state ((int) ecp_mp::task::ECP_GEN_TEACH_IN, 0, "../trj/rcsc/irp6ot_ap_1.trj", 1, lib::ROBOT_IRP6_ON_TRACK);
	sr_ecp_msg->message("after genload");

	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
	(1, 1, lib::ROBOT_IRP6_ON_TRACK, lib::ROBOT_IRP6_ON_TRACK);
	sr_ecp_msg->message("after genstart");

	set_next_ecps_state ((int) ecp_mp::task::ECP_GEN_TEACH_IN, 0, "../trj/rcsc/irp6ot_ap_2.trj", 1, lib::ROBOT_IRP6_ON_TRACK);
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
	(1, 1, lib::ROBOT_IRP6_ON_TRACK, lib::ROBOT_IRP6_ON_TRACK);


	// wlaczenie generatora transparentnego w obu robotach
	set_next_ecps_state ((int) ecp_mp::task::ECP_GEN_TRANSPARENT, (int) 1, "", 1, lib::ROBOT_IRP6_ON_TRACK);


	// opcjonalne serwo wizyjne
	//if (mode)
	//{

	eyegen.Move();
	//}

	send_end_motion_to_ecps (1, lib::ROBOT_IRP6_ON_TRACK);

	// wlaczenie generatora zacisku na kostce w robocie irp6ot
	set_next_ecps_state ((int) ecp_mp::task::ECP_GEN_TFF_RUBIK_GRAB, (int) ecp_mp::task::RCSC_RG_FROM_OPEARTOR_PHASE_1, "", 1, lib::ROBOT_IRP6_ON_TRACK);
	// uruchomienie generatora empty_gen
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
	(1, 1, lib::ROBOT_IRP6_ON_TRACK, lib::ROBOT_IRP6_ON_TRACK);
	// wlaczenie generatora zacisku na kostce w robocie irp6ot
	set_next_ecps_state ((int) ecp_mp::task::ECP_GEN_TFF_RUBIK_GRAB, (int) ecp_mp::task::RCSC_RG_FROM_OPEARTOR_PHASE_2, "", 1, lib::ROBOT_IRP6_ON_TRACK);
	// uruchomienie generatora empty_gen
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
	(1, 1, lib::ROBOT_IRP6_ON_TRACK, lib::ROBOT_IRP6_ON_TRACK);

}


} // namespace task
} // namespace mp
} // namespace mrrocpp
