// ------------------------------------------------------------------------
//   ecp_t_sk.cc - zadanie eksperymentalne by yoyek
//
// Ostatnia modyfikacja: 2007
// ------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_postument/ecp_local.h"
#include "ecp/common/ecp_g_force.h"
#include "ecp_mp/ecp_mp_s_schunk.h"
#include "ecp/common/ecp_t_tw.h"


namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

// KONSTRUKTORY
tw::tw(lib::configurator &_config) : base(_config)
{
	nrg = NULL;
	yefg = NULL;
}

// methods for ECP template to redefine in concrete classes
void tw::task_initialization(void)
{

	// the robot is choose dependendant on the section of configuration file sent as argv[4]
	if (strcmp(config.section_name, "[ecp_irp6_on_track]") == 0)
	{
		ecp_m_robot = new irp6ot::ecp_irp6_on_track_robot (*this);
	}
	else if (strcmp(config.section_name, "[ecp_irp6_postument]") == 0)
	{
		ecp_m_robot = new irp6p::ecp_irp6_postument_robot (*this);
	}

	usleep(1000*100);

	nrg = new generator::tff_nose_run(*this, 8);



	nrg->configure_behaviour(lib::UNGUARDED_MOTION, lib::UNGUARDED_MOTION, lib::CONTACT, lib::UNGUARDED_MOTION, lib::UNGUARDED_MOTION, lib::UNGUARDED_MOTION);
	//	nrg->configure_pulse_check (false);
	//	nrg->configure_velocity (0.0, 0.0, 0.4, 0.0, 0.0, 0.0);
	//	nrg->configure_force (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	nrg->configure_reciprocal_damping (FORCE_RECIPROCAL_DAMPING, FORCE_RECIPROCAL_DAMPING, FORCE_RECIPROCAL_DAMPING,
			TORQUE_RECIPROCAL_DAMPING, TORQUE_RECIPROCAL_DAMPING, TORQUE_RECIPROCAL_DAMPING);
	nrg->configure_inertia (FORCE_INERTIA, FORCE_INERTIA, FORCE_INERTIA/5, TORQUE_INERTIA, TORQUE_INERTIA, TORQUE_INERTIA);
	//	nrg->configure_inertia (0, 0, 0, 0, 0, 0);



	befg = new generator::bias_edp_force(*this);


	switch (ecp_m_robot->robot_name)
	{
	case ROBOT_IRP6_ON_TRACK:
		sr_ecp_msg->message("ECP tw ex irp6ot loaded");
		break;
	case ROBOT_IRP6_POSTUMENT:
		sr_ecp_msg->message("ECP tw ex irp6p loaded");
		break;
	default:
		fprintf(stderr, "%s:%d unknown robot type\n", __FILE__, __LINE__);
	}

	// sprawdzenie dodatkowej opcji w konfiguracji dotyczacej uruchomienie zapamietywania trajektorii do pliku
	if (config.exists("save_activated"))
	{
		save_activated = (bool) config.return_int_value("save_activated");
	}
	else
	{
		save_activated = false;
	}

}


void tw::main_task_algorithm(void)
{
	//   weight_meassure_generator wmg(*this, 0.3, 2);


	sr_ecp_msg->message("NOWA SERIA");
	sr_ecp_msg->message("FORCE SENSOR BIAS");
	befg->Move();
	sr_ecp_msg->message("Wodzenie do pozycji sledzenia konturu");
	sr_ecp_msg->message("Nastepny etap - nacisnij PULSE ECP trigger");
	nrg->Move();

}

base* return_created_ecp_task (lib::configurator &_config)
{
	return new tw(_config);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp
