#include <stdio.h>
#include <unistd.h>
#include <map>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"


#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/common/ecp_g_force.h"
#include "ecp/irp6_on_track/ecp_g_vis_sac_lx.h"
#include "ecp_mp/ecp_mp_s_vis.h"
#include "ecp/irp6_on_track/ecp_t_vislx_irp6ot.h"

#include "ecp_mp/ecp_mp_s_vis_sac_lx.h"
#include "ecp_mp/ecp_mp_s_schunk.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

// KONSTRUKTORY
ecp_task_vislx_irp6ot::ecp_task_vislx_irp6ot(configurator &_config) : ecp_task(_config)
{
}

// methods for ECP template to redefine in concrete classes
void ecp_task_vislx_irp6ot::task_initialization(void)
{
	ecp_m_robot = new ecp_irp6_on_track_robot (*this);

	// Powolanie czujnikow
/*	sensor_m[SENSOR_FORCE_ON_TRACK] =
		new ecp_mp_schunk_sensor (SENSOR_FORCE_ON_TRACK, "[vsp_force_irp6ot]", *this);*/

	sensor_m[SENSOR_CAMERA_SA] =
		new ecp_mp::sensor::vis_sac_lx (SENSOR_CAMERA_SA, "[vsp_vis]", *this); //change if SENSOR_CAMERA_SA used for nonnn recog (vsp_vis_pbeolsac)

	// Konfiguracja wszystkich czujnikow

	for (std::map <SENSOR_ENUM, ::sensor*>::iterator sensor_m_iterator = sensor_m.begin();
		 sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
	{
		sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
		sensor_m_iterator->second->configure_sensor();
	}


	usleep(1000*100);
	sr_ecp_msg->message("ECP vis lx loaded");
}


void ecp_task_vislx_irp6ot::main_task_algorithm(void)
{
	generator::ecp_vis_sac_lx_generator ynrlg(*this, 4);
	ynrlg.sensor_m = sensor_m;

	common::generator::bias_edp_force befg(*this);

	for(;;) {

		sr_ecp_msg->message("NOWA SERIA");

		sr_ecp_msg->message("FORCE SENSOR BIAS");
		befg.Move();

		ynrlg.Move();

	}
}

}
} // namespace irp6ot

namespace common {
namespace task {

ecp_task* return_created_ecp_task (configurator &_config)
{
	return new irp6ot::task::ecp_task_vislx_irp6ot(_config);
}
}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

