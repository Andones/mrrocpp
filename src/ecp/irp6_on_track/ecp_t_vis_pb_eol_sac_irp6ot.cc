#include <stdio.h>
#include <unistd.h>
#include <map>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
//#include "ecp_mp/ecp_mp_t_rcsc.h"


#include "ecp/irp6_on_track/ecp_local.h"
//#include "ecp/common/ecp_g_force.h"
#include "ecp/irp6_on_track/ecp_vis_pb_eol_sac_irp6ot.h"
#include "ecp_mp/ecp_mp_s_vis.h"
#include "ecp/irp6_on_track/ecp_t_vislx_irp6ot.h"

#include "ecp_mp/ecp_mp_s_vis_sac_lx.h"
//#include "ecp_mp/ecp_mp_s_schunk.h"

// KONSTRUKTORY
ecp_task_vislx_irp6ot::ecp_task_vislx_irp6ot(configurator &_config) : ecp_task(_config)
{
}

// methods for ECP template to redefine in concrete classes
void ecp_task_vislx_irp6ot::task_initialization(void)
{
	ecp_m_robot = new ecp_irp6_on_track_robot (*this);

	// Powolanie czujnikow

	sensor_m[SENSOR_CAMERA_SA] =
		new ecp_mp::sensor::ecp_mp_vis_sac_lx_sensor (SENSOR_CAMERA_SA, "[vsp_vis]", *this);

	// Konfiguracja wszystkich czujnikow

	for (std::map <SENSOR_ENUM, ::sensor*>::iterator sensor_m_iterator = sensor_m.begin();
		 sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
	{
		sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
		sensor_m_iterator->second->configure_sensor();
	}

	usleep(1000*100);
	sr_ecp_msg->message("ECP PBEOLSAC loaded");
}


void ecp_task_vislx_irp6ot::main_task_algorithm(void)
{
	ecp_vis_pb_eol_sac_irp6ot ynrlg(*this, 4);
	ynrlg.sensor_m = sensor_m;

	for(;;) {

		sr_ecp_msg->message("NOWA SERIA");
		ynrlg.Move();

	}
}

ecp_task* return_created_ecp_task (configurator &_config)
{
	return new ecp_task_vislx_irp6ot(_config);
}
