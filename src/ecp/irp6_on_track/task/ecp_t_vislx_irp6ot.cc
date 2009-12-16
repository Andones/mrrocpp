#include <stdio.h>
#include <unistd.h>
#include <map>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/task/ecp_mp_t_rcsc.h"


#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/common/generator/ecp_g_force.h"
#include "ecp/irp6_on_track/generator/ecp_g_vis_sac_lx.h"
#include "ecp_mp/sensor/ecp_mp_s_vis.h"
#include "ecp/irp6_on_track/task/ecp_t_vislx_irp6ot.h"

#include "ecp_mp/sensor/ecp_mp_s_vis_sac_lx.h"
#include "ecp_mp/sensor/ecp_mp_s_schunk.h"

#include <boost/foreach.hpp>

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

// KONSTRUKTORY
vislx::vislx(lib::configurator &_config) : task(_config)
{
	ecp_m_robot = new robot (*this);

	// Powolanie czujnikow
/*	sensor_m[lib::SENSOR_FORCE_ON_TRACK] =
		new ecp_mp_schunk_sensor (lib::SENSOR_FORCE_ON_TRACK, "[vsp_force_irp6ot]", *this);*/

	sensor_m[lib::SENSOR_CAMERA_SA] =
		new ecp_mp::sensor::vis_sac_lx (lib::SENSOR_CAMERA_SA, "[vsp_vis]", *this); //change if SENSOR_CAMERA_SA used for nonnn recog (vsp_vis_pbeolsac)

	// Konfiguracja wszystkich czujnikow
	BOOST_FOREACH(ecp_mp::sensor_item_t & sensor_item, sensor_m)
	{
		sensor_item.second->to_vsp.parameters=1; // biasowanie czujnika
		sensor_item.second->configure_sensor();
	}

	usleep(1000*100);
	sr_ecp_msg->message("ECP vis lx loaded");
}


void vislx::main_task_algorithm(void)
{
	generator::vis_sac_lx ynrlg(*this, 4);
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

task* return_created_ecp_task (lib::configurator &_config)
{
	return new irp6ot::task::vislx(_config);
}
}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

