#include <stdio.h>
#include <unistd.h>
#include <map>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"

#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/irp6_on_track/ecp_g_vis.h"
#include "ecp_mp/ecp_mp_s_vis.h"
#include "ecp/irp6_on_track/ecp_t_vis_irp6ot.h"


namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {


// KONSTRUKTORY
vis::vis(lib::configurator &_config) : task(_config)
{
}

// methods for ECP template to redefine in concrete classes
void vis::task_initialization(void)
{
    ecp_m_robot = new robot (*this);

    // Powolanie czujnikow
    sensor_m[lib::SENSOR_CAMERA_SA] =
        new ecp_mp::sensor::vis (lib::SENSOR_FORCE_ON_TRACK, "[vsp_section]", *this);

    // Konfiguracja wszystkich czujnikow
    for (ecp_mp::sensors_t::iterator sensor_m_iterator = sensor_m.begin();
            sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
    {
        sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
        sensor_m_iterator->second->configure_sensor();
    }

    usleep(1000*100);
    sr_ecp_msg->message("ECP loaded");
}


void vis::main_task_algorithm(void)
{
	generator::seven_eye_run_linear ynrlg(*this, 4);
	ynrlg.sensor_m = sensor_m;

	for(;;)
	{
		sr_ecp_msg->message("NOWA SERIA");
		ynrlg.Move();

	}
}

}
} // namespace irp6ot

namespace common {
namespace task {

task* return_created_ecp_task (lib::configurator &_config)
{
	return new irp6ot::task::vis(_config);
}
}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

