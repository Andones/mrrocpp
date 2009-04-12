#include <string.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"
#include <fstream>

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_s_wiimote.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_t_wii.h"
#include "lib/mathtr.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {

ecp_task_wii::ecp_task_wii(configurator &_config) : ecp_task(_config) {};

void ecp_task_wii::task_initialization(void)
{
	ecp_m_robot = new ecp_irp6_on_track_robot (*this);
    sr_ecp_msg->message("ECP loaded");

	//create Wii-mote virtual sensor object
	sensor_m[SENSOR_WIIMOTE] = new ecp_mp::sensor::wiimote(SENSOR_WIIMOTE, "[vsp_wiimote]", *this, sizeof(sensor_image_t::sensor_union_t::wiimote_t));
	//configure the sensor
	sensor_m[SENSOR_WIIMOTE]->configure_sensor();
}

void ecp_task_wii::main_task_algorithm(void)
{
	double* firstPosition;

    sg = new common::ecp_smooth_generator(*this,true);
    eg = new ecp_wii_generator(*this);
    
    eg->sensor_m[SENSOR_WIIMOTE] = sensor_m[SENSOR_WIIMOTE];
	firstPosition = eg->getFirstPosition();
	
	sg->reset();
	sg->load_coordinates(XYZ_EULER_ZYZ,firstPosition[0],firstPosition[1],firstPosition[2],firstPosition[3],firstPosition[4],firstPosition[5],firstPosition[6],firstPosition[7]);
	sg->Move();

	while(1)
	{    
    	eg->Move();
    }
    
    ecp_termination_notice();
}

} // namespace irp6ot

namespace common {

ecp_task* return_created_ecp_task (configurator &_config)
{
	return new irp6ot::ecp_task_wii(_config);
}


} // namespace common
} // namespace ecp
} // namespace mrrocpp

