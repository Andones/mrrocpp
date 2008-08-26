/*
 * ecp_g_spots_recognition.cc
 *
 *  Created on: Aug 26, 2008
 *      Author: ghard
 */

#include "ecp/irp6_on_track/ecp_g_spots_recognition.h"
#include <unistd.h>


ecp_spots_generator::ecp_spots_generator (ecp_task& _ecp_task)
        : ecp_smooth_generator (_ecp_task, false)
{

}

bool ecp_spots_generator::first_step()
{
	sensor = (ecp_mp_cvfradia_sensor *)sensor_m[SENSOR_CVFRADIA];

	comm_struct.command = 38;
	comm_struct.i_code = VSP_GET_READING;
	sensor->send_reading(comm_struct);
	usleep(2000);

	return ecp_smooth_generator::first_step();
}

