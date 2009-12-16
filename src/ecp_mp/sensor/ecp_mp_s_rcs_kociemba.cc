// -------------------------------------------------------------------------
// Proces: 	EFFECTOR CONTROL PROCESS (lib::ECP)
// Plik:	ecp_mp_s_rcs_kociemba.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:	metody klasy ecp_mp_rcs_kociemba dla czujnika znajdujacego rozwiazanie kostki Rubika alg. Kociemby
// Autor:	jsalacka
// Data:	25.03.2007
// -------------------------------------------------------------------------

/********************************* INCLUDES *********************************/
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#if !defined(USE_MESSIP_SRR)
#include <devctl.h>
#else
#warning file not ported to MESSIP yet
#endif

#include "lib/com_buf.h"

#include "lib/srlib.h"

#include "ecp_mp/sensor/ecp_mp_s_rcs_kociemba.h"

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

/***************************** CONSTRUCTOR ********************************/
rcs_kociemba::rcs_kociemba(lib::SENSOR_t _sensor_name, const char* _section_name, task::task& _ecp_mp_object)
	: sensor(_sensor_name, _section_name, _ecp_mp_object) {
	// Ustawienie wielkosci przesylanej unii.
	union_size = sizeof(image.sensor_union.rcs);
	// Wyzerowanie odczytow.
	image.sensor_union.rcs.cube_solution[0] = '\0';
} // end:

/************************** CONFIGURE SENSOR ******************************/
void rcs_kociemba::configure_sensor() {
#if !defined(USE_MESSIP_SRR)
	// Rozkaz konfiguracjii czujnika.
	devmsg.to_vsp.i_code= lib::VSP_CONFIGURE_SENSOR;
	memcpy(&devmsg.to_vsp.rcs, &to_vsp.rcs, union_size);
	//strncpy(devmsg.to_vsp.sensor_union.rcs.cube_state, to_vsp.sensor_union.rcs.cube_state,54);
	// Wyslanie polecenia do procesu VSP.
	if (devctl(sd, DEVCTL_RW, &devmsg, sizeof(lib::DEVCTL_MSG), NULL) == 9)
		throw sensor_error(lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);
#endif
} // end: configure_sensor

/************************** INITIATE READING *********************************/
void rcs_kociemba::initiate_reading(){
#if !defined(USE_MESSIP_SRR)
	devmsg.to_vsp.i_code= lib::VSP_INITIATE_READING;
	memcpy(&devmsg.to_vsp.rcs, &to_vsp.rcs, union_size);
	if (devctl(sd, DEVCTL_RW, &devmsg, sizeof(lib::DEVCTL_MSG), NULL) == 9) {
		image.sensor_union.rcs.init_mode = lib::RCS_INIT_FAILURE;
		throw sensor_error(lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);
	}
	if (devmsg.from_vsp.vsp_report == lib::VSP_REPLY_OK) {
		image.sensor_union.rcs.init_mode = lib::RCS_INIT_SUCCESS;
	} else {
		image.sensor_union.rcs.init_mode = lib::RCS_INIT_FAILURE;
		printf("ECP_MP KR initiate_reading: Reply from VSP not OK!\n");
	}
#endif
}

/***************************** GET  READING *********************************/
void rcs_kociemba::get_reading() {
#if !defined(USE_MESSIP_SRR)
	if(read(sd, &from_vsp, sizeof(lib::VSP_ECP_MSG)) == -1) {
		image.sensor_union.rcs.cube_solution[0] = '\0';
		image.sensor_union.rcs.reading_mode = lib::RCS_SOLUTION_NOTFOUND;
		sr_ecp_msg.message (lib::SYSTEM_ERROR, CANNOT_READ_FROM_DEVICE, VSP_NAME.c_str());
	}
	// jesli odczyt sie powiodl, przepisanie pol obrazu z bufora komunikacyjnego do image;
	if(from_vsp.vsp_report == lib::VSP_REPLY_OK) {
		// Przepisanie pol obrazu z bufora komunikacyjnego do image.
		memcpy(&image.sensor_union.rcs, &from_vsp.comm_image.sensor_union.rcs, union_size);
	} else {
		image.sensor_union.rcs.cube_solution[0] = '\0';
		image.sensor_union.rcs.reading_mode = lib::RCS_SOLUTION_NOTFOUND;
		printf("ECP_MP KC get_reading: Reply from VSP not OK!\n");
	}
#endif
} // end: get_reading

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp

