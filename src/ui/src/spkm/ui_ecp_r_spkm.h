// -------------------------------------------------------------------------
//                            robot.h
// Definicje struktur danych i metod do komunikacji UI z EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef _UI_ECP_R_SPKM_H
#define _UI_ECP_R_SPKM_H

#include "ui/src/ui.h"
// Konfigurator.
#include "base/lib/configurator.h"
#include "base/lib/mrmath/mrmath.h"
#include "base/ecp/ecp_robot.h"
#include "robot/spkm/ecp_r_spkm.h"

namespace mrrocpp {
namespace ui {
namespace spkm {

// ---------------------------------------------------------------
class EcpRobot
{

public:

	lib::single_thread_port <lib::epos::epos_cubic_command> * epos_cubic_command_data_port;
	lib::single_thread_port <lib::epos::epos_trapezoidal_command> * epos_trapezoidal_command_data_port;
	lib::single_thread_port <lib::epos::epos_operational_command> * epos_operational_command_data_port;
	lib::single_thread_port <bool> * epos_brake_command_data_port;

	lib::single_thread_request_port <lib::epos::epos_reply> * epos_reply_data_request_port;

	ecp::spkm::robot *the_robot;

	// by Y - do odczytu stanu poczatkowego robota
	void get_controller_state(lib::controller_state_t & robot_controller_initial_state_l);
	virtual void execute_motion(void);

	// ecp_buffer ui_edp_package; // by Y
	EcpRobot(lib::configurator &_config, lib::sr_ecp &_sr_ecp_msg); // Konstruktor

	virtual ~EcpRobot();

};

}
} //namespace ui
} //namespace mrrocpp

#endif

