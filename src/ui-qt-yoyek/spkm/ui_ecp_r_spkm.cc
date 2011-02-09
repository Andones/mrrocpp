// -------------------------------------------------------------------------
//                            ui_ecp->cc
// Metody sluzace do komunikacji UI z EDP - zlecenia dla driver'a
//
// Ostatnio modyfikowany: 2005
// -------------------------------------------------------------------------

/* Standard headers */
#include <iostream>

#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <cstring>
#include <cassert>
#include <fcntl.h>
#include <cerrno>
#include <cmath>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "../base/interface.h"

#include "base/lib/sr/srlib.h"

#include "ui_ecp_r_spkm.h"

namespace mrrocpp {
namespace ui {
namespace spkm {

// ---------------------------------------------------------------
EcpRobot::EcpRobot(common::Interface& _interface) :
	common::EcpRobotDataPort(_interface)
{

	the_robot = new ecp::spkm::robot(*(_interface.config), *(_interface.all_ecp_msg));

	epos_motor_command_data_port
			= the_robot->port_manager.get_port <lib::epos::epos_simple_command> (lib::epos::EPOS_MOTOR_COMMAND_DATA_PORT);

	epos_joint_command_data_port
			= the_robot->port_manager.get_port <lib::epos::epos_simple_command> (lib::epos::EPOS_JOINT_COMMAND_DATA_PORT);

	epos_external_command_data_port
			= the_robot->port_manager.get_port <lib::epos::epos_simple_command> (lib::epos::EPOS_EXTERNAL_COMMAND_DATA_PORT);

	/*
	 epos_cubic_command_data_port
	 = the_robot->port_manager.get_port <lib::epos::epos_cubic_command> (lib::epos::EPOS_CUBIC_COMMAND_DATA_PORT);

	 epos_trapezoidal_command_data_port
	 = the_robot->port_manager.get_port <lib::epos::epos_trapezoidal_command> (lib::epos::EPOS_TRAPEZOIDAL_COMMAND_DATA_PORT);

	 epos_operational_command_data_port
	 = the_robot->port_manager.get_port <lib::epos::epos_operational_command> (lib::epos::EPOS_OPERATIONAL_COMMAND_DATA_PORT);
	 */

	epos_brake_command_data_port = the_robot->port_manager.get_port <bool> (lib::epos::EPOS_BRAKE_COMMAND_DATA_PORT);

	epos_clear_fault_data_port = the_robot->port_manager.get_port <bool> (lib::epos::EPOS_CLEAR_FAULT_DATA_PORT);

	epos_reply_data_request_port
			= the_robot->port_manager.get_request_port <lib::epos::epos_reply> (lib::epos::EPOS_REPLY_DATA_REQUEST_PORT);

	epos_joint_reply_data_request_port
			= the_robot->port_manager.get_request_port <lib::epos::epos_reply> (lib::epos::EPOS_JOINT_REPLY_DATA_REQUEST_PORT);

	epos_external_reply_data_request_port
			= the_robot->port_manager.get_request_port <lib::epos::epos_reply> (lib::epos::EPOS_EXTERNAL_REPLY_DATA_REQUEST_PORT);

	assert(the_robot);

}

// ---------------------------------------------------------------
void EcpRobot::move_motors(const double final_position[])
{
	for (int i = 0; i < lib::spkm::NUM_OF_SERVOS; i++) {
		epos_motor_command_data_port->data.desired_position[i] = final_position[i];
	}
	//	std::cout << "UI final_position[4]" << final_position[4] << std::endl;
	epos_motor_command_data_port->set();
	execute_motion();

}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
void EcpRobot::move_joints(const double final_position[])
{
	for (int i = 0; i < lib::spkm::NUM_OF_SERVOS; i++) {
		epos_joint_command_data_port->data.desired_position[i] = final_position[i];
	}
	//	std::cout << "UI final_position[4]" << final_position[4] << std::endl;
	epos_joint_command_data_port->set();
	execute_motion();

}

void EcpRobot::move_external(const double final_position[])
{

	for (int i = 0; i < 6; i++) {
		epos_external_command_data_port->data.desired_position[i] = final_position[i];
	}

	//	epos_external_command_data_port->data.desired_position[i] = final_position[i];

	//	std::cout << "UI final_position[4]" << final_position[4] << std::endl;
	epos_external_command_data_port->set();
	execute_motion();

}

void EcpRobot::clear_fault()
{

	epos_clear_fault_data_port->data = true;

	epos_clear_fault_data_port->set();
	execute_motion();

}

void EcpRobot::stop_motors()
{

	epos_brake_command_data_port->data = true;

	epos_brake_command_data_port->set();
	execute_motion();

}

}
} //namespace ui
} //namespace mrrocpp
