/*!
 * @file
 * @brief File contains ecp robot class definition for SwarmItFix Parallel Kinematic Machine
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup spkm
 */

#include "base/lib/impconst.h"

#include "robot/spkm/ecp_r_spkm.h"
#include "base/lib/sr/sr_ecp.h"

namespace mrrocpp {
namespace ecp {
namespace spkm {

robot::robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
	robot::ecp_robot(lib::spkm::ROBOT_NAME, lib::spkm::NUM_OF_SERVOS, lib::spkm::EDP_SECTION, _config, _sr_ecp),
			kinematics_manager(), epos_motor_command_data_port(lib::epos::EPOS_MOTOR_COMMAND_DATA_PORT, port_manager),
			epos_joint_command_data_port(lib::epos::EPOS_JOINT_COMMAND_DATA_PORT, port_manager),
			epos_external_command_data_port(lib::epos::EPOS_EXTERNAL_COMMAND_DATA_PORT, port_manager),
			epos_cubic_command_data_port(lib::epos::EPOS_CUBIC_COMMAND_DATA_PORT, port_manager),
			epos_trapezoidal_command_data_port(lib::epos::EPOS_TRAPEZOIDAL_COMMAND_DATA_PORT, port_manager),
			epos_operational_command_data_port(lib::epos::EPOS_OPERATIONAL_COMMAND_DATA_PORT, port_manager),
			epos_brake_command_data_port(lib::epos::EPOS_BRAKE_COMMAND_DATA_PORT, port_manager),
			epos_reply_data_request_port(lib::epos::EPOS_REPLY_DATA_REQUEST_PORT, port_manager),
			epos_joint_reply_data_request_port(lib::epos::EPOS_JOINT_REPLY_DATA_REQUEST_PORT, port_manager),
			epos_external_reply_data_request_port(lib::epos::EPOS_EXTERNAL_REPLY_DATA_REQUEST_PORT, port_manager)

{
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();
}

robot::robot(common::task::task& _ecp_object) :
	robot::ecp_robot(lib::spkm::ROBOT_NAME, lib::spkm::NUM_OF_SERVOS, lib::spkm::EDP_SECTION, _ecp_object),
			kinematics_manager(), epos_motor_command_data_port(lib::epos::EPOS_MOTOR_COMMAND_DATA_PORT, port_manager),
			epos_joint_command_data_port(lib::epos::EPOS_JOINT_COMMAND_DATA_PORT, port_manager),
			epos_external_command_data_port(lib::epos::EPOS_EXTERNAL_COMMAND_DATA_PORT, port_manager),
			epos_cubic_command_data_port(lib::epos::EPOS_CUBIC_COMMAND_DATA_PORT, port_manager),
			epos_trapezoidal_command_data_port(lib::epos::EPOS_TRAPEZOIDAL_COMMAND_DATA_PORT, port_manager),
			epos_operational_command_data_port(lib::epos::EPOS_OPERATIONAL_COMMAND_DATA_PORT, port_manager),
			epos_brake_command_data_port(lib::epos::EPOS_BRAKE_COMMAND_DATA_PORT, port_manager),
			epos_reply_data_request_port(lib::epos::EPOS_REPLY_DATA_REQUEST_PORT, port_manager),
			epos_joint_reply_data_request_port(lib::epos::EPOS_JOINT_REPLY_DATA_REQUEST_PORT, port_manager),
			epos_external_reply_data_request_port(lib::epos::EPOS_EXTERNAL_REPLY_DATA_REQUEST_PORT, port_manager)

{
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();
}

void robot::check_then_set_command_flag(bool& flag)
{
	if (flag) {
		throw common::robot::ECP_error(lib::NON_FATAL_ERROR, INVALID_COMMAND_TO_EDP);
	} else {
		flag = true;
	}
}

void robot::create_command()
{
	//	int new_data_counter;
	bool is_new_data;
	bool is_new_request;

	sr_ecp_msg.message("create_command");

	is_new_data = false;
	is_new_request = false;

	// Set default variant to error in order to help tracking errors in communication
	// TODO: the following should be if-then-elseif-elseif-elseif...-else branch tree
	ecp_edp_cbuffer.variant = (lib::spkm::CBUFFER_VARIANT) -1;

	if (epos_motor_command_data_port.get() == mrrocpp::lib::NewData) {
		ecp_command.instruction.set_type = ARM_DEFINITION;
		if (!is_synchronised()) {
			ecp_command.instruction.motion_type = lib::RELATIVE;
			ecp_command.instruction.set_arm_type = lib::MOTOR;
		}

		ecp_edp_cbuffer.variant = lib::spkm::CBUFFER_EPOS_MOTOR_COMMAND;

		ecp_edp_cbuffer.epos_simple_command_structure = epos_motor_command_data_port.data;

		check_then_set_command_flag(is_new_data);
	}

	if (epos_joint_command_data_port.get() == mrrocpp::lib::NewData) {
		ecp_command.instruction.set_type = ARM_DEFINITION;

		ecp_edp_cbuffer.variant = lib::spkm::CBUFFER_EPOS_JOINT_COMMAND;

		ecp_edp_cbuffer.epos_simple_command_structure = epos_joint_command_data_port.data;

		check_then_set_command_flag(is_new_data);
	}

	if (epos_external_command_data_port.get() == mrrocpp::lib::NewData) {
		ecp_command.instruction.set_type = ARM_DEFINITION;

		ecp_edp_cbuffer.variant = lib::spkm::CBUFFER_EPOS_EXTERNAL_COMMAND;

		lib::Homog_matrix tmp_matrix(epos_external_command_data_port.data);
		tmp_matrix.get_frame_tab(ecp_edp_cbuffer.desired_frame);
		//ecp_edp_cbuffer.desired_frame = epos_external_command_data_port.data;

		check_then_set_command_flag(is_new_data);
	}

	if (epos_motor_command_data_port.get() == mrrocpp::lib::NewData) {
		ecp_command.instruction.set_type = ARM_DEFINITION;
		if (!is_synchronised()) {
			ecp_command.instruction.motion_type = lib::RELATIVE;
			ecp_command.instruction.set_arm_type = lib::MOTOR;
		}

		ecp_edp_cbuffer.variant = lib::spkm::CBUFFER_EPOS_MOTOR_COMMAND;

		ecp_edp_cbuffer.epos_simple_command_structure = epos_motor_command_data_port.data;

		check_then_set_command_flag(is_new_data);
	}

	if (epos_cubic_command_data_port.get() == mrrocpp::lib::NewData) {
		ecp_command.instruction.set_type = ARM_DEFINITION;

		ecp_edp_cbuffer.variant = lib::spkm::CBUFFER_EPOS_CUBIC_COMMAND;

		ecp_edp_cbuffer.epos_cubic_command_structure = epos_cubic_command_data_port.data;

		check_then_set_command_flag(is_new_data);
	}

	if (epos_trapezoidal_command_data_port.get() == mrrocpp::lib::NewData) {
		ecp_command.instruction.set_type = ARM_DEFINITION;

		ecp_edp_cbuffer.variant = lib::spkm::CBUFFER_EPOS_TRAPEZOIDAL_COMMAND;

		ecp_edp_cbuffer.epos_trapezoidal_command_structure = epos_trapezoidal_command_data_port.data;

		check_then_set_command_flag(is_new_data);
	}

	if (epos_operational_command_data_port.get() == mrrocpp::lib::NewData) {
		ecp_command.instruction.set_type = ARM_DEFINITION;

		ecp_edp_cbuffer.variant = lib::spkm::CBUFFER_EPOS_OPERATIONAL_COMMAND;

		ecp_edp_cbuffer.epos_operational_command_structure = epos_operational_command_data_port.data;

		check_then_set_command_flag(is_new_data);
	}
	if (epos_brake_command_data_port.get() == mrrocpp::lib::NewData) {
		ecp_command.instruction.set_type = ARM_DEFINITION;
		// generator command interpretation
		// narazie proste przepisanie

		ecp_edp_cbuffer.variant = lib::spkm::CBUFFER_EPOS_BRAKE_COMMAND;

		check_then_set_command_flag(is_new_data);
	}
	/*
	 if (epos_gen_parameters_data_port.get(epos_gen_parameters_structure) == mrrocpp::lib::NewData) {
	 ecp_command.instruction.set_type = ARM_DEFINITION;
	 // generator command interpretation
	 // narazie proste przepisanie

	 ecp_edp_cbuffer.variant = lib::spkm::CBUFFER_EPOS_GEN_PARAMETERS;

	 ecp_edp_cbuffer.epos_gen_parameters_structure = epos_gen_parameters_structure;

	 if (is_new_data) {
	 throw common::robot::ECP_error(lib::NON_FATAL_ERROR, INVALID_COMMAND_TO_EDP);
	 } else {
	 is_new_data = true;
	 }
	 }
	 */

	if (epos_reply_data_request_port.is_new_request()) {
		ecp_command.instruction.get_arm_type = lib::MOTOR;

		check_then_set_command_flag(is_new_request);
	}

	if (epos_joint_reply_data_request_port.is_new_request()) {
		ecp_command.instruction.get_arm_type = lib::JOINT;
		sr_ecp_msg.message("epos_joint_reply_data_request_port.is_new_request()");
		check_then_set_command_flag(is_new_request);
	}

	if (epos_external_reply_data_request_port.is_new_request()) {
		ecp_command.instruction.get_arm_type = lib::FRAME;
		sr_ecp_msg.message("epos_external_reply_data_request_port.is_new_request()");
		check_then_set_command_flag(is_new_request);
	}

	communicate_with_edp = true;
	if (is_new_data && is_new_request) {
		ecp_command.instruction.instruction_type = lib::SET_GET;
		sr_ecp_msg.message("is_new_data is_new_request");
	} else if (is_new_data) {
		ecp_command.instruction.instruction_type = lib::SET;
		sr_ecp_msg.message("is_new_data");
	} else if (is_new_request) {
		ecp_command.instruction.instruction_type = lib::GET;
		sr_ecp_msg.message("is_new_request");
	} else {
		communicate_with_edp = false;
	}

	if (is_new_request) {
		ecp_command.instruction.get_type = ARM_DEFINITION; // arm - ORYGINAL
	}

	// message serialization
	if (communicate_with_edp) {
		if ((ecp_command.instruction.instruction_type == lib::SET) || (ecp_command.instruction.instruction_type
				== lib::SET_GET)) {
			assert(sizeof(ecp_command.instruction.arm.serialized_command) >= sizeof(ecp_edp_cbuffer));
			memcpy(ecp_command.instruction.arm.serialized_command, &ecp_edp_cbuffer, sizeof(ecp_edp_cbuffer));
			std::cerr << "ECP: " << ecp_edp_cbuffer << std::endl;
		}
	}
}

void robot::get_reply()
{
	// message deserialization
	memcpy(&edp_ecp_rbuffer, reply_package.arm.serialized_reply, sizeof(edp_ecp_rbuffer));

	if (epos_reply_data_request_port.is_new_request()) {
		// generator reply generation
		for (int i = 0; i < lib::spkm::NUM_OF_SERVOS; i++) {
			epos_reply_data_request_port.data.epos_controller[i].position = edp_ecp_rbuffer.epos_controller[i].position;
			epos_reply_data_request_port.data.epos_controller[i].current = edp_ecp_rbuffer.epos_controller[i].current;
			epos_reply_data_request_port.data.epos_controller[i].motion_in_progress
					= edp_ecp_rbuffer.epos_controller[i].motion_in_progress;
		}
		epos_reply_data_request_port.data.contact = edp_ecp_rbuffer.contact;

		epos_reply_data_request_port.set();
	}

	if (epos_joint_reply_data_request_port.is_new_request()) {
		// generator reply generation
		sr_ecp_msg.message("ECP get_reply epos_joint_reply_data_request_port");

		for (int i = 0; i < lib::spkm::NUM_OF_SERVOS; i++) {
			epos_joint_reply_data_request_port.data.epos_controller[i].position
					= edp_ecp_rbuffer.epos_controller[i].position;
			epos_joint_reply_data_request_port.data.epos_controller[i].current
					= edp_ecp_rbuffer.epos_controller[i].current;
			epos_joint_reply_data_request_port.data.epos_controller[i].motion_in_progress
					= edp_ecp_rbuffer.epos_controller[i].motion_in_progress;
		}
		epos_joint_reply_data_request_port.data.contact = edp_ecp_rbuffer.contact;

		epos_joint_reply_data_request_port.set();
	}

	if (epos_external_reply_data_request_port.is_new_request()) {
		sr_ecp_msg.message("ECP get_reply epos_external_reply_data_request_port");
		// generator reply generation
		for (int i = 0; i < lib::spkm::NUM_OF_SERVOS; i++) {
			epos_external_reply_data_request_port.data.epos_controller[i].position
					= edp_ecp_rbuffer.epos_controller[i].position;
			epos_external_reply_data_request_port.data.epos_controller[i].current
					= edp_ecp_rbuffer.epos_controller[i].current;
			epos_external_reply_data_request_port.data.epos_controller[i].motion_in_progress
					= edp_ecp_rbuffer.epos_controller[i].motion_in_progress;
		}
		epos_external_reply_data_request_port.data.contact = edp_ecp_rbuffer.contact;
		lib::Homog_matrix tmp(edp_ecp_rbuffer.current_frame);
		tmp.get_frame_tab(epos_external_reply_data_request_port.data.current_frame);

		epos_external_reply_data_request_port.set();
	}

}

// Stworzenie modeli kinematyki dla robota IRp-6 na postumencie.
void robot::create_kinematic_models_for_given_robot(void)
{
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::spkm::kinematic_model_spkm());
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);
}

} // namespace spkm
} // namespace ecp
} // namespace mrrocpp

