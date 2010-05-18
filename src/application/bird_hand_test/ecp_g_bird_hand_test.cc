/*
 * generator/ecp_g_bird_hand.cc
 *
 *Author: yoyek
 */

#include "ecp_g_bird_hand_test.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

//constructor with parameters: task and time to sleep [s]
bird_hand::bird_hand(common::task::task& _ecp_task) :
	generator(_ecp_task) {
	//	if (the_robot) the_robot->communicate_with_edp = false; //do not communicate with edp


	bird_hand_low_level_command_data_port = the_robot->port_manager.get_port<
			lib::bird_hand_low_level_command> (
			BIRD_HAND_LOW_LEVEL_COMMAND_DATA_PORT);
	bird_hand_reply_data_request_port
			= the_robot->port_manager.get_request_port<lib::bird_hand_reply> (
					BIRD_HAND_REPLY_DATA_REQUEST_PORT);

	bird_hand_gen_parameters_data_port = the_robot->port_manager.get_port<
			lib::bird_hand_gen_parameters> (BIRD_HAND_GEN_PARAMETERS_DATA_PORT);

}

void bird_hand::create_ecp_mp_reply() {

}

void bird_hand::get_mp_ecp_command() {
	memcpy(&mp_ecp_bird_hand_gen_parameters_structure,
			ecp_t.mp_command.ecp_next_state.mp_2_ecp_next_state_string,
			sizeof(mp_ecp_bird_hand_gen_parameters_structure));

	printf("aaaaa: %lf\n", mp_ecp_bird_hand_gen_parameters_structure.dm[4]);
}

bool bird_hand::first_step() {

	// parameters copying
	get_mp_ecp_command();

	ecp_t.sr_ecp_msg->message("bird_hand first_step");

	//bird_hand_data_port_command_structure.da[3] = 3.13;
	ecp_edp_bird_hand_gen_parameters_structure
			= mp_ecp_bird_hand_gen_parameters_structure;
	bird_hand_gen_parameters_data_port->set(
			ecp_edp_bird_hand_gen_parameters_structure);
	bird_hand_reply_data_request_port->set_request();

	return true;
}

bool bird_hand::next_step() {
	ecp_t.sr_ecp_msg->message("bird_hand next_step");

	if (bird_hand_reply_data_request_port->is_new_data()) {
		edp_ecp_bird_hand_reply_structure
				= bird_hand_reply_data_request_port->get();

		std::stringstream ss(std::stringstream::in | std::stringstream::out);
		ss << "licznik: "
				<< edp_ecp_bird_hand_reply_structure.bird_hand_controller[3].position;

		ecp_t.sr_ecp_msg->message(ss.str().c_str());

	}

	bool motion_in_progress = false;

	for (int i = 0; i < 6; i++) {
		if (edp_ecp_bird_hand_reply_structure.bird_hand_controller[i].motion_in_progress
				== true) {
			motion_in_progress = true;
			break;
		}
	}

	if (motion_in_progress) {
		bird_hand_reply_data_request_port->set_request();
		return true;
	} else {
		return false;
	}

}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

