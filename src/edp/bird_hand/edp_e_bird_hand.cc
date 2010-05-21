// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			edp_irp6m_effector.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Robot IRp-6 na postumencie
//				- definicja metod klasy edp_irp6m_effector
//				- definicja funkcji return_created_efector()
//
// Autor:
// Data:		14.02.2007
// ------------------------------------------------------------------------

#include <stdio.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/mis_fun.h"
#include "lib/mrmath/mrmath.h"

// Klasa edp_irp6ot_effector.
#include "edp/bird_hand/edp_e_bird_hand.h"
#include "edp/common/reader.h"
// Kinematyki.
#include "kinematics/bird_hand/kinematic_model_bird_hand.h"
#include "edp/common/manip_trans_t.h"
#include "edp/common/vis_server.h"

#define PORT "/dev/ser2"

using namespace mrrocpp::lib::exception;

namespace mrrocpp {
namespace edp {
namespace bird_hand {

void effector::master_order(common::MT_ORDER nm_task, int nm_tryb) {
	manip_effector::single_thread_master_order(nm_task, nm_tryb);
}

void effector::get_controller_state(lib::c_buffer &instruction) {

	for(uint8_t i=0; i<8; i++)
	{
		int16_t abspos;
		if(i<2)
			device.getSynchroPos(i, abspos);
		synchro_position[i] = (int32_t)(abspos * 124345.23443); // TODO : tu wspawi� magiczn� sta��
	}

	controller_state_edp_buf.is_synchronised = true;

	printf("synchro position readed : %d \n", synchro_position[0]);

	reply.controller_state = controller_state_edp_buf;

	Eigen::Matrix<int32_t, 8, 1> pos_tmp;

	for(uint8_t i=0; i<8; i++)
	{
		int32_t pos;
		int16_t t, c;
		uint8_t status;
		if(i<1)
			device.getStatus(i, status, pos, c, t);

		pos_tmp[i] = pos;
	}

	pos_tmp += synchro_position;


	printf("position readed : %d \n", pos_tmp[0]);

	fflush(stdout);

	current_motor_pos = pos_tmp.cast<double>();

	// dla pierwszego wypelnienia current_joints
	get_current_kinematic_model()->mp2i_transform(current_motor_pos,
			current_joints);

	{
		boost::mutex::scoped_lock lock(edp_irp6s_effector_mutex);

		// Ustawienie poprzedniej wartosci zadanej na obecnie odczytane polozenie walow silnikow
		for (int i = 0; i < number_of_servos; i++) {
			servo_current_motor_pos[i] = desired_motor_pos_new[i]
					= desired_motor_pos_old[i] = current_motor_pos[i];
			desired_joints[i] = current_joints[i];
		}
	}

	// inicjacja czasow

	struct timespec current_timespec;

	if (clock_gettime(CLOCK_MONOTONIC, &current_timespec) == -1) {
		perror("clock gettime");
	}

	macrostep_end_time = timespec2nsec(&current_timespec);

}

// Konstruktor.
effector::effector(lib::configurator &_config) :
	manip_effector(_config, lib::ROBOT_BIRD_HAND) {
	number_of_servos = BIRD_HAND_NUM_OF_SERVOS;

	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();

	reset_variables();

	device.connect(PORT);
}

/*--------------------------------------------------------------------------*/
void effector::move_arm(const lib::c_buffer &instruction) {

	struct timespec current_timespec;

	for(unsigned int i = 0; i<BIRD_HAND_NUM_OF_SERVOS; i++)
	{
		switch (ecp_edp_cbuffer.bird_hand_command_structure.profile_type[i])
		{
		case 0: //BIRD_HAND_SIGLE_STEP_POSTION_INCREMENT:
			device.setCMD1((uint16_t)i, (int16_t)ecp_edp_cbuffer.bird_hand_command_structure.motion_steps, (int16_t)ecp_edp_cbuffer.bird_hand_command_structure.reciprocal_of_damping[i], (int16_t)ecp_edp_cbuffer.bird_hand_command_structure.desired_torque[i], (int32_t)ecp_edp_cbuffer.bird_hand_command_structure.desired_position[i]);
			break;
		case 1: //BIRD_HAND_MACROSTEP_POSITION_INCREMENT:
			device.setCMD2((uint16_t)i, (int16_t)ecp_edp_cbuffer.bird_hand_command_structure.motion_steps, (int16_t)ecp_edp_cbuffer.bird_hand_command_structure.reciprocal_of_damping[i], (int16_t)ecp_edp_cbuffer.bird_hand_command_structure.desired_torque[i], (int32_t)ecp_edp_cbuffer.bird_hand_command_structure.desired_position[i]);
			break;
		case 2: //BIRD_HAND_MACROSTEP_ABSOLUTE_POSITION:
			device.setCMD3((uint16_t)i, (int16_t)ecp_edp_cbuffer.bird_hand_command_structure.motion_steps, (int16_t)ecp_edp_cbuffer.bird_hand_command_structure.reciprocal_of_damping[i], (int16_t)ecp_edp_cbuffer.bird_hand_command_structure.desired_torque[i], (int32_t)ecp_edp_cbuffer.bird_hand_command_structure.desired_position[i]);
			break;
		}
	}

	if (clock_gettime(CLOCK_MONOTONIC, &current_timespec) == -1) {
		perror("clock gettime");
	}

	_uint64 current_time = timespec2nsec(&current_timespec);

	if (current_time >= macrostep_end_time) {
		// stan bierny
		query_time = current_time
				+ ecp_edp_cbuffer.bird_hand_command_structure.ecp_query_step
						* BIRD_HAND_STEP_TIME_IN_NS;

		macrostep_end_time = current_time
				+ ecp_edp_cbuffer.bird_hand_command_structure.motion_steps
						* BIRD_HAND_STEP_TIME_IN_NS;

	} else {
		// stan czynny
		// UWAGA NA KOLEJNOSC OBLICZEN query_time i macrostep_end_time NIE ZAMIENIAC
		query_time = macrostep_end_time
				+ ecp_edp_cbuffer.bird_hand_command_structure.ecp_query_step
						* BIRD_HAND_STEP_TIME_IN_NS;

		macrostep_end_time
				+= ecp_edp_cbuffer.bird_hand_command_structure.motion_steps
						* BIRD_HAND_STEP_TIME_IN_NS;

	}

	device.synchronize(255,40);

}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void effector::get_arm_position(bool read_hardware, lib::c_buffer &instruction) {

	struct timespec query_timespec;

	if (test_mode) {
		for (int i = 0; i < BIRD_HAND_NUM_OF_SERVOS; i++) {
			edp_ecp_rbuffer.bird_hand_status_reply_structure.meassured_position[i]
					= ecp_edp_cbuffer.bird_hand_command_structure.desired_position[i];
			edp_ecp_rbuffer.bird_hand_status_reply_structure.meassured_torque[i]
							= ecp_edp_cbuffer.bird_hand_command_structure.desired_torque[i];
		}
	}

	nsec2timespec(&query_timespec, query_time);

	// zawieszenie do query_time

	clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &query_timespec, NULL);

	for(uint8_t i=0; i<8; i++)
	{
		int32_t pos;
		int16_t t, c;
		uint8_t status;

		if(i<2)
			device.getStatus(i, status, pos, c, t);

		edp_ecp_rbuffer.bird_hand_status_reply_structure.meassured_position[i] = (pos - synchro_position[i]);
		edp_ecp_rbuffer.bird_hand_status_reply_structure.meassured_current[i] = c;
		edp_ecp_rbuffer.bird_hand_status_reply_structure.meassured_torque[i] = t;
	}

	reply.servo_step = step_counter;

}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void effector::set_robot_model(const lib::c_buffer &instruction) {

}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void effector::get_robot_model(lib::c_buffer &instruction) {

	edp_ecp_rbuffer.bird_hand_configuration_reply_structure.d_factor[2] = 121;

}
/*--------------------------------------------------------------------------*/

// Stworzenie modeli kinematyki dla robota IRp-6 na postumencie.
void effector::create_kinematic_models_for_given_robot(void) {
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::bird_hand::kinematic_model_bird_hand());
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);
}

/*--------------------------------------------------------------------------*/
void effector::create_threads() {
	rb_obj = new common::reader_buffer(*this);
	vis_obj = new common::vis_server(*this);
}

void effector::instruction_deserialization() {

	memcpy(&ecp_edp_cbuffer, instruction.arm.serialized_command,
			sizeof(ecp_edp_cbuffer));

}

void effector::reply_serialization(void) {
	memcpy(reply.arm.serialized_reply, &edp_ecp_rbuffer,
			sizeof(edp_ecp_rbuffer));

}

}
// namespace bird_hand


namespace common {

// Stworzenie obiektu edp_bird_hand_effector.
effector* return_created_efector(lib::configurator &_config) {
	return new bird_hand::effector(_config);
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

