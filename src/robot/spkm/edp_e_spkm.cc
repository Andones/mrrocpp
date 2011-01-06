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

#include <cstdio>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/mrmath/mrmath.h"

// Klasa edp_irp6ot_effector.
#include "robot/spkm/edp_e_spkm.h"
#include "base/edp/reader.h"
// Kinematyki.
#include "robot/spkm/kinematic_model_spkm.h"
#include "base/edp/manip_trans_t.h"
#include "base/edp/vis_server.h"
#include "robot/epos/epos_gen.h"

using namespace mrrocpp::lib::exception;

namespace mrrocpp {
namespace edp {
namespace spkm {

void effector::master_order(common::MT_ORDER nm_task, int nm_tryb)
{
	manip_effector::single_thread_master_order(nm_task, nm_tryb);
}

void effector::get_controller_state(lib::c_buffer &instruction)
{

	if (robot_test_mode) {
		// correct
		// controller_state_edp_buf.is_synchronised = true;
		// debug
		controller_state_edp_buf.is_synchronised = false;
	}
	//printf("get_controller_state: %d\n", controller_state_edp_buf.is_synchronised); fflush(stdout);
	reply.controller_state = controller_state_edp_buf;

	/*
	 // aktualizacja pozycji robota
	 // Uformowanie rozkazu odczytu dla SERVO_GROUP
	 sb->servo_command.instruction_code = lib::READ;
	 // Wyslanie rozkazu do SERVO_GROUP
	 // Pobranie z SERVO_GROUP aktualnej pozycji silnikow
	 //	printf("get_arm_position read_hardware\n");

	 sb->send_to_SERVO_GROUP();
	 */
	// dla pierwszego wypelnienia current_joints
	get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);

	{
		boost::mutex::scoped_lock lock(edp_irp6s_effector_mutex);

		// Ustawienie poprzedniej wartosci zadanej na obecnie odczytane polozenie walow silnikow
		for (int i = 0; i < number_of_servos; i++) {
			servo_current_motor_pos[i] = desired_motor_pos_new[i] = desired_motor_pos_old[i] = current_motor_pos[i];
			desired_joints[i] = current_joints[i];
		}
	}
}

// Konstruktor.
effector::effector(lib::configurator &_config) :
	manip_effector(_config, lib::spkm::ROBOT_NAME)
{
	number_of_servos = lib::spkm::NUM_OF_SERVOS;

	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();

	reset_variables();
}

/*--------------------------------------------------------------------------*/
void effector::move_arm(const lib::c_buffer &instruction)
{

	std::stringstream ss(std::stringstream::in | std::stringstream::out);

	switch (ecp_edp_cbuffer.variant)
	{
		/*
		 case lib::spkm::CBUFFER_EPOS_GEN_PARAMETERS: {
		 // epos parameters computation basing on trajectory parameters
		 lib::epos_gen_parameters epos_gen_parameters_structure;
		 lib::epos_low_level_command epos_low_level_command_structure;

		 memcpy(&epos_gen_parameters_structure, &(ecp_edp_cbuffer.epos_gen_parameters_structure), sizeof(epos_gen_parameters_structure));

		 compute_epos_command(epos_gen_parameters_structure, epos_low_level_command_structure);

		 ss << ecp_edp_cbuffer.epos_gen_parameters_structure.dm[4];

		 msg->message(ss.str().c_str());

		 // previously computed parameters send to epos2 controllers


		 // start the trajectory execution

		 }
		 break;*/
		case lib::spkm::CBUFFER_EPOS_MOTOR_COMMAND: {
			msg->message("move_arm CBUFFER_EPOS_MOTOR_COMMAND");
			lib::epos::epos_simple_command epos_simple_command_structure;
			epos_simple_command_structure = ecp_edp_cbuffer.epos_simple_command_structure;
			std::cout << "CBUFFER_EPOS_MOTOR_COMMAND: desired_position[4]: "
					<< epos_simple_command_structure.desired_position[4] << std::endl;
			if (robot_test_mode) {

				desired_motor_pos_new[4] = epos_simple_command_structure.desired_position[4];
			}
		}
			break;
		case lib::spkm::CBUFFER_EPOS_JOINT_COMMAND: {
			msg->message("move_arm CBUFFER_EPOS_JOINT_COMMAND");

			std::cout << "CBUFFER_EPOS_JOINT_COMMAND: desired_position[2]: "
					<< ecp_edp_cbuffer.epos_simple_command_structure.desired_position[2] << std::endl;

			lib::JointArray desired_joints_tmp(number_of_servos); // Wspolrzedne wewnetrzne -


			get_current_kinematic_model()->i2mp_transform(desired_motor_pos_new, desired_joints_tmp);
		}
			break;
		case lib::spkm::CBUFFER_EPOS_EXTERNAL_COMMAND: {
			msg->message("move_arm CBUFFER_EPOS_EXTERNAL_COMMAND");
			lib::Homog_matrix tmp_frame(ecp_edp_cbuffer.desired_frame);
			std::cout << tmp_frame;
		}
			break;

		case lib::spkm::CBUFFER_EPOS_CUBIC_COMMAND: {
			lib::epos::epos_cubic_command epos_cubic_command_structure;
			memcpy(&epos_cubic_command_structure, &(ecp_edp_cbuffer.epos_cubic_command_structure), sizeof(epos_cubic_command_structure));

		}
			break;
		case lib::spkm::CBUFFER_EPOS_TRAPEZOIDAL_COMMAND: {
			lib::epos::epos_trapezoidal_command epos_trapezoidal_command_structure;
			memcpy(&epos_trapezoidal_command_structure, &(ecp_edp_cbuffer.epos_trapezoidal_command_structure), sizeof(epos_trapezoidal_command_structure));

		}
			break;
		case lib::spkm::CBUFFER_EPOS_OPERATIONAL_COMMAND: {
			lib::epos::epos_operational_command epos_operational_command_structure;
			memcpy(&epos_operational_command_structure, &(ecp_edp_cbuffer.epos_operational_command_structure), sizeof(epos_operational_command_structure));

		}
			break;
		case lib::spkm::CBUFFER_EPOS_BRAKE_COMMAND: {

		}
			break;
		default:
			break;

	}

}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void effector::get_arm_position(bool read_hardware, lib::c_buffer &instruction)
{
	//lib::JointArray desired_joints_tmp(lib::MAX_SERVOS_NR); // Wspolrzedne wewnetrzne -
	//	printf(" GET ARM\n");
	//	flushall();
	if (robot_test_mode) {

		switch (instruction.get_arm_type)
		{
			case lib::MOTOR: {
				msg->message("EDP get_arm_position MOTOR");
				static int licznikaaa = (-11);

				std::stringstream ss(std::stringstream::in | std::stringstream::out);
				ss << "get_arm_position: " << licznikaaa;
				msg->message(ss.str().c_str());
				//	printf("%s\n", ss.str().c_str());


				edp_ecp_rbuffer.epos_controller[3].position = licznikaaa;
				edp_ecp_rbuffer.epos_controller[0].position = licznikaaa;
				edp_ecp_rbuffer.epos_controller[0].current = licznikaaa - 2;

				edp_ecp_rbuffer.epos_controller[4].position = desired_motor_pos_new[4];

				edp_ecp_rbuffer.epos_controller[5].position = licznikaaa + 5;
				edp_ecp_rbuffer.epos_controller[5].current = licznikaaa + 3;

				if (licznikaaa < 10) {
					for (int i = 0; i < number_of_servos; i++) {
						edp_ecp_rbuffer.epos_controller[i].motion_in_progress = true;
					}

				} else {
					for (int i = 0; i < number_of_servos; i++) {
						edp_ecp_rbuffer.epos_controller[i].motion_in_progress = false;
					}
				}
				licznikaaa++;
			}
				break;
			case lib::JOINT: {
				msg->message("EDP get_arm_position JOINT");
				static int licznik_joint = (-11);
				edp_ecp_rbuffer.epos_controller[2].position = licznik_joint;
				licznik_joint++;
			}
				break;
			case lib::FRAME: {
				msg->message("EDP get_arm_position FRAME");

				lib::Homog_matrix tmp_frame;

				tmp_frame.get_frame_tab(edp_ecp_rbuffer.current_frame);


			}
				break;
			default:
				break;

		}
	} else {

	}

	reply.servo_step = step_counter;
}
/*--------------------------------------------------------------------------*/

// Stworzenie modeli kinematyki dla robota IRp-6 na postumencie.
void effector::create_kinematic_models_for_given_robot(void)
{
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::spkm::kinematic_model_spkm());
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);
}

void effector::synchronise(void)
{
	if (robot_test_mode) {
		controller_state_edp_buf.is_synchronised = true;
	}

	std::cout << "EDP synchronisation" << std::endl;
}

/*--------------------------------------------------------------------------*/
void effector::create_threads()
{
	rb_obj = (boost::shared_ptr <common::reader_buffer>) new common::reader_buffer(*this);
	vis_obj = (boost::shared_ptr <common::vis_server>) new common::vis_server(*this);
}

void effector::instruction_deserialization()
{

	memcpy(&ecp_edp_cbuffer, instruction.arm.serialized_command, sizeof(ecp_edp_cbuffer));

	std::cerr << "EDP: " << ecp_edp_cbuffer << std::endl;
}

void effector::reply_serialization(void)
{
	memcpy(reply.arm.serialized_reply, &edp_ecp_rbuffer, sizeof(edp_ecp_rbuffer));
	assert(sizeof(reply.arm.serialized_reply) >= sizeof(edp_ecp_rbuffer));
}

}
// namespace spkm


namespace common {

// Stworzenie obiektu edp_irp6m_effector.
effector* return_created_efector(lib::configurator &_config)
{
	return new spkm::effector(_config);
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

