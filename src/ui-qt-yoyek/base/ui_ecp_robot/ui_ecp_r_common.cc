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
#include "../interface.h"

#include "base/lib/sr/srlib.h"

#include "ui_ecp_r_common.h"

namespace mrrocpp {
namespace ui {
namespace common {

// ---------------------------------------------------------------
EcpRobot::EcpRobot(Interface& _interface, lib::robot_name_t _robot_name) :
	interface(_interface), ecp(NULL)
{

}
// ---------------------------------------------------------------

EcpRobot::~EcpRobot()
{
	delete ecp;
}

// ---------------------------------------------------------------
/* // by Y - zdefiniowane w irp6_on_track_robot - przemyslec czy nie trzeba wstawic warunku na poprawnosc synchronizacji
 void EcpRobot::synchronise ( void ) {
 // Zlecenie synchronizacji robota
 ecp->ecp_command.instruction_type = lib::SYNCHRO;
 ecp->EDP_buffer.send(EDP_fd);  // Wyslanie zlecenia synchronizacji
 ecp->EDP_buffer.query(EDP_fd); // Odebranie wyniku zlecenia
 if (ecp->reply_package.reply_type == lib::SYNCHRO_OK)
 synchronised = true;
 };// end: EcpRobot::synchronise ()
 */
// ---------------------------------------------------------------

// ---------------------------------------------------------------
// virtual  // by Y - wywalone


// ---------------------------------------------------------------
void EcpRobot::move_motors(const double final_position[])
{
	// Zlecenie wykonania makrokroku ruchu zadanego dla walow silnikow
	int nr_of_steps = 0, nr_tmp = 0; // Liczba krokow
	double temp = 0.0; // Zmienne pomocnicze

	/*
	 if (is_synchronised())
	 printf("zsynchronizowany move motors\n");
	 else
	 printf("niezsynchronizowany move motors\n");
	 */
	if (ecp->is_synchronised()) { // Robot zsynchronizowany
		// Odczyt aktualnego polozenia
		//   	printf("is synchronised przed read motors\n");
		read_motors(current_position);

		for (int j = 0; j < ecp->number_of_servos; j++) {
			temp = fabs(final_position[j] - current_position[j]);
			nr_tmp = (int) ceil(temp / MOTOR_STEP[j]);
			nr_of_steps = (nr_of_steps > nr_tmp) ? nr_of_steps : nr_tmp;
		}

		//  printf("is synchronised za read motors: nr of steps %d\n", nr_of_steps);
		// Parametry zlecenia ruchu i odczytu polozenia
		ecp->ecp_command.instruction_type = lib::SET_GET;
		ecp->ecp_command.motion_type = lib::ABSOLUTE;
		ecp->ecp_command.interpolation_type = lib::MIM;
	} else {
		// printf("!is_synchronised: %f \n",MOTOR_STEP);
		// Robot niezsynchroniozowany
		for (int j = 0; j < ecp->number_of_servos; j++) {
			temp = fabs(final_position[j]);
			nr_tmp = (int) ceil(temp / MOTOR_STEP[j]);
			nr_of_steps = (nr_of_steps > nr_tmp) ? nr_of_steps : nr_tmp;
		}

		ecp->ecp_command.instruction_type = lib::SET;
		ecp->ecp_command.motion_type = lib::RELATIVE;
		ecp->ecp_command.interpolation_type = lib::MIM;
	}
	ecp->ecp_command.get_type = ARM_DEFINITION; // ARM
	ecp->ecp_command.get_arm_type = lib::MOTOR;
	ecp->ecp_command.set_type = ARM_DEFINITION; // ARM
	ecp->ecp_command.set_arm_type = lib::MOTOR;
	ecp->ecp_command.motion_steps = nr_of_steps;
	ecp->ecp_command.value_in_step_no = nr_of_steps;

	if (nr_of_steps < 1) // Nie wykowywac bo zadano ruch do aktualnej pozycji
		return;
	for (int j = 0; j < ecp->number_of_servos; j++)
		ecp->ecp_command.arm.pf_def.arm_coordinates[j] = final_position[j];

	// printf("\n ilosc krokow: %d, po ilu komun: %d, odleglosc 1: %f\n",ecp_command.motion_steps, ecp_command.value_in_step_no, ecp_command.arm.pf_def.arm_coordinates[1]);

	execute_motion();

	if (ecp->is_synchronised())
		for (int j = 0; j < ecp->number_of_servos; j++) // Przepisanie aktualnych polozen
			current_position[j] = ecp->reply_package.arm.pf_def.arm_coordinates[j];
}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
void EcpRobot::move_joints(const double final_position[])
{
	// Zlecenie wykonania makrokroku ruchu zadanego dla wspolrzednych wewnetrznych
	int nr_of_steps = 0, nr_tmp = 0; // Liczba krokow
	double temp = 0.0; // Zmienne pomocnicze

	// Odczyt aktualnego polozenia
	read_joints(current_position);

	for (int j = 0; j < ecp->number_of_servos; j++) {
		temp = fabs(final_position[j] - current_position[j]);
		nr_tmp = (int) ceil(temp / JOINT_STEP[j]);
		nr_of_steps = (nr_of_steps > nr_tmp) ? nr_of_steps : nr_tmp;
	}

	// Parametry zlecenia ruchu i odczytu polozenia
	ecp->ecp_command.instruction_type = lib::SET_GET;
	ecp->ecp_command.get_type = ARM_DEFINITION; // ARM
	ecp->ecp_command.get_arm_type = lib::JOINT;
	ecp->ecp_command.set_type = ARM_DEFINITION; // ARM
	ecp->ecp_command.set_arm_type = lib::JOINT;
	ecp->ecp_command.motion_type = lib::ABSOLUTE;
	ecp->ecp_command.interpolation_type = lib::MIM;
	ecp->ecp_command.motion_steps = nr_of_steps;
	ecp->ecp_command.value_in_step_no = nr_of_steps;

	// cprintf("NOS=%u\n",ecp_command.motion_steps);

	if (nr_of_steps < 1) // Nie wykowywac bo zadano ruch do aktualnej pozycji
		return;

	for (int j = 0; j < ecp->number_of_servos; j++)
		ecp->ecp_command.arm.pf_def.arm_coordinates[j] = final_position[j];

	execute_motion();

	for (int j = 0; j < ecp->number_of_servos; j++) // Przepisanie aktualnych polozen
		current_position[j] = ecp->reply_package.arm.pf_def.arm_coordinates[j];
}
// ---------------------------------------------------------------


void EcpRobot::execute_motion(void)
{

	// Zlecenie wykonania ruchu przez robota jest to polecenie dla EDP


	interface.set_ui_state_notification(UI_N_COMMUNICATION);

	// TODO: in QNX/Photon exceptions are handled at the main loop
	// in GTK exceptions triggered signals cannot be handled in main loop

	ecp->execute_motion();
}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
void EcpRobot::set_desired_position(const double d_position[])
{
	// Przepisanie polozen zadanych do tablicy desired_position[]
	for (int j = 0; j < ecp->number_of_servos; j++)
		desired_position[j] = d_position[j];
}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
void EcpRobot::get_current_position(double c_position[])
{
	// Pobranie aktualnych polozen
	for (int j = 0; j < ecp->number_of_servos; j++)
		c_position[j] = current_position[j];
}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
// zlecenie odczytu numeru modelu kinematyki i korektora oraz numerow
// algorytmow serwo i numerow zestawow parametrow algorytmow

void EcpRobot::get_kinematic(uint8_t* kinematic_model_no)
{
	// Zlecenie odczytu numeru modelu i korektora kinematyki
	ecp->ecp_command.instruction_type = lib::GET;
	ecp->ecp_command.get_type = ROBOT_MODEL_DEFINITION; // ROBOT_MODEL
	ecp->ecp_command.get_robot_model_type = lib::ARM_KINEMATIC_MODEL; // ROBOT_MODEL
	execute_motion();

	*kinematic_model_no = ecp->reply_package.robot_model.kinematic_model.kinematic_model_no;
}

void EcpRobot::get_servo_algorithm(uint8_t algorithm_no[], uint8_t parameters_no[])
{

	// Zlecenie odczytu numerow algorytmow i zestawow parametrow
	ecp->ecp_command.instruction_type = lib::GET;
	ecp->ecp_command.get_type = ROBOT_MODEL_DEFINITION; // ROBOT_MODEL
	ecp->ecp_command.get_robot_model_type = lib::SERVO_ALGORITHM; //
	execute_motion();

	// Przepisanie aktualnych numerow algorytmow i zestawow parametrow
	memcpy(algorithm_no, ecp->reply_package.robot_model.servo_algorithm.servo_algorithm_no, ecp->number_of_servos
			* sizeof(uint8_t));
	memcpy(parameters_no, ecp->reply_package.robot_model.servo_algorithm.servo_parameters_no, ecp->number_of_servos
			* sizeof(uint8_t));
}

// do odczytu stanu poczatkowego robota
void EcpRobot::get_controller_state(lib::controller_state_t & robot_controller_initial_state_l)
{
	// Zlecenie odczytu numeru modelu i korektora kinematyki
	ecp->ecp_command.instruction_type = lib::GET;
	ecp->ecp_command.get_type = CONTROLLER_STATE_DEFINITION;

	execute_motion();

	robot_controller_initial_state_l = ecp->reply_package.controller_state;
	ecp->synchronised = robot_controller_initial_state_l.is_synchronised;
}

// ---------------------------------------------------------------
void EcpRobot::set_kinematic(uint8_t kinematic_model_no)
{
	// zlecenie zapisu numeru modelu kinematyki i korektora oraz numerow
	// algorytmow serwo i numerow zestawow parametrow algorytmow

	// Zlecenie zapisu numeru modelu i korektora kinematyki
	ecp->ecp_command.instruction_type = lib::SET;
	ecp->ecp_command.set_type = ROBOT_MODEL_DEFINITION; // ROBOT_MODEL
	ecp->ecp_command.robot_model.type = lib::ARM_KINEMATIC_MODEL; // ROBOT_MODEL
	ecp->ecp_command.get_robot_model_type = lib::ARM_KINEMATIC_MODEL; // ROBOT_MODEL

	ecp->ecp_command.robot_model.kinematic_model.kinematic_model_no = kinematic_model_no;

	execute_motion();
}
// ---------------------------------------------------------------


// ---------------------------------------------------------------
void EcpRobot::set_servo_algorithm(uint8_t algorithm_no[], uint8_t parameters_no[])
{

	// Zlecenie zapisu numerow algorytmow i zestawow parametrow
	// Przepisanie zadanych numerow algorytmow i zestawow parametrow
	memcpy(ecp->ecp_command.robot_model.servo_algorithm.servo_algorithm_no, algorithm_no, ecp->number_of_servos
			* sizeof(uint8_t));
	memcpy(ecp->ecp_command.robot_model.servo_algorithm.servo_parameters_no, parameters_no, ecp->number_of_servos
			* sizeof(uint8_t));
	ecp->ecp_command.instruction_type = lib::SET;
	ecp->ecp_command.set_type = ROBOT_MODEL_DEFINITION; // ROBOT_MODEL
	ecp->ecp_command.robot_model.type = lib::SERVO_ALGORITHM; //
	ecp->ecp_command.get_robot_model_type = lib::SERVO_ALGORITHM; //
	execute_motion();
}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
void EcpRobot::read_motors(double current_position[])
{
	// Zlecenie odczytu polozenia

	// printf("poczatek read motors\n");
	// Parametry zlecenia ruchu i odczytu polozenia
	ecp->ecp_command.get_type = ARM_DEFINITION;
	ecp->ecp_command.instruction_type = lib::GET;
	ecp->ecp_command.get_arm_type = lib::MOTOR;
	ecp->ecp_command.interpolation_type = lib::MIM;

	execute_motion();
	// printf("dalej za query read motors\n");
	for (int i = 0; i < ecp->number_of_servos; i++) // Przepisanie aktualnych polozen
		// { // printf("current position: %f\n",ecp->reply_package.arm.pf_def.arm_coordinates[i]);
		current_position[i] = ecp->reply_package.arm.pf_def.arm_coordinates[i];
	// 			    }
	// printf("koniec read motors\n");
}
// ---------------------------------------------------------------


// ---------------------------------------------------------------
void EcpRobot::read_joints(double current_position[])
{
	// Zlecenie odczytu polozenia

	// Parametry zlecenia ruchu i odczytu polozenia
	ecp->ecp_command.instruction_type = lib::GET;
	ecp->ecp_command.get_type = ARM_DEFINITION;
	ecp->ecp_command.get_arm_type = lib::JOINT;
	ecp->ecp_command.interpolation_type = lib::MIM;

	execute_motion();

	for (int i = 0; i < ecp->number_of_servos; i++) // Przepisanie aktualnych polozen
		current_position[i] = ecp->reply_package.arm.pf_def.arm_coordinates[i];
}
// ---------------------------------------------------------------

}
} //namespace ui
} //namespace mrrocpp

