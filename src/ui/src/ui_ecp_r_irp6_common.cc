// -------------------------------------------------------------------------
//                            ui_ecp->cc
// Metody sluzace do komunikacji UI z EDP - zlecenia dla driver'a
//
// Ostatnio modyfikowany: 2005
// -------------------------------------------------------------------------

/* Standard headers */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>

#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <iostream>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/irp6_postument/ecp_r_irp6p.h"
#include "ecp/irp6_mechatronika/ecp_r_irp6m.h"
#include "ecp/polycrank/ecp_r_polycrank.h"
#include "ecp/smb/ecp_r_smb.h"
#include "ecp/spkm/ecp_r_spkm.h"
#include "ui/ui_ecp_r_irp6_common.h"

#include <math.h>
#include "lib/mrmath/mrmath.h"

// ---------------------------------------------------------------
ui_common_robot::ui_common_robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp_msg, lib::robot_name_t _robot_name)
{

	switch (_robot_name)
	{
		case lib::ROBOT_IRP6_ON_TRACK:
			ecp = new ecp::irp6ot::robot(_config, _sr_ecp_msg);
			break;
		case lib::ROBOT_IRP6_POSTUMENT:
			ecp = new ecp::irp6p::robot(_config, _sr_ecp_msg);
			break;
		case lib::ROBOT_IRP6_MECHATRONIKA:
			ecp = new ecp::irp6m::robot(_config, _sr_ecp_msg);
			break;
		case lib::ROBOT_POLYCRANK:
			ecp = new ecp::polycrank::robot(_config, _sr_ecp_msg);
			break;
		case lib::ROBOT_SMB:
			ecp = new ecp::smb::robot(_config, _sr_ecp_msg);
			break;
		case lib::ROBOT_SPKM:
			ecp = new ecp::spkm::robot(_config, _sr_ecp_msg);
			break;
			//    case lib::ROBOT_SPEAKER:
			//    case lib::ROBOT_CONVEYOR:
			//        break;
		default:
			fprintf(stderr, "ERROR: unknown robot name in ecp_robot ui_common_robot::ui_common_robot\n");
			ecp = NULL;
			break;
	}

	assert(ecp);

	// Konstruktor klasy
	ecp->ecp_command.instruction.rmodel.kinematic_model.kinematic_model_no = 0;
	ecp->ecp_command.instruction.get_type = ARM_DV; // ARM
	ecp->ecp_command.instruction.get_arm_type = lib::MOTOR;
	ecp->ecp_command.instruction.set_type = ARM_DV; // ARM
	ecp->ecp_command.instruction.set_arm_type = lib::MOTOR;
	ecp->ecp_command.instruction.motion_steps = 0;
	ecp->ecp_command.instruction.value_in_step_no = 0;

	ecp->synchronised = false;

	MOTOR_STEP = 0.1; // Przyrost kata obrotu walu silnika [rad]
	MOTOR_GRIPPER_STEP = 0.5;
	JOINT_ANGULAR_STEP = 0.0004; // Przyrost kata obrotu w przegubie obrotowym [rad]
	JOINT_LINEAR_STEP = 0.00004; // Przyrost liniowy w przegubach posuwistych [m]
	JOINT_GRIPPER_STEP = 0.000004; // Przyrost liniowy w chwytaku [m]
	END_EFFECTOR_LINEAR_STEP = 0.00002;// Przyrost wspolrzednej polozenia koncowki [m]
	END_EFFECTOR_ANGULAR_STEP = 0.0002; // Przyrost wspolrzednej orientacji koncowki [rad]
	END_EFFECTOR_GRIPPER_STEP = 0.000005; // Przyrost wspolrzednej orientacji koncowki [rad]

}
// ---------------------------------------------------------------

ui_common_robot::~ui_common_robot()
{
	delete ecp;
}

// ---------------------------------------------------------------
/* // by Y - zdefiniowane w irp6_on_track_robot - przemyslec czy nie trzeba wstawic warunku na poprawnosc synchronizacji
 void ui_common_robot::synchronise ( void ) {
 // Zlecenie synchronizacji robota
 ecp->ecp_command.instruction.instruction_type = lib::SYNCHRO;
 ecp->EDP_buffer.send(EDP_fd);  // Wyslanie zlecenia synchronizacji
 ecp->EDP_buffer.query(EDP_fd); // Odebranie wyniku zlecenia
 if (ecp->reply_package.reply_type == lib::SYNCHRO_OK)
 synchronised = true;
 };// end: ui_common_robot::synchronise ()
 */
// ---------------------------------------------------------------

// ---------------------------------------------------------------
// virtual  // by Y - wywalone

void ui_common_robot::execute_motion(void)
{

	// Zlecenie wykonania ruchu przez robota jest to polecenie dla EDP


	set_ui_state_notification(UI_N_COMMUNICATION);

	// TODO: in QNX/Photon exceptions are handled at the main loop
	// in GTK exceptions triggered signals cannot be handled in main loop

	ecp->execute_motion();
}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
void ui_common_robot::set_desired_position(double d_position[])
{
	// Przepisanie polozen zadanych do tablicy desired_position[]
	for (int j = 0; j < ecp->number_of_servos; j++)
		desired_position[j] = d_position[j];
}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
void ui_common_robot::get_current_position(double c_position[])
{
	// Pobranie aktualnych polozen
	for (int j = 0; j < ecp->number_of_servos; j++)
		c_position[j] = current_position[j];
}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
// zlecenie odczytu numeru modelu kinematyki i korektora oraz numerow
// algorytmow serwo i numerow zestawow parametrow algorytmow

void ui_common_robot::get_kinematic(uint8_t* kinematic_model_no)
{
	// Zlecenie odczytu numeru modelu i korektora kinematyki
	ecp->ecp_command.instruction.instruction_type = lib::GET;
	ecp->ecp_command.instruction.get_type = RMODEL_DV; // RMODEL
	ecp->ecp_command.instruction.get_rmodel_type = lib::ARM_KINEMATIC_MODEL; // RMODEL
	execute_motion();

	*kinematic_model_no = ecp->reply_package.rmodel.kinematic_model.kinematic_model_no;
}

void ui_common_robot::get_servo_algorithm(uint8_t algorithm_no[], uint8_t parameters_no[])
{

	// Zlecenie odczytu numerow algorytmow i zestawow parametrow
	ecp->ecp_command.instruction.instruction_type = lib::GET;
	ecp->ecp_command.instruction.get_type = RMODEL_DV; // RMODEL
	ecp->ecp_command.instruction.get_rmodel_type = lib::SERVO_ALGORITHM; //
	execute_motion();

	// Przepisanie aktualnych numerow algorytmow i zestawow parametrow
	memcpy(algorithm_no, ecp->reply_package.rmodel.servo_algorithm.servo_algorithm_no, ecp->number_of_servos
			* sizeof(uint8_t));
	memcpy(parameters_no, ecp->reply_package.rmodel.servo_algorithm.servo_parameters_no, ecp->number_of_servos
			* sizeof(uint8_t));
}

// do odczytu stanu poczatkowego robota
void ui_common_robot::get_controller_state(lib::controller_state_t & robot_controller_initial_state_l)
{
	// Zlecenie odczytu numeru modelu i korektora kinematyki
	ecp->ecp_command.instruction.instruction_type = lib::GET;
	ecp->ecp_command.instruction.get_type = CONTROLLER_STATE_DV;

	execute_motion();

	robot_controller_initial_state_l = ecp->reply_package.controller_state;
	ecp->synchronised = robot_controller_initial_state_l.is_synchronised;
}

// ---------------------------------------------------------------
void ui_common_robot::set_kinematic(uint8_t kinematic_model_no)
{
	// zlecenie zapisu numeru modelu kinematyki i korektora oraz numerow
	// algorytmow serwo i numerow zestawow parametrow algorytmow

	// Zlecenie zapisu numeru modelu i korektora kinematyki
	ecp->ecp_command.instruction.instruction_type = lib::SET;
	ecp->ecp_command.instruction.set_type = RMODEL_DV; // RMODEL
	ecp->ecp_command.instruction.set_rmodel_type = lib::ARM_KINEMATIC_MODEL; // RMODEL
	ecp->ecp_command.instruction.get_rmodel_type = lib::ARM_KINEMATIC_MODEL; // RMODEL

	ecp->ecp_command.instruction.rmodel.kinematic_model.kinematic_model_no = kinematic_model_no;

	execute_motion();
}
// ---------------------------------------------------------------


// ---------------------------------------------------------------
void ui_common_robot::set_servo_algorithm(uint8_t algorithm_no[], uint8_t parameters_no[])
{

	// Zlecenie zapisu numerow algorytmow i zestawow parametrow
	// Przepisanie zadanych numerow algorytmow i zestawow parametrow
	memcpy(ecp->ecp_command.instruction.rmodel.servo_algorithm.servo_algorithm_no, algorithm_no, ecp->number_of_servos
			* sizeof(uint8_t));
	memcpy(ecp->ecp_command.instruction.rmodel.servo_algorithm.servo_parameters_no, parameters_no, ecp->number_of_servos
			* sizeof(uint8_t));
	ecp->ecp_command.instruction.instruction_type = lib::SET;
	ecp->ecp_command.instruction.set_type = RMODEL_DV; // RMODEL
	ecp->ecp_command.instruction.set_rmodel_type = lib::SERVO_ALGORITHM; //
	ecp->ecp_command.instruction.get_rmodel_type = lib::SERVO_ALGORITHM; //
	execute_motion();
}
// ---------------------------------------------------------------


// ZADANIE NARZEDZIA
// ---------------------------------------------------------------
void ui_common_robot::set_tool_xyz_angle_axis(double tool_vector[6])
{

	ecp->ecp_command.instruction.instruction_type = lib::SET;
	ecp->ecp_command.instruction.set_type = RMODEL_DV; // RMODEL
	ecp->ecp_command.instruction.set_rmodel_type = lib::TOOL_FRAME;
	ecp->ecp_command.instruction.get_rmodel_type = lib::TOOL_FRAME;

	lib::Homog_matrix tmp;
	tmp.set_from_xyz_angle_axis(tool_vector);
	tmp.get_frame_tab(ecp->ecp_command.instruction.rmodel.tool_frame_def.tool_frame);

	execute_motion();
}
// ---------------------------------------------------------------


// ZADANIE NARZEDZIA
// ---------------------------------------------------------------
void ui_common_robot::set_tool_xyz_euler_zyz(double tool_vector[6])
{

	ecp->ecp_command.instruction.instruction_type = lib::SET;
	ecp->ecp_command.instruction.set_type = RMODEL_DV; // RMODEL
	ecp->ecp_command.instruction.set_rmodel_type = lib::TOOL_FRAME;
	ecp->ecp_command.instruction.get_rmodel_type = lib::TOOL_FRAME;

	lib::Homog_matrix tmp;
	tmp.set_from_xyz_euler_zyz(tool_vector);
	tmp.get_frame_tab(ecp->ecp_command.instruction.rmodel.tool_frame_def.tool_frame);

	execute_motion();
}
// ---------------------------------------------------------------


// ODCZYT NARZEDZIA
// ---------------------------------------------------------------
void ui_common_robot::read_tool_xyz_angle_axis(double tool_vector[6])
{

	// Zlecenie odczytu numeru modelu i korektora kinematyki
	ecp->ecp_command.instruction.instruction_type = lib::GET;
	ecp->ecp_command.instruction.get_type = RMODEL_DV; // RMODEL
	ecp->ecp_command.instruction.set_rmodel_type = lib::TOOL_FRAME;
	ecp->ecp_command.instruction.get_rmodel_type = lib::TOOL_FRAME;

	execute_motion();

	lib::Homog_matrix tmp(ecp->reply_package.rmodel.tool_frame_def.tool_frame);
	tmp.get_xyz_angle_axis(tool_vector);

}
// ---------------------------------------------------------------


// ODCZYT NARZEDZIA
// ---------------------------------------------------------------
void ui_common_robot::read_tool_xyz_euler_zyz(double tool_vector[6])
{

	// Zlecenie odczytu numeru modelu i korektora kinematyki
	ecp->ecp_command.instruction.instruction_type = lib::GET;
	ecp->ecp_command.instruction.get_type = RMODEL_DV; // RMODEL
	ecp->ecp_command.instruction.set_rmodel_type = lib::TOOL_FRAME;
	ecp->ecp_command.instruction.get_rmodel_type = lib::TOOL_FRAME;

	execute_motion();
	lib::Homog_matrix tmp(ecp->reply_package.rmodel.tool_frame_def.tool_frame);
	tmp.get_xyz_euler_zyz(tool_vector);

}
// ---------------------------------------------------------------


// ---------------------------------------------------------------
void ui_common_robot::move_motors(double final_position[])
{
	// Zlecenie wykonania makrokroku ruchu zadanego dla walow silnikow
	int nr_of_steps, nr_ang, nr_grip; // Liczba krokow
	double max_inc = 0.0, max_inc_grip = 0.0, temp = 0.0; // Zmienne pomocnicze

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
			if (j == (ecp->number_of_servos - 1)) // gripper
				max_inc_grip = (max_inc_grip > temp) ? max_inc_grip : temp;
			else
				max_inc = (max_inc > temp) ? max_inc : temp;
		}
		nr_ang = (int) ceil(max_inc / MOTOR_STEP);
		nr_grip = (int) ceil(max_inc_grip / MOTOR_GRIPPER_STEP);
		nr_of_steps = (nr_ang > nr_grip) ? nr_ang : nr_grip;
		//  printf("is synchronised za read motors: nr of steps %d\n", nr_of_steps);
		// Parametry zlecenia ruchu i odczytu polozenia
		ecp->ecp_command.instruction.instruction_type = lib::SET_GET;
		ecp->ecp_command.instruction.motion_type = lib::ABSOLUTE;
		ecp->ecp_command.instruction.interpolation_type = lib::MIM;
	} else {
		// printf("!is_synchronised: %f \n",MOTOR_STEP);
		// Robot niezsynchroniozowany
		for (int j = 0; j < ecp->number_of_servos; j++) {
			temp = fabs(final_position[j]);
			if (j == (ecp->number_of_servos - 1)) // gripper
				max_inc_grip = (max_inc_grip > temp) ? max_inc_grip : temp;
			else
				max_inc = (max_inc > temp) ? max_inc : temp;
		}
		nr_ang = (int) ceil(max_inc / MOTOR_STEP);
		nr_grip = (int) ceil(max_inc_grip / MOTOR_GRIPPER_STEP);
		nr_of_steps = (nr_ang > nr_grip) ? nr_ang : nr_grip;

		ecp->ecp_command.instruction.instruction_type = lib::SET;
		ecp->ecp_command.instruction.motion_type = lib::RELATIVE;
		ecp->ecp_command.instruction.interpolation_type = lib::MIM;
	}
	ecp->ecp_command.instruction.get_type = ARM_DV; // ARM
	ecp->ecp_command.instruction.get_arm_type = lib::MOTOR;
	ecp->ecp_command.instruction.set_type = ARM_DV; // ARM
	ecp->ecp_command.instruction.set_arm_type = lib::MOTOR;
	ecp->ecp_command.instruction.motion_steps = nr_of_steps;
	ecp->ecp_command.instruction.value_in_step_no = nr_of_steps;

	if (nr_of_steps < 1) // Nie wykowywac bo zadano ruch do aktualnej pozycji
		return;

	for (int j = 0; j < ecp->number_of_servos; j++) {
		ecp->ecp_command.instruction.arm.pf_def.arm_coordinates[j] = final_position[j];
		/*
		 printf("ui_ecp_aa: %f, %f, %f, %f, %f, %f, %f, %d\n", final_position[0], final_position[1], final_position[2], final_position[3],
		 final_position[4], final_position[5], final_position[6], ecp->ecp_command.instruction.motion_steps);
		 */
		// printf("\n ilosc krokow: %d, po ilu komun: %d, odleglosc 1: %f\n",ecp->ecp_command.instruction.motion_steps, ecp->ecp_command.instruction.value_in_step_no, ecp->ecp_command.instruction.arm.pf_def.arm_coordinates[1]);
	}

	execute_motion();

	if (ecp->is_synchronised())
		for (int j = 0; j < ecp->number_of_servos; j++) // Przepisanie aktualnych polozen
			current_position[j] = ecp->reply_package.arm.pf_def.arm_coordinates[j];
}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
void ui_common_robot::move_joints(double final_position[])
{
	// Zlecenie wykonania makrokroku ruchu zadanego dla wspolrzednych wewnetrznych
	int nr_of_steps; // Liczba krokow
	int nr_ang, nr_grip, nr_lin;
	double max_inc_ang = 0.0, max_inc_lin = 0.0, max_inc_grip = 0.0, temp = 0.0; // Zmienne pomocnicze

	max_inc_ang = max_inc_lin = 0.0;

	// Odczyt aktualnego polozenia
	read_joints(current_position);

	for (int j = 0; j < ecp->number_of_servos; j++) {
		temp = fabs(final_position[j] - current_position[j]);
		if (ecp->robot_name == lib::ROBOT_IRP6_ON_TRACK && j == 0) // tor
			max_inc_lin = (max_inc_lin > temp) ? max_inc_lin : temp;
		else if (j == ecp->number_of_servos - 1) // gripper
			max_inc_grip = (max_inc_grip > temp) ? max_inc_grip : temp;
		else
			max_inc_ang = (max_inc_ang > temp) ? max_inc_ang : temp;
	}

	nr_ang = (int) ceil(max_inc_ang / JOINT_ANGULAR_STEP);
	nr_lin = (int) ceil(max_inc_lin / JOINT_LINEAR_STEP);
	nr_grip = (int) ceil(max_inc_grip / JOINT_GRIPPER_STEP);
	nr_of_steps = (nr_ang > nr_lin) ? nr_ang : nr_lin;
	if (ecp->robot_name == lib::ROBOT_IRP6_ON_TRACK) {
		nr_of_steps = (nr_ang > nr_lin) ? nr_ang : nr_lin;
	}
	nr_of_steps = (nr_of_steps > nr_grip) ? nr_of_steps : nr_grip;
	// printf("nr_of_steps: %d, nr_grip: %d\n",nr_of_steps,nr_grip);
	// Parametry zlecenia ruchu i odczytu polozenia
	ecp->ecp_command.instruction.instruction_type = lib::SET_GET;
	ecp->ecp_command.instruction.get_type = ARM_DV; // ARM
	ecp->ecp_command.instruction.get_arm_type = lib::JOINT;
	ecp->ecp_command.instruction.set_type = ARM_DV; // ARM
	ecp->ecp_command.instruction.set_arm_type = lib::JOINT;
	ecp->ecp_command.instruction.motion_type = lib::ABSOLUTE;
	ecp->ecp_command.instruction.interpolation_type = lib::MIM;
	ecp->ecp_command.instruction.motion_steps = nr_of_steps;
	ecp->ecp_command.instruction.value_in_step_no = nr_of_steps;

	if (nr_of_steps < 1) // Nie wykowywac bo zadano ruch do aktualnej pozycji
		return;

	for (int j = 0; j < ecp->number_of_servos; j++) {
		ecp->ecp_command.instruction.arm.pf_def.arm_coordinates[j] = final_position[j];
		ecp->ecp_command.instruction.arm.pf_def.desired_torque[j] = final_position[j];
	}

	execute_motion();

	for (int j = 0; j < ecp->number_of_servos; j++) // Przepisanie aktualnych polozen
		current_position[j] = ecp->reply_package.arm.pf_def.arm_coordinates[j];
}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
void ui_common_robot::move_xyz_euler_zyz(double final_position[7])
{
	// Zlecenie wykonania makrokroku ruchu zadanego we wspolrzednych
	// zewnetrznych: xyz i katy Euler'a Z-Y-Z

	int nr_of_steps; // Liczba krokow
	int nr_ang, nr_lin, nr_grip;
	double max_inc_ang = 0.0, max_inc_lin = 0.0, max_inc_grip = 0.0, temp_lin, temp_ang, temp_grip; // Zmienne pomocnicze

	max_inc_ang = max_inc_lin = 0.0;
	// Odczyt aktualnego polozenia we wsp. zewn. xyz i katy Euler'a Z-Y-Z
	read_xyz_euler_zyz(current_position);

	for (int j = 0; j < 3; j++) {
		temp_lin = fabs(final_position[j] - current_position[j]);
		temp_ang = fabs(final_position[j + 3] - current_position[j + 3]);
		max_inc_ang = (max_inc_ang > temp_ang) ? max_inc_ang : temp_ang;
		max_inc_lin = (max_inc_lin > temp_lin) ? max_inc_lin : temp_lin;
	}

	temp_grip = fabs(final_position[6] - current_position[6]);
	max_inc_grip = temp_grip;

	nr_ang = (int) ceil(max_inc_ang / END_EFFECTOR_ANGULAR_STEP);
	nr_lin = (int) ceil(max_inc_lin / END_EFFECTOR_LINEAR_STEP);
	nr_grip = (int) ceil(max_inc_grip / END_EFFECTOR_GRIPPER_STEP);

	nr_of_steps = (nr_ang > nr_lin) ? nr_ang : nr_lin;
	nr_of_steps = (nr_of_steps > nr_grip) ? nr_of_steps : nr_grip;

	// Parametry zlecenia ruchu i odczytu polozenia
	ecp->ecp_command.instruction.instruction_type = lib::SET_GET;
	ecp->ecp_command.instruction.get_arm_type = lib::FRAME;
	ecp->ecp_command.instruction.set_type = ARM_DV; // ARM
	ecp->ecp_command.instruction.set_arm_type = lib::FRAME;
	ecp->ecp_command.instruction.motion_type = lib::ABSOLUTE;
	ecp->ecp_command.instruction.interpolation_type = lib::MIM;
	ecp->ecp_command.instruction.motion_steps = nr_of_steps;
	ecp->ecp_command.instruction.value_in_step_no = nr_of_steps;

	// cprintf("eNOS=%u\n",ecp->ecp_command.instruction.motion_steps);
	if (nr_of_steps < 1) // Nie wykowywac bo zadano ruch do aktualnej pozycji
		return;

	lib::Homog_matrix tmp;
	tmp.set_from_xyz_euler_zyz(final_position);
	tmp.get_frame_tab(ecp->ecp_command.instruction.arm.pf_def.arm_frame);

	ecp->ecp_command.instruction.arm.pf_def.gripper_coordinate = final_position[6];

	execute_motion();

	for (int j = 0; j < 6; j++) { // Przepisanie aktualnych polozen
		current_position[j] = ecp->reply_package.arm.pf_def.arm_coordinates[j];
	}
	current_position[6] = ecp->reply_package.arm.pf_def.gripper_coordinate;
}
// ---------------------------------------------------------------


void ui_common_robot::move_xyz_angle_axis(double final_position[7])
{
	double aa_eul[6]; // tablica przechowujaca polecenie przetransformowane
	// do formy XYZ_EULER_ZYZ
	double x, y, z, alfa, kx, ky, kz;

	x = final_position[0];
	y = final_position[1];
	z = final_position[2];

	alfa = sqrt(final_position[3] * final_position[3] + final_position[4] * final_position[4] + final_position[5]
			* final_position[5]);

	kx = final_position[3] / alfa;
	ky = final_position[4] / alfa;
	kz = final_position[5] / alfa;

	lib::Homog_matrix A(kx, ky, kz, alfa, x, y, z);
	A.get_xyz_euler_zyz(aa_eul); // zadane polecenie w formie XYZ_EULER_ZYZ

	// Odczyt aktualnego polozenia we wsp. zewn. xyz i katy Euler'a Z-Y-Z
	read_xyz_euler_zyz(current_position);

	// Wyznaczenie liczby krokow

	int nr_of_steps; // Liczba krokow
	int nr_ang, nr_lin, nr_grip;
	double max_inc_ang = 0.0, max_inc_lin = 0.0, max_inc_grip = 0.0, temp_lin, temp_ang, temp_grip; // Zmienne pomocnicze

	max_inc_ang = max_inc_lin = 0.0;
	for (int i = 0; i < 3; i++) {
		temp_lin = fabs(aa_eul[i] - current_position[i]);
		temp_ang = fabs(aa_eul[i + 3] - current_position[i + 3]);
		if (temp_ang > max_inc_ang)
			max_inc_ang = temp_ang;
		if (temp_lin > max_inc_lin)
			max_inc_lin = temp_lin;
	}

	temp_grip = fabs(final_position[6] - current_position[6]);
	max_inc_grip = temp_grip;

	nr_ang = (int) ceil(max_inc_ang / END_EFFECTOR_ANGULAR_STEP);
	nr_lin = (int) ceil(max_inc_lin / END_EFFECTOR_LINEAR_STEP);
	nr_grip = (int) ceil(max_inc_grip / END_EFFECTOR_GRIPPER_STEP);

	nr_of_steps = (nr_ang > nr_lin) ? nr_ang : nr_lin;
	nr_of_steps = (nr_of_steps > nr_grip) ? nr_of_steps : nr_grip;

	// Zadano ruch do aktualnej pozycji
	if (nr_of_steps < 1)
		return;

	ecp->ecp_command.instruction.instruction_type = lib::SET_GET;
	ecp->ecp_command.instruction.get_arm_type = lib::FRAME;
	ecp->ecp_command.instruction.set_type = ARM_DV; // ARM
	ecp->ecp_command.instruction.set_arm_type = lib::FRAME;
	ecp->ecp_command.instruction.motion_type = lib::ABSOLUTE;
	ecp->ecp_command.instruction.interpolation_type = lib::MIM;
	ecp->ecp_command.instruction.motion_steps = nr_of_steps;
	ecp->ecp_command.instruction.value_in_step_no = nr_of_steps;

	lib::Homog_matrix tmp;
	tmp.set_from_xyz_angle_axis(final_position);
	tmp.get_frame_tab(ecp->ecp_command.instruction.arm.pf_def.arm_frame);

	ecp->ecp_command.instruction.arm.pf_def.gripper_coordinate = final_position[6];
	execute_motion();

	for (int j = 0; j < 6; j++) { // Przepisanie aktualnych polozen
		current_position[j] = ecp->reply_package.arm.pf_def.arm_coordinates[j];
	}
	current_position[6] = ecp->reply_package.arm.pf_def.gripper_coordinate;
}

void ui_common_robot::move_xyz_angle_axis_relative(double position_increment[7])
{
	int nr_of_steps; // Liczba krokow
	int nr_ang, nr_lin, nr_grip;

	double max_inc_ang = 0.0, max_inc_lin = 0.0, max_inc_grip = 0.0, temp_lin, temp_ang, temp_grip; // Zmienne pomocnicze

	max_inc_ang = max_inc_lin = 0.0;
	for (int i = 0; i < 3; i++) {
		temp_lin = fabs(position_increment[i]);
		temp_ang = fabs(position_increment[i + 3]);
		if (temp_ang > max_inc_ang)
			max_inc_ang = temp_ang;
		if (temp_lin > max_inc_lin)
			max_inc_lin = temp_lin;
	}

	max_inc_grip = fabs(position_increment[6]);

	nr_ang = (int) ceil(max_inc_ang / END_EFFECTOR_ANGULAR_STEP);
	nr_lin = (int) ceil(max_inc_lin / END_EFFECTOR_LINEAR_STEP);
	nr_grip = (int) ceil(max_inc_grip / END_EFFECTOR_GRIPPER_STEP);

	nr_of_steps = (nr_ang > nr_lin) ? nr_ang : nr_lin;
	nr_of_steps = (nr_of_steps > nr_grip) ? nr_of_steps : nr_grip;

	// Zadano ruch do aktualnej pozycji
	if (nr_of_steps < 1)
		return;

	ecp->ecp_command.instruction.instruction_type = lib::SET_GET;
	ecp->ecp_command.instruction.get_arm_type = lib::FRAME;
	ecp->ecp_command.instruction.set_type = ARM_DV; // ARM
	ecp->ecp_command.instruction.set_arm_type = lib::FRAME;
	ecp->ecp_command.instruction.motion_type = lib::RELATIVE;
	ecp->ecp_command.instruction.interpolation_type = lib::MIM;
	ecp->ecp_command.instruction.motion_steps = nr_of_steps;
	ecp->ecp_command.instruction.value_in_step_no = nr_of_steps;

	lib::Homog_matrix tmp;
	tmp.set_from_xyz_angle_axis(position_increment);
	tmp.get_frame_tab(ecp->ecp_command.instruction.arm.pf_def.arm_frame);

	ecp->ecp_command.instruction.arm.pf_def.gripper_coordinate = position_increment[6];
	execute_motion();
}

// ---------------------------------------------------------------
void ui_common_robot::read_motors(double current_position[])
{
	// Zlecenie odczytu polozenia

	// printf("poczatek read motors\n");
	// Parametry zlecenia ruchu i odczytu polozenia
	ecp->ecp_command.instruction.get_type = ARM_DV;
	ecp->ecp_command.instruction.instruction_type = lib::GET;
	ecp->ecp_command.instruction.get_arm_type = lib::MOTOR;
	ecp->ecp_command.instruction.interpolation_type = lib::MIM;

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
void ui_common_robot::read_joints(double current_position[])
{
	// Zlecenie odczytu polozenia

	// Parametry zlecenia ruchu i odczytu polozenia
	ecp->ecp_command.instruction.instruction_type = lib::GET;
	ecp->ecp_command.instruction.get_type = ARM_DV;
	ecp->ecp_command.instruction.get_arm_type = lib::JOINT;
	ecp->ecp_command.instruction.interpolation_type = lib::MIM;

	execute_motion();

	for (int i = 0; i < ecp->number_of_servos; i++) // Przepisanie aktualnych polozen
		current_position[i] = ecp->reply_package.arm.pf_def.arm_coordinates[i];
}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
void ui_common_robot::read_xyz_euler_zyz(double current_position[])
{
	// Zlecenie odczytu polozenia

	// Parametry zlecenia ruchu i odczytu polozenia
	ecp->ecp_command.instruction.get_type = ARM_DV;
	ecp->ecp_command.instruction.instruction_type = lib::GET;
	ecp->ecp_command.instruction.get_arm_type = lib::FRAME;
	ecp->ecp_command.instruction.interpolation_type = lib::MIM;

	execute_motion();

	lib::Homog_matrix tmp;
	tmp.set_from_frame_tab(ecp->reply_package.arm.pf_def.arm_frame);
	tmp.get_xyz_euler_zyz(current_position);

	current_position[6] = ecp->reply_package.arm.pf_def.gripper_coordinate;
}
// ---------------------------------------------------------------


void ui_common_robot::read_xyz_angle_axis(double current_position[])
{
	// Pobranie aktualnego polozenia ramienia robota

	ecp->ecp_command.instruction.get_type = ARM_DV;
	ecp->ecp_command.instruction.instruction_type = lib::GET;
	ecp->ecp_command.instruction.get_arm_type = lib::FRAME;
	ecp->ecp_command.instruction.interpolation_type = lib::MIM;
	execute_motion();

	lib::Homog_matrix tmp;
	tmp.set_from_frame_tab(ecp->reply_package.arm.pf_def.arm_frame);
	tmp.get_xyz_angle_axis(current_position);

	current_position[6] = ecp->reply_package.arm.pf_def.gripper_coordinate;
}
