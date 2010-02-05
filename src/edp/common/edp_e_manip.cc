// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			edp_irp6s.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Metody wspolne dla robotow IRp-6
// 				- definicja metod klasy edp_irp6s_effector
//
// Autor:		tkornuta
// Data:		14.02.2007
// -------------------------------------------------------------------------

#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <errno.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/mis_fun.h"
#include "lib/mrmath/mrmath.h"
#include "edp/common/servo_gr.h"
#include "edp/common/reader.h"
#include "edp/common/manip_trans_t.h"
#include "edp/common/edp_e_manip.h"

#include "kinematics/common/kinematic_model_with_tool.h"

using std::cout;

namespace mrrocpp {
namespace edp {
namespace common {

bool manip_effector::compute_servo_joints_and_frame(void)
{
	static int catch_nr = 0;
	bool ret_val = true;
	if (!(motor_driven_effector::compute_servo_joints_and_frame())) {
		ret_val = false;
	} else {
		// wyznaczenie nowych wartosci joints and frame dla obliczen w servo
		try {

			lib::Homog_matrix local_matrix;

			// Obliczenie lokalnej macierzy oraz obliczenie położenia robota we wsp. zewnętrznych.
			get_current_kinematic_model()->i2e_transform(servo_current_joints, local_matrix);
			// Pobranie wsp. zewnętrznych w układzie

			lib::Xyz_Euler_Zyz_vector servo_real_kartez_pos; // by Y polozenie we wspolrzednych xyz_euler_zyz obliczane co krok servo   XXXXX
			local_matrix.get_xyz_euler_zyz(servo_real_kartez_pos);

			//obliczanie zadanej pozycji koncowki wedlug aktualnego rozkazu przetwarzanego w servo


			lib::MotorArray servo_desired_motor_pos(sb->command.parameters.move.abs_position, number_of_servos);

			lib::JointArray servo_desired_joints(number_of_servos);

			get_current_kinematic_model()->mp2i_transform(servo_desired_motor_pos, servo_desired_joints);
			get_current_kinematic_model()->i2e_transform(servo_desired_joints, local_matrix);
			// Pobranie wsp. zewnętrznych w układzie

			lib::Xyz_Euler_Zyz_vector servo_desired_kartez_pos; // by Y polozenie we wspolrzednych xyz_euler_zyz obliczane co krok servo   XXXXX
			local_matrix.get_xyz_euler_zyz(servo_desired_kartez_pos);

			// scope-locked reader data update
			{
				boost::mutex::scoped_lock lock(rb_obj->reader_mutex);

				servo_real_kartez_pos.to_table(rb_obj->step_data.real_cartesian_position);
				servo_desired_kartez_pos.to_table(rb_obj->step_data.desired_cartesian_position);
			}

			// Obliczenie polozenia robota we wsp. zewnetrznych bez narzedzia.
			((mrrocpp::kinematics::common::kinematic_model_with_tool*) get_current_kinematic_model())->i2e_wo_tool_transform(servo_current_joints, servo_current_frame_wo_tool);

			catch_nr = 0;

		}//: try
		catch (...) {
			if ((++catch_nr) == 1)
				printf("servo thread compute_servo_joints_and_frame throw catch exception\n");
			ret_val = false;
		}//: catch
	}

	return ret_val;
}

/*--------------------------------------------------------------------------*/
manip_effector::manip_effector(lib::configurator &_config, lib::robot_name_t l_robot_name) :
	motor_driven_effector(_config, l_robot_name)
{
}

/*--------------------------------------------------------------------------*/
void manip_effector::get_arm_position_get_arm_type_switch(lib::c_buffer &instruction)
{ // odczytanie pozycji ramienia

	switch (instruction.get_arm_type)
	{
		case lib::FRAME:
			// przeliczenie wspolrzednych do poziomu, ktory ma byc odczytany
			get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);
			get_current_kinematic_model()->i2e_transform(current_joints, current_end_effector_frame);
			// TRANS z wewntrznych struktur danych TRANSFORMATORa
			// do wewntrznych struktur danych REPLY_BUFFER

			current_end_effector_frame.get_frame_tab(reply.arm.pf_def.arm_frame);
			break;
		default: // blad: nieznany sposob zapisu wspolrzednych koncowki
			motor_driven_effector::get_arm_position_get_arm_type_switch(instruction);
	}

}



/*--------------------------------------------------------------------------*/
void manip_effector::compute_base_pos_xyz_rot_xyz_vector(const lib::JointArray begining_joints, const lib::Homog_matrix begining_end_effector_frame, const lib::c_buffer &instruction, lib::Xyz_Angle_Axis_vector& base_pos_xyz_rot_xyz_vector)
{
	const lib::MOTION_TYPE &motion_type = instruction.motion_type;
	const lib::POSE_SPECIFICATION &set_arm_type = instruction.set_arm_type;
	lib::Homog_matrix arm_frame(instruction.arm.pf_def.arm_frame);
	lib::JointArray joint_arm_coordinates(instruction.arm.pf_def.arm_coordinates, number_of_servos);
	lib::MotorArray motor_arm_coordinates(instruction.arm.pf_def.arm_coordinates, number_of_servos);
	const uint16_t &ECP_motion_steps = instruction.motion_steps; // liczba krokow w makrokroku

	// WYLICZENIE POZYCJI POCZATKOWEJ
	lib::JointArray tmp_joints(number_of_servos);
	lib::MotorArray tmp_motor_pos(number_of_servos);

	lib::Homog_matrix goal_frame;

	lib::Homog_matrix goal_frame_increment_in_end_effector;
	lib::Xyz_Angle_Axis_vector goal_xyz_angle_axis_increment_in_end_effector;

	switch (motion_type)
	{
		case lib::ABSOLUTE:
			switch (set_arm_type)
			{
				case lib::FRAME:
					goal_frame = arm_frame;
					break;
				case lib::JOINT:
					get_current_kinematic_model()->i2e_transform(joint_arm_coordinates, goal_frame);

					break;
				case lib::MOTOR:
					get_current_kinematic_model()->mp2i_transform(motor_arm_coordinates, tmp_joints);
					get_current_kinematic_model()->i2e_transform(tmp_joints, goal_frame);

					break;
				default:
					break;
			}
			break;
		case lib::RELATIVE:
			switch (set_arm_type)
			{
				case lib::FRAME:
					goal_frame = arm_frame;
					goal_frame = begining_end_effector_frame * goal_frame;
					break;
				case lib::JOINT:
					for (int i = 0; i < number_of_servos; i++) {
						tmp_joints[i] = begining_joints[i] + joint_arm_coordinates[i];
					}
					get_current_kinematic_model()->i2e_transform(tmp_joints, goal_frame);
					break;
				case lib::MOTOR:
					for (int i = 0; i < number_of_servos; i++) {
						tmp_motor_pos[i] = desired_motor_pos_new[i] + motor_arm_coordinates[i];
					}
					get_current_kinematic_model()->mp2i_transform(tmp_motor_pos, tmp_joints);
					get_current_kinematic_model()->i2e_transform(tmp_joints, goal_frame);
					break;
				default:
					break;
			}
			break;
		default:
			break;

	}

	// WYZNACZENIE PREDKOSCI RUCHU

	switch (set_arm_type)
	{
		case lib::FRAME:
		case lib::JOINT:
		case lib::MOTOR:
			goal_frame_increment_in_end_effector = ((!begining_end_effector_frame) * goal_frame);
			goal_frame_increment_in_end_effector.get_xyz_angle_axis(goal_xyz_angle_axis_increment_in_end_effector);
			for (int i = 0; i < 6; i++) {
				base_pos_xyz_rot_xyz_vector[i] = goal_xyz_angle_axis_increment_in_end_effector[i] * (double) (1
						/ (((double) STEP) * ((double) ECP_motion_steps)));
			}
			break;
		case lib::PF_VELOCITY:
			base_pos_xyz_rot_xyz_vector.set_values(instruction.arm.pf_def.arm_coordinates);
			break;
		default:
			throw System_error();
	}
}




/*--------------------------------------------------------------------------*/
void manip_effector::set_rmodel(lib::c_buffer &instruction)
{
	// uint8_t previous_model;
	// uint8_t previous_corrector;
	//printf(" SET RMODEL: ");
	switch (instruction.set_rmodel_type)
	{
		case lib::FORCE_TOOL:
			if (vs == NULL) {
				printf("Nie wlaczono force_tryb=2 w pliku ini\n");
				break;
			}
			{
				boost::mutex::scoped_lock lock(vs->mtx);
				vs->new_command_synchroniser.null_command();
				vs->command = FORCE_SET_TOOL;
				for (int i = 0; i < 3; i++) {
					vs->next_force_tool_position[i] = instruction.rmodel.force_tool.position[i];
				}
				vs->next_force_tool_weight = instruction.rmodel.force_tool.weight;
				vs->new_edp_command = true;
			}
			vs->new_command_synchroniser.wait();
			break;
		case lib::FORCE_BIAS:
			if (vs == NULL) {
				printf("Nie wlaczono force_tryb=2 w pliku ini\n");
				break;
			}
			{
				boost::mutex::scoped_lock lock(vs->mtx);
				vs->new_command_synchroniser.null_command();
				vs->command = FORCE_CONFIGURE;
				vs->new_edp_command = true;
			}
			vs->new_command_synchroniser.wait();
			break;
		case lib::TOOL_FRAME:
			//printf("TOOL_FRAME\n");
			// przepisa specyfikacj do TRANSFORMATORa
			// Przepisanie definicji narzedzia danej w postaci TOOL_FRAME
			// do wewntrznych struktur danych TRANSFORMATORa
			// Sprawdzenie czy przepisana macierz jest jednorodna
			// Jezeli nie, to wyzwalany jest wyjatek.


			// Przyslano dane dotyczace narzedzia i koncowki.
			// Sprawdzenie poprawnosci macierzy
			//	set_tool_frame_in_kinematic_model(lib::Homog_matrix(instruction.rmodel.tool_frame_def.tool_frame));
		{
			lib::Homog_matrix hm(instruction.rmodel.tool_frame_def.tool_frame);

			if (!(hm.is_valid())) {
				throw NonFatal_error_2(INVALID_HOMOGENEOUS_MATRIX);
			}
			// Ustawienie macierzy reprezentujacej narzedzie.
			((mrrocpp::kinematics::common::kinematic_model_with_tool*) get_current_kinematic_model())->tool = hm;

			/*
			 // odswierzanie
			 get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);
			 get_current_kinematic_model()->i2e_transform(current_joints, &current_end_effector_frame);
			 */
		}

			break;
		default: // blad: nie istniejaca specyfikacja modelu robota
			motor_driven_effector::set_rmodel(instruction);
	}
}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void manip_effector::get_rmodel(lib::c_buffer &instruction)
{
	//printf(" GET RMODEL: ");
	switch (instruction.get_rmodel_type)
	{
		case lib::FORCE_TOOL:
			if (vs == NULL) {
				printf("Nie wlaczono force_tryb=2 w pliku ini\n");
				break;
			}
			for (int i = 0; i < 3; i++) {
				reply.rmodel.force_tool.position[i] = vs->current_force_tool_position[i];
			}
			reply.rmodel.force_tool.weight = vs->current_force_tool_weight;
			break;
		case lib::TOOL_FRAME:
			//printf("TOOL_FRAME\n");
			// przepisac specyfikacje z TRANSFORMATORa do bufora wysylkowego
			// Przepisanie definicji narzedzia danej w postaci TOOL_FRAME
			// z wewntrznych struktur danych TRANSFORMATORa
			// do wewntrznych struktur danych REPLY_BUFFER

			reply.rmodel_type = lib::TOOL_FRAME;

			((mrrocpp::kinematics::common::kinematic_model_with_tool*) get_current_kinematic_model())->tool.get_frame_tab(reply.rmodel.tool_frame_def.tool_frame);

			break;
		default: // blad: nie istniejaca specyfikacja modelu robota
			motor_driven_effector::get_rmodel(instruction);
	}
}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void manip_effector::compute_frame(const lib::c_buffer &instruction)
{
	lib::MotorArray desired_motor_pos_new_tmp(number_of_servos);
	lib::JointArray desired_joints_tmp(number_of_servos); // Wspolrzedne wewnetrzne -
	const lib::MOTION_TYPE &motion_type = instruction.motion_type;
	// obliczenia dla ruchu ramienia (kocwk: FRAME)
	/* Wypenienie struktury danych transformera na podstawie parametrow polecenia otrzymanego z ECP */
	/* Zlecenie transformerowi przeliczenie wspolrzednych */

	motion_steps = instruction.motion_steps;
	value_in_step_no = instruction.value_in_step_no;
	lib::Homog_matrix p_m(instruction.arm.pf_def.arm_frame);

	if ((value_in_step_no <= 0) || (motion_steps <= 0) || (value_in_step_no > motion_steps + 1)) {
		throw NonFatal_error_2(INVALID_MOTION_PARAMETERS);
	}
	switch (motion_type)
	{
		case lib::ABSOLUTE: // ruch bezwzgledny
			desired_end_effector_frame = p_m;
			//      fprintf(stderr, "debug@%s:%d\n", __FILE__, __LINE__);
			break;
		case lib::RELATIVE: // ruch wzgledny
			//      fprintf(stderr, "debug@%s:%d\n", __FILE__, __LINE__);
			desired_end_effector_frame = current_end_effector_frame * p_m;
			break;
		default:
			throw NonFatal_error_2(INVALID_MOTION_TYPE);
	}
	// Przeliczenie wspolrzednych zewnetrznych na wspolrzedne wewnetrzne
	get_current_kinematic_model()->e2i_transform(desired_joints_tmp, current_joints, desired_end_effector_frame);
	// Przeliczenie wspolrzednych wewnetrznych na polozenia walow silnikow
	get_current_kinematic_model()->i2mp_transform(desired_motor_pos_new_tmp, desired_joints_tmp);

	// kinematyka nie stwierdzila bledow, przepisanie wartosci
	for (int i = 0; i < number_of_servos; i++) {
		desired_joints[i] = desired_joints_tmp[i];
		desired_motor_pos_new[i] = desired_motor_pos_new_tmp[i];
	}
}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/

lib::Homog_matrix manip_effector::return_current_frame(TRANSLATION_ENUM translation_mode)
{// by Y
	boost::mutex::scoped_lock lock(edp_irp6s_effector_mutex);
	// przepisanie danych na zestaw lokalny dla edp_force
	// lib::copy_frame(force_current_end_effector_frame, global_current_end_effector_frame);
	lib::Homog_matrix return_frame(servo_current_frame_wo_tool);

	if (translation_mode == WITHOUT_TRANSLATION)
		return_frame.remove_translation();
	return return_frame;
}

void manip_effector::force_msr_upload(const lib::Ft_vector l_vector)
{// by Y wgranie globalnego zestawu danych
	boost::mutex::scoped_lock lock(force_mutex);
	global_force_msr = l_vector;
}

// by Y odczytanie globalnego zestawu danych
void manip_effector::force_msr_download(lib::Ft_vector& l_vector)
{
	boost::mutex::scoped_lock lock(force_mutex);
	l_vector = global_force_msr;
}

/*--------------------------------------------------------------------------*/

// Synchronizacja robota.
void manip_effector::synchronise()
{
	motor_driven_effector::synchronise();
	get_current_kinematic_model()->i2e_transform(current_joints, current_end_effector_frame);
}

//   sprawdza stan robota
void manip_effector::get_controller_state(lib::c_buffer &instruction)
{
	motor_driven_effector::get_controller_state(instruction);
	if (is_synchronised()) {
		get_current_kinematic_model()->i2e_transform(current_joints, current_end_effector_frame);
	}
}

/*--------------------------------------------------------------------------*/
void manip_effector::single_thread_move_arm(lib::c_buffer &instruction)
{ // przemieszczenie ramienia
	// Wypenienie struktury danych transformera na podstawie parametrow polecenia
	// otrzymanego z ECP. Zlecenie transformerowi przeliczenie wspolrzednych


	switch (instruction.set_arm_type)
	{
		case lib::FRAME:
			compute_frame(instruction);
			move_servos();
			break;
		default: // blad: niezdefiniowany sposb specyfikacji pozycji koncowki
			motor_driven_effector::single_thread_move_arm(instruction);
	}

}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void manip_effector::multi_thread_move_arm(lib::c_buffer &instruction)
{ // przemieszczenie ramienia
	// Wypenienie struktury danych transformera na podstawie parametrow polecenia
	// otrzymanego z ECP. Zlecenie transformerowi przeliczenie wspolrzednych

	switch (instruction.set_arm_type)
	{
		case lib::FRAME:
			compute_frame(instruction);
			move_servos();
			mt_tt_obj->trans_t_to_master_synchroniser.command();

			break;
		default: // blad: niezdefiniowany sposb specyfikacji pozycji koncowki
			motor_driven_effector::multi_thread_move_arm(instruction);
	}
}
/*--------------------------------------------------------------------------*/

} // namespace common
} // namespace edp
} // namespace mrrocpp

