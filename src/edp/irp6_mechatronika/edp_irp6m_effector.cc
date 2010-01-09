// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			edp_irp6m_effector.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Robot IRp-6 na postumencie
//				- definicja metod klasy edp_irp6m_effector
//				- definicja funkcji return_created_efector()
//
// Autor:		tkornuta
// Data:		14.02.2007
// ------------------------------------------------------------------------

#include <stdio.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/mis_fun.h"
#include "lib/mrmath/mrmath.h"

// Klasa edp_irp6ot_effector.
#include "edp/irp6_mechatronika/edp_irp6m_effector.h"
#include "edp/common/reader.h"
// Kinematyki.
#include "kinematics/irp6_mechatronika/kinematic_model_irp6m_with_wrist.h"
#include "kinematics/irp6_mechatronika/kinematic_model_irp6m_5dof.h"
#include "edp/common/servo_gr.h"
#include "edp/common/manip_trans_t.h"
#include "kinematics/common/kinematic_model_with_tool.h"

namespace mrrocpp {
namespace edp {
namespace irp6m {

common::servo_buffer* effector::return_created_servo_buffer()
{
	return new irp6m::servo_buffer(*this);
}

void effector::master_order(common::MT_ORDER nm_task, int nm_tryb)
{
	manip_and_conv_effector::multi_thread_master_order(nm_task, nm_tryb);
}

// Konstruktor.
effector::effector(lib::configurator &_config) :
	manip_effector(_config, lib::ROBOT_IRP6_MECHATRONIKA)
{
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();

	number_of_servos = IRP6_MECHATRONIKA_NUM_OF_SERVOS;

	reset_variables();
}

/*--------------------------------------------------------------------------*/
void effector::set_rmodel(lib::c_buffer &instruction)
{
	// uint8_t previous_model;
	// uint8_t previous_corrector;
	//printf(" SET RMODEL: ");
	switch (instruction.set_rmodel_type)
	{
		case lib::SERVO_ALGORITHM:
			sb->set_rmodel_servo_algorithm(instruction);
			break;

		default: // blad: nie istniejaca specyfikacja modelu robota
			// ustawia numer bledu
			manip_effector::set_rmodel(instruction);
	}
}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void effector::move_arm(lib::c_buffer &instruction)
{ // przemieszczenie ramienia
	// Wypenienie struktury danych transformera na podstawie parametrow polecenia
	// otrzymanego z ECP. Zlecenie transformerowi przeliczenie wspolrzednych


	manip_effector::multi_thread_move_arm(instruction);

}
/*--------------------------------------------------------------------------*/

// sprawdza stan EDP zaraz po jego uruchomieniu


void effector::servo_joints_and_frame_actualization_and_upload(void)
{
	static int catch_nr = 0;



	// wyznaczenie nowych wartosci joints and frame dla obliczen w servo
	try {
		get_current_kinematic_model()->mp2i_transform(servo_current_motor_pos, servo_current_joints);

		// scope-locked reader data update
		{
			boost::mutex::scoped_lock lock(rb_obj->reader_mutex);

			for (int j = 0; j < number_of_servos; j++) {
				rb_obj->step_data.current_joints[j] = servo_current_joints[j];
			}
		}

		// Obliczenie lokalnej macierzy oraz obliczenie położenia robota we wsp. zewnętrznych.
		lib::Homog_matrix local_matrix;
		get_current_kinematic_model()->i2e_transform(servo_current_joints, local_matrix);
		// Pobranie wsp. zewnętrznych w układzie


		local_matrix.get_mech_xyz_euler_zyz(servo_real_kartez_pos);

		// scope-locked reader data update
		{
			boost::mutex::scoped_lock lock(rb_obj->reader_mutex);

			for (int i = 0; i < 6; i++) {
				rb_obj->step_data.real_cartesian_position[i] = servo_real_kartez_pos[i];
			}
		}

		// Obliczenie polozenia robota we wsp. zewnetrznych bez narzedzia.
		((mrrocpp::kinematics::common::kinematic_model_with_tool*) get_current_kinematic_model())->i2e_wo_tool_transform(servo_current_joints, servo_current_frame_wo_tool);

		catch_nr = 0;
	}//: try

	catch (...) {
		if ((++catch_nr) == 1)
			printf("servo thread servo_joints_and_frame_actualization_and_upload throw catch exception\n");
	}

	{
		boost::mutex::scoped_lock lock(edp_irp6s_effector_mutex);

		// przepisnie danych na zestaw globalny
		for (int i = 0; i < number_of_servos; i++) {
			global_current_motor_pos[i] = servo_current_motor_pos[i];
			global_current_joints[i] = servo_current_joints[i];
		}

		// T.K.: Nad tym trzeba pomyslec - co w tym momencie dzieje sie z global_current_end_effector_frame?
		// Jezeli zmienna ta przechowyje polozenie bez narzedzia, to nazwa jest nie tylko nieadekwatna, a wrecz mylaca.
		global_current_frame_wo_tool = servo_current_frame_wo_tool;
	}
}

/*--------------------------------------------------------------------------*/
void effector::get_arm_position(bool read_hardware, lib::c_buffer &instruction)
{ // odczytanie pozycji ramienia
	//lib::JointArray desired_joints_tmp(MAX_SERVOS_NR); // Wspolrzedne wewnetrzne -
	//   printf(" GET ARM\n");

	if (read_hardware) {
		manip_and_conv_effector::get_arm_position_read_hardware_sb();
	}

	// okreslenie rodzaju wspolrzednych, ktore maja by odczytane
	// oraz adekwatne wypelnienie bufora odpowiedzi
	common::manip_effector::get_arm_position_get_arm_type_switch(instruction);

	// scope-locked reader data update
	{
		boost::mutex::scoped_lock lock(rb_obj->reader_mutex);

		reply.servo_step = rb_obj->step_data.step;
	}
}
/*--------------------------------------------------------------------------*/

// Stworzenie modeli kinematyki dla robota IRp-6 na postumencie.
void effector::create_kinematic_models_for_given_robot(void)
{
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::irp6m::model_with_wrist());
	add_kinematic_model(new kinematics::irp6m::model_5dof());
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);
}

} // namespace irp6m
namespace common {

// Stworzenie obiektu edp_irp6m_effector.
effector* return_created_efector(lib::configurator &_config)
{
	return new irp6m::effector(_config);
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

