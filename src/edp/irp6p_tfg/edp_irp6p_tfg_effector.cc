// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			edp_irp6p_effector.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Robot IRp-6 na postumencie
//				- definicja metod klasy edp_irp6p_effector
//				- definicja funkcji return_created_efector()
//
// Autor:		tkornuta
// Data:		14.02.2007
// ------------------------------------------------------------------------

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/mrmath/mrmath.h"

// Klasa edp_irp6ot_effector.
#include "edp/irp6p_tfg/edp_irp6p_tfg_effector.h"
// Kinematyki.
#include "kinematics/irp6_postument/kinematic_model_irp6p_with_wrist.h"
#include "kinematics/irp6_postument/kinematic_model_irp6p_5dof.h"
#include "kinematics/irp6_postument/kinematic_model_calibrated_irp6p_with_wrist.h"
#include "kinematics/irp6_postument/kinematic_model_irp6p_jacobian_with_wrist.h"
#include "kinematics/irp6_postument/kinematic_model_irp6p_jacobian_transpose_with_wrist.h"
#include "kinematics/common/kinematic_model_with_tool.h"

namespace mrrocpp {
namespace edp {
namespace irp6p {

common::servo_buffer* effector::return_created_servo_buffer ()
{
	return new irp6p::servo_buffer (*this);
}

// Konstruktor.
effector::effector(lib::configurator &_config) :
	irp6s_postument_track_effector(_config, lib::ROBOT_IRP6_POSTUMENT)
{
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();

	if (is_gripper_active)
		number_of_servos = IRP6_POSTUMENT_NUM_OF_SERVOS;
	else
		number_of_servos = IRP6_POSTUMENT_NUM_OF_SERVOS-1;

	gripper_servo_nr = IRP6P_GRIPPER_CATCH_AXE;

	reset_variables();
}

// Stworzenie modeli kinematyki dla robota IRp-6 na postumencie.
void effector::create_kinematic_models_for_given_robot(void)
{
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::irp6p::model_with_wrist(number_of_servos));
	add_kinematic_model(new kinematics::irp6p::model_5dof(number_of_servos));
	add_kinematic_model(new kinematics::irp6p::model_calibrated_with_wrist(number_of_servos));
	add_kinematic_model(new kinematics::irp6p::model_jacobian_with_wrist(number_of_servos));
	add_kinematic_model(new kinematics::irp6p::model_jacobian_transpose_with_wrist(number_of_servos));
	//add_kinematic_model(new kinematic_model_irp6p_jacobian_with_wrist());

	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);
}

} // namespace irp6p

namespace common {

// Stworzenie obiektu edp_irp6p_effector.
effector* return_created_efector(lib::configurator &_config)
{
	return new irp6p::effector (_config);
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

