// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			kinematic_model_conveyor.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Model kinematyki tasmociogu
//				- definicja metod klasy
//
// Autor:		tkornuta
// Data:		24.02.2007
// ------------------------------------------------------------------------

// Klasa kinematic_model_conveyor.
#include "kinematics/conveyor/kinematic_model_conveyor.h"

namespace mrrocpp {
namespace kinematics {
namespace conveyor {

/* -----------------------------------------------------------------------
  Konstruktor.
 ------------------------------------------------------------------------- */
model::model (void)
{
  // Ustawienie etykiety modelu kinematycznego.
  set_kinematic_model_label("Switching to standard kinematic model");

  // Ustawienie parametrow kinematycznych.
  set_kinematic_parameters();

} //: set_kinematic_parameters

/* -----------------------------------------------------------------------
  Ustawienia wszystkie parametry modelu kinematycznego danego modelu.
 ------------------------------------------------------------------------- */
void model::set_kinematic_parameters(void)
{
  // Polozenie synchronizacji.
  synchro_motor_position = 0;
  // Stosunek polozenia walu silnika do polozenia we wsp. wewn (zewn) w metrach.
  motor_to_intext_ratio = 2250;

} // end: set_kinematic_parameters



/* ------------------------------------------------------------------------
  Sprawdzenie ograniczen na polozenia katowe walow silnikow.
 ------------------------------------------------------------------------ */
void model::check_motor_position(const lib::MotorArray & motor_position)
{
	return;
} // end: kinematic_model_conveyor::check_motor_position(const )


/* ------------------------------------------------------------------------
  Sprawdzenie ograniczen na wspolrzedne wewnetrzne.
 ------------------------------------------------------------------------ */
void model::check_joints(const lib::JointArray & q)
{
	return;
} // end: kinematic_model_conveyor::check_joints(const )


/* ------------------------------------------------------------------------
  Przeliczenie polozenia walow silnikow na wspolrzedne wewnetrzne
  (mp2i - motor position to internal)
 ------------------------------------------------------------------------ */
void model::mp2i_transform(const lib::MotorArray & local_current_motor_pos, lib::JointArray & local_current_joints)
{
  local_current_joints[0] = local_current_motor_pos[0] / motor_to_intext_ratio;
}//: mp2i_transform



/* ------------------------------------------------------------------------
  Przeliczenie wspolrzednych wewnetrznych na polozenia walow silnikow
  (i2mp - internal to motor position)
 ------------------------------------------------------------------------ */
void model::i2mp_transform(lib::MotorArray & local_desired_motor_pos_new, lib::JointArray & local_desired_joints)
{
  local_desired_motor_pos_new[0] =  local_desired_joints[0] * motor_to_intext_ratio;
} //: i2mp_transform


/* ------------------------------------------------------------------------
  Zadanie proste kinematyki dla tasmociagu.
  Przeliczenie wspolrzednych wewnetrznych na wspolrzedne zewnetrzne.

  Wejscie:
  * current_joints[6] - wspolrzedne wewnetrzne robota (kolejno d0, q1, q2, ...)
  * local_tool[4][3] - macierz przeksztacenia jednorodnego (MPJ)
		opisujca przeksztalcenie zwiazane z narzedziem.

  Wyjscie:
  * current_end_effector_frame[4][3] - macierz przeksztacenia jednorodnego (MPJ)
		opisujca aktualne poloenie i orientacje koncowki (narzedzia) w ukladzie bazowym.
 ------------------------------------------------------------------------ */
void model::direct_kinematics_transform(const lib::JointArray & local_current_joints, lib::Homog_matrix& local_current_end_effector_frame)
{

  // Proste zadanie kinematyki.
  (local_current_end_effector_frame)[0][0] = 0.0;
  (local_current_end_effector_frame)[0][1] = 0.0;
  (local_current_end_effector_frame)[0][2] = 0.0;
  (local_current_end_effector_frame)[0][3] = local_current_joints[0];
  (local_current_end_effector_frame)[1][0] = 0.0;
  (local_current_end_effector_frame)[1][1] = 0.0;
  (local_current_end_effector_frame)[1][2] = 0.0;
  (local_current_end_effector_frame)[1][3] = 0.0;
  (local_current_end_effector_frame)[2][0] = 0.0;
  (local_current_end_effector_frame)[2][1] = 0.0;
  (local_current_end_effector_frame)[2][2] = 0.0;
  (local_current_end_effector_frame)[2][3] = 0.0;

  // Obliczenia zwiazane z przeksztalceniami do globalnego ukladu odniesienia.

} //:: direct_kinematics_transform()


/* ------------------------------------------------------------------------
  Zadanie odwrotne kinematyki dla tasmociagu.
  Przeliczenie wspolrzednych zewnetrznych na wspolrzedne wewnetrzne.

  Wejscie:
  * local_current_joints - obecne (w rzeczywistosci poprzednie) wspolrzedne wewnetrzne robota (kolejno d0, q1, q2, ...)
  * local_desired_end_effector_frame - macierz przeksztacenia jednorodnego (MPJ)
		opisujca zadane poloenie i orientacje koncowki (narzedzia) w ukladzie bazowym.
  * local_tool[4][3] - macierz przeksztacenia jednorodnego (MPJ)
		opisujca przeksztalcenie zwiazane z narzedziem.

  Wyjscie:
  * local_desired_joints - wyliczone wspolrzedne wewnetrzne robota (kolejno d0, q1, q2, ...)
 ------------------------------------------------------------------------ */
void model::inverse_kinematics_transform(lib::JointArray & local_desired_joints, lib::JointArray & local_current_joints, const lib::Homog_matrix& local_desired_end_effector_frame)
{
    local_desired_joints[0] = (local_desired_end_effector_frame)[0][3];
} //: inverse_kinematics_transform()

} // namespace conveyor
} // namespace kinematic
} // namespace mrrocpp

