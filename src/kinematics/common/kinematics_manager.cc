// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			kinematics_manager.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Klasa zarzadzajaca modelami kinematyki.
//				- definicja metod klasy
//				- wspolna dla wszystkich robotow
//
// Autor:		tkornuta
// Data:		19.01.2007
// ------------------------------------------------------------------------

#include "lib/com_buf.h"
#include "kinematics/common/transformer_error.h"

#include "kinematics/common/kinematics_manager.h"

namespace mrrocpp {
namespace kinematics {
namespace common {

// extern edp_effector* master;   // Bufor polecen i odpowiedzi EDP_MASTER

// Konstruktor - tworzy liste kinematyk.
manager::manager(void)
{
    //	create_kinematics_list_for_given_robot();
};//: manager



// Destruktor - niszczy liste kinematyk.
manager::~manager(void)
{
    // tutaj usuniecie kinematyk z listy oraz samej listy.
};//:~manager


// Zmiana aktywnego modelu kinematyki.
void manager::set_kinematic_model (int _desired_kinematic_model_nr)
{
    if (_desired_kinematic_model_nr >= kinematic_models_list.size() || _desired_kinematic_model_nr <0 )
    {
        throw common::transformer_error::NonFatal_error_2 (INVALID_KINEMATIC_MODEL_NO);
    }
    current_kinematic_model = (simple_kinematic_model*) (kinematic_models_list[_desired_kinematic_model_nr]);
    current_kinematic_model_no = _desired_kinematic_model_nr;
    // Wypisanie nazwy modelu kinematyki.
  //  master->msg->message(current_kinematic_model->get_kinematic_model_label());
}


// Zmiana aktywnego modelu kinematyki.
void manager::add_kinematic_model (simple_kinematic_model* _model)
{
    // Dodanie nowego modelu na koniec listy.
    kinematic_models_list[kinematic_models_list.size()] = _model;
}

// Zwraca obecnie wybrany modelu kinematyki.
simple_kinematic_model* manager::get_current_kinematic_model (void)
{
    return current_kinematic_model;
}

// Zwraca number obecnie wybranego modelu kinematyki.
int manager::get_current_kinematic_model_no (void)
{
    return current_kinematic_model_no;
}

} // namespace common
} // namespace kinematics
} // namespace mrrocpp
