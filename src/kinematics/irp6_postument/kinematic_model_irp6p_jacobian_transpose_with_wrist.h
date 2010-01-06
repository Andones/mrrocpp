// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			kinematic_model_irp6p_jacobian_transpose_with_wrist.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		Model kinematyki robota IRp-6 na postumencie
//				- deklaracja klasy
//				- przedefiniowanie rozwiazania odwrotnego zadania
//				  kinematyki - metoda uwzgledniajaca jakobian transponowany
//
// Autor:		Anna Maria Sibilska
// Data:		18.07.2007
// ------------------------------------------------------------------------

#if !defined(_IRP6P_KIN_MODEL_WITH_WRIST_JACOBIAN_TRANSPOSE)
#define _IRP6P_KIN_MODEL_WITH_WRIST_JACOBIAN_TRANSPOSE

// Definicja klasy kinematic_model.
#include "kinematics/irp6_postument/kinematic_model_irp6p_with_wrist.h"

namespace mrrocpp {
namespace kinematics {
namespace irp6p {

class model_jacobian_transpose_with_wrist: public model_with_wrist
{

public:
  // Konstruktor.
  model_jacobian_transpose_with_wrist (void);

  //Rozwiazanie odwrotnego zagadnienia kinematyki.
  virtual void inverse_kinematics_transform(std::vector<double> & local_desired_joints, std::vector<double> & local_current_joints, lib::Homog_matrix& local_desired_end_effector_frame);

};//: kinematic_model_irp6p_jacobian_transpose_with_wrist

} // namespace irp6p
} // namespace kinematic
} // namespace mrrocpp

#endif
