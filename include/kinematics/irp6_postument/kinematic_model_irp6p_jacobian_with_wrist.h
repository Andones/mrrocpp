// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			kinematic_model_irp6p_jacobian_with_wrist.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		Model kinematyki robota IRp-6 na postumencie
//				- deklaracja klasy
//				- przedefiniowanie rozwiazania odwrotnego zadania
//				  kinematyki - metoda uwzgledniajaca jakobian
//
// Autor:		Anna Maria Sibilska
// Data:		18.07.2007
// ------------------------------------------------------------------------

#if !defined(_IRP6P_KIN_MODEL_WITH_WRIST_JACOBIAN)
#define _IRP6P_KIN_MODEL_WITH_WRIST_JACOBIAN

// Definicja klasy kinematic_model.
#include "kinematics/irp6_postument/kinematic_model_irp6p_with_wrist.h"

namespace mrrocpp {
namespace kinematic {
namespace irp6p {

class kinematic_model_irp6p_jacobian_with_wrist: public kinematic_model_irp6p_with_wrist
{

public:
  // Konstruktor.
  kinematic_model_irp6p_jacobian_with_wrist (void);

  //Rozwiazanie odwrotnego zagadnienia kinematyki.
  virtual void inverse_kinematics_transform(double* local_desired_joints, double* local_current_joints, frame_tab* local_desired_end_effector_frame);

};//: kinematic_model_irp6p_jacobian_with_wrist

} // namespace irp6p
} // namespace kinematic
} // namespace mrrocpp

#endif
