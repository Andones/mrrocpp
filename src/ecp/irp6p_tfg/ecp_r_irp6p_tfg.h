// -------------------------------------------------------------------------
//                            ecp_local.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP
// robot - irp6_postument
//
// -------------------------------------------------------------------------

#if !defined(_ECP_R_IRP6P_TFG_H)
#define _ECP_R_IRP6P_TFG_H

#include "ecp/common/ecp_robot.h"
#include "lib/irp6p_tfg_const.h"

#include "kinematics/common/kinematics_manager.h"

#include "kinematics/irp6p_tfg/kinematic_model_irp6p_tfg.h"


namespace mrrocpp {
namespace ecp {
namespace irp6p {

// ---------------------------------------------------------------
class robot: public common::ecp_robot, public kinematics::common::kinematics_manager
{
	// Klasa dla robota irp6_postument (sztywnego)
protected:
	// Metoda tworzy modele kinematyczne dla robota IRp-6 na torze.
	virtual void create_kinematic_models_for_given_robot(void);

public:
	robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp);
	robot(common::task::task& _ecp_object);

}; // end: class ecp_irp6_postument_robot
// ---------------------------------------------------------------

} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp

#endif
