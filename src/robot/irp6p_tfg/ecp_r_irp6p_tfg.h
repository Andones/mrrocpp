#if !defined(_ECP_R_IRP6P_TFG_H)
#define _ECP_R_IRP6P_TFG_H

/*!
 * @file
 * @brief File contains ecp robot class declaration for IRp6 postument two finger gripper
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup irp6p_tfg
 */

#include "base/ecp/ecp_robot.h"
#include "robot/irp6p_tfg/const_irp6p_tfg.h"

#include "base/kinematics/kinematics_manager.h"

#include "robot/irp6p_tfg/kinematic_model_irp6p_tfg.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p_tfg {

// ---------------------------------------------------------------
class robot : public common::ecp_robot, public kinematics::common::kinematics_manager
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
