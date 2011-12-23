/*!
 * @file edp_e_spkm2.h
 * @brief File containing the declaration of edp::spkm::effector class.
 *
 * @author Tomasz Winiarski
 * @date 2009
 *
 */

#ifndef __EDP_E_SPKM2_H
#define __EDP_E_SPKM2_H

#include "edp_e_spkm.h"

namespace mrrocpp {
namespace edp {
namespace spkm2 {

/*!
 * @brief class of EDP SwarmItFix parallel kinematic manipulator
 *
 * It is the base of the head mounted on the mobile base.
 */
class effector : public spkm::effector
{
private:

protected:

	/*!
	 * @brief method,  creates a list of available kinematic models for spkm effector.
	 *
	 * Here it is parallel manipulator direct and inverse kinematic transform
	 * and motor to joint transform
	 */
	virtual void create_kinematic_models_for_given_robot(void);

public:

	/*!
	 * @brief Initializes EPOS objects, basing on motor mappings.
	 */
	effector(common::shell &_shell);

};

} // namespace spkm2
} // namespace edp
} // namespace mrrocpp


#endif
