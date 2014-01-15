/*!
 * @file
 * @brief File contains  tff nose run generator definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup generators
 */

#include <cstdio>
#include <fstream>
#include <iostream>
#include <iostream>

#include "base/lib/typedefs.h"

#include "base/lib/sr/sr_ecp.h"
#include "base/ecp/ecp_robot.h"
#include "ecp_g_hswitals_generatore.h"
#include "ecp_mp_g_hswitals_generatore.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

hswitals_generatore::hswitals_generatore(common::task::task& _ecp_task, int step) :
        common::generator::tff_nose_run(_ecp_task, step)
{
    generator_name = ecp_mp::generator::ECP_GEN_HSWITALS_GENERATORE;
	// domyslnie wszytkie osie podatne a pulse_check nieaktywne
	configure_behaviour(lib::CONTACT, lib::CONTACT, lib::CONTACT, lib::CONTACT, lib::CONTACT, lib::CONTACT);
	configure_pulse_check(false);
	configure_velocity(0.0,0.0,0.0,0.0,0.0,0.0);
    configure_force(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	configure_reciprocal_damping(lib::FORCE_RECIPROCAL_DAMPING, lib::FORCE_RECIPROCAL_DAMPING, lib::FORCE_RECIPROCAL_DAMPING, lib::TORQUE_RECIPROCAL_DAMPING, lib::TORQUE_RECIPROCAL_DAMPING, lib::TORQUE_RECIPROCAL_DAMPING);
	configure_inertia(lib::FORCE_INERTIA, lib::FORCE_INERTIA, lib::FORCE_INERTIA, lib::TORQUE_INERTIA, lib::TORQUE_INERTIA, lib::TORQUE_INERTIA);

    std::cout<<"HSWITALS 123 test"<<std::endl;

}

void hswitals_generatore::conditional_execution()
{

    switch ((ecp_mp::generator::hswitals_generatore::communication_type) ecp_t.mp_command.ecp_next_state.variant)
	{
        case ecp_mp::generator::hswitals_generatore::specification: {
            ecp_mp::generator::hswitals_generatore::specification_data_type dt;
            ecp_t.mp_command.ecp_next_state.sg_buf.get(dt);
            configure_behaviour(dt.behaviour[0], dt.behaviour[1], dt.behaviour[2], dt.behaviour[3], dt.behaviour[4], dt.behaviour[5]);
            configure_inertia(dt.inertia[0], dt.inertia[1], dt.inertia[2], dt.inertia[3], dt.inertia[4], dt.inertia[5]);
            configure_reciprocal_damping(dt.reciprocal_damping[0], dt.reciprocal_damping[1], dt.reciprocal_damping[2], dt.reciprocal_damping[3], dt.reciprocal_damping[4], dt.reciprocal_damping[5]);
            break;
		}
        case ecp_mp::generator::hswitals_generatore::no_data:
			break;
		default:
			break;
	}

	Move();
}


} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
