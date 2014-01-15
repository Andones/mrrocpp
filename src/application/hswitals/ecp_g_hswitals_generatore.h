#if !defined(_ECP_GEN_HSWITALS_GENERATORE_H)
#define _ECP_GEN_HSWITALS_GENERATORE_H

/*!
 * @file
 * @brief File contains tff nose run generator declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup generators
 */

#include "ecp_mp_g_hswitals_generatore.h"
#include "base/ecp/ecp_generator.h"

#include "generator/ecp/tff_nose_run/ecp_g_tff_nose_run.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/*!
 * @brief generator to move manipulator end-effector by exerting force to the tool
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup generators
 */
class hswitals_generatore : public ecp::common::generator::tff_nose_run
{
protected:

public:

    hswitals_generatore(common::task::task& _ecp_task, int step = 0);
    void conditional_execution();
};
// end : class ecp_hswitals_generatore_generator

}// namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
