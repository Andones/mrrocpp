/*!
 * @file
 * @brief File contains ecp_sub_task_bias_edp_force definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup subtasks
 */

#include "base/lib/typedefs.h"

#include "base/lib/sr/srlib.h"
#include "generator/ecp/force/ecp_g_force.h"
#include "subtask/ecp_st_bias_edp_force.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

ecp_sub_task_bias_edp_force::ecp_sub_task_bias_edp_force(task &_ecp_t) :
	ecp_sub_task(_ecp_t)
{
	befg = new generator::bias_edp_force(_ecp_t);
}

void ecp_sub_task_bias_edp_force::conditional_execution()
{

	befg->Move();
}

} // namespace task

} // namespace common
} // namespace ecp
} // namespace mrrocpp
