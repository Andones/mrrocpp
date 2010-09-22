#if !defined(_ECP_ST_BIAS_EDP_FORCE_H)
#define _ECP_ST_BIAS_EDP_FORCE_H

/*!
 * @file
 * @brief File contains sub_task_bias_edp_force declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup subtasks
 */

#include "base/ecp/ecp_task.h"
#include "generator/ecp/force/ecp_g_bias_edp_force.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
class bias_edp_force;
}

namespace task {

/*!
 * @brief subtask to execute bias_edp_force generator
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup subtasks
 */
class sub_task_bias_edp_force : public sub_task
{

private:

	/*!
	 * @brief bias_edp_force generator pojnter
	 */
	generator::bias_edp_force* befg;
public:

	/**
	 * @brief Constructor
	 * @param _ecp_t ecp task object reference.
	 */
	sub_task_bias_edp_force(task &_ecp_t);

	void conditional_execution();
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp


#endif
