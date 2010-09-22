#if !defined(_ECP_ST_TFF_NOSE_RUN_H)
#define _ECP_ST_TFF_NOSE_RUN_H

/*!
 * @file
 * @brief File contains ecp_sub_task_tff_nose_run declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup subtasks
 */

#include "base/ecp/ecp_task.h"
#include "subtask/ecp_mp_st_tff_nose_run.h"
#include "generator/ecp/force/ecp_g_tff_nose_run.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
class tff_nose_run;
}

namespace task {

/*!
 * @brief subtask to execute tff_nose_run generator
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup subtasks
 */
class ecp_sub_task_tff_nose_run : public ecp_sub_task
{

private:

public:

	/*!
	 * @brief tff_nose_run generator pojnter
	 */
	generator::tff_nose_run* nrg;

	/**
	 * @brief Constructor
	 * @param _ecp_t ecp task object reference.
	 */
	ecp_sub_task_tff_nose_run(task &_ecp_t);

	void conditional_execution();
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp


#endif
