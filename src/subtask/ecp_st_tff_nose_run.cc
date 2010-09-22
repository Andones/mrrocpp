/*!
 * @file
 * @brief File contains sub_task_tff_nose_run definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup subtasks
 */

#include "base/lib/typedefs.h"
#include "base/lib/sr/srlib.h"

#include "subtask/ecp_st_tff_nose_run.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

sub_task_tff_nose_run::sub_task_tff_nose_run(task &_ecp_t) :
	sub_task(_ecp_t)
{
	nrg = new generator::tff_nose_run(_ecp_t, 8);
}

void sub_task_tff_nose_run::conditional_execution()
{

	nrg->Move();
}

} // namespace task

} // namespace common
} // namespace ecp
} // namespace mrrocpp
