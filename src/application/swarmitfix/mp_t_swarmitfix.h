#if !defined(__MP_T_SWARMITFIX_H)
#define __MP_T_SWARMITFIX_H

#include <boost/unordered_map.hpp>
#include <boost/ptr_container/ptr_unordered_map.hpp>
#include <boost/ptr_container/ptr_vector.hpp>

#include "base/lib/impconst.h"
#include "base/lib/agent/InputBufferBase.h"

namespace mrrocpp {
namespace mp {
namespace task {

/** @defgroup swarmitfix swarmitfix
 *  @ingroup application
 *  A swarmitfix QNX test application
 *  @{
 */

class swarmitfix : public task
{
private:
	//! Type for plan realization status
	typedef enum _PLAN_STATUS { ONGOING, FAILURE } PLAN_STATUS;

	//! Type for worker (ECP) agent status
	typedef enum _WORKER_STATUS { IDLE, BUSY } WORKER_STATUS;

	//! Current plan status
	PLAN_STATUS current_plan_status;

	//! Current workers status
	boost::unordered_map<lib::robot_name_t, WORKER_STATUS> current_workers_status;

	//! Input and output buffer container
	boost::ptr_unordered_map<const lib::robot_name_t, InputBufferBase> IOBuffers;
	boost::ptr_vector<InputBufferBase> IOBuffersv;

public:
	swarmitfix(lib::configurator &_config);

	/// utworzenie robotow
	void create_robots(void);

	// methods for mp template
	void main_task_algorithm(void);
};

/** @} */// end of swarmitfix

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
