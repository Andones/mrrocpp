#include <cstdio>
#include <unistd.h>
#include "base/mp/mp_task.h"
#include "base/mp/mp_robot.h"
#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/srlib.h"

#include "base/mp/MP_main_error.h"
#include "base/mp/mp_t_c.h"
#include "base/mp/mp_g_common.h"

#include <boost/foreach.hpp>
namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config)
{
	return new cxx(_config);
}

cxx::cxx(lib::configurator &_config) :
	task(_config)
{
}

void cxx::main_task_algorithm(void)
{

	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
	{
		robot_node.second->ecp_reply_package.reply = lib::ECP_ACKNOWLEDGE;

	}

	generator::extended_empty empty_gen(*this); // "Pusty" generator
	empty_gen.robot_m = robot_m;

	// Zlecenie wykonania kolejnego makrokroku
	empty_gen.Move();
}

} // namespace task
} // namespace mp
} // namespace mrrocpp


