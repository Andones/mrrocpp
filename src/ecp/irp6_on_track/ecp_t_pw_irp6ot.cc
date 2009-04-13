#include <stdio.h>

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_t_pw_irp6ot.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

//Konstruktory
ecp_task_pw_irp6ot::ecp_task_pw_irp6ot(configurator &_config) : base(_config)
{}

void ecp_task_pw_irp6ot::task_initialization(void)
{
	ecp_m_robot= new ecp_irp6_on_track_robot(*this);
	kolo_gen= new generator::ecp_g_pw_kolo(*this);

	sr_ecp_msg->message("ECP PW loaded");
}


void ecp_task_pw_irp6ot::main_task_algorithm(void)
{
	sr_ecp_msg->message("Przed MOVE");
	kolo_gen->Move();
	sr_ecp_msg->message("Po MOVE");

	sr_ecp_msg->message("przed wait_for_stop\n");
	ecp_termination_notice();
}

}
} // namespace irp6ot

namespace common {
namespace task {

base* return_created_ecp_task(configurator &_config)
{
	return new irp6ot::task::ecp_task_pw_irp6ot(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

