#include <unistd.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/common/ecp_g_space.h"
#include "ecp/irp6_on_track/ecp_t_kbochnia_irp6ot.h"

namespace mrrocpp {
namespace ecp {
namespace common {

// KONSTRUKTORY
ecp_task_kbochnia_irp6ot::ecp_task_kbochnia_irp6ot(configurator &_config) : ecp_task(_config)
{
}


// methods for ECP template to redefine in concrete classes
void ecp_task_kbochnia_irp6ot::task_initialization(void)
{
    ecp_m_robot = new ecp_irp6_on_track_robot (*this);

    sr_ecp_msg->message("ECP loaded");
}

void ecp_task_kbochnia_irp6ot::main_task_algorithm(void)
{
	irp6ot_natural_spline_generator	spline_gen(*this, 0.02,1);

	spline_gen.load_file_from_ui();
	sr_ecp_msg->message("Zaladowano plik");

	spline_gen.Move();

	sr_ecp_msg->message("Motion is finished");

	ecp_termination_notice ();
}

ecp_task* return_created_ecp_task (configurator &_config)
{
	return new ecp_task_kbochnia_irp6ot(_config);
}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


