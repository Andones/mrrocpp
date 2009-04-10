#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/mis_fun.h"
#include "lib/srlib.h"
#include "mp/mp_r_irp6_mechatronika.h"

namespace mrrocpp {
namespace mp {
namespace common {

irp6_mechatronika_robot::irp6_mechatronika_robot (task::base &mp_object_l) :
		irp6s_and_conv_robot (ROBOT_IRP6_MECHATRONIKA, "[ecp_irp6_mechatronika]", mp_object_l)
{}

} // namespace common
} // namespace mp
} // namespace mrrocpp

