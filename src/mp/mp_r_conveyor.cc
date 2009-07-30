#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/mis_fun.h"
#include "lib/srlib.h"
#include "mp/mp.h"
#include "mp/mp_r_conveyor.h"

namespace mrrocpp {
namespace mp {
namespace robot {

conveyor::conveyor(task::task &mp_object_l) :
	manip_and_conv(lib::ROBOT_CONVEYOR, "[ecp_conveyor]", mp_object_l)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp

