#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "edp_e_shead2.h"
#include "const_shead2.h"

namespace mrrocpp {
namespace edp {
namespace shead2 {

// Konstruktor.
effector::effector(common::shell &_shell) :
	shead::effector(_shell, lib::shead2::ROBOT_NAME)
{
	// FIXME: Set those values here to not bother with separate kinematics parameters class.
	homing_velocity = +100;
	homing_offset = -111500;
}

}// namespace shead2


namespace common {

effector* return_created_efector(common::shell &_shell)
{
	return new shead2::effector(_shell);
}

} // namespace common
} // namespace edp
} // namespace mrrocpp
