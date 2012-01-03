/*!
 * @file
 * @brief File contains dp_shead class definition for SwarmItFix head
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup shead
 */

#include <cmath>
#include <cstring>
#include <ostream>

#include "dp_shead.h"

namespace mrrocpp {
namespace lib {
namespace shead {

reply::reply() :
		solidification_state(SOLIDIFICATION_STATE_INTERMEDIATE), vacuum_state(VACUUM_STATE_INTERMEDIATE)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp

