#if !defined(_SARKOFAG_CONST_H)
#define _SARKOFAG_CONST_H

#include "base/lib/impconst.h"

namespace mrrocpp {
namespace lib {
const robot_name_t ROBOT_SARKOFAG = "ROBOT_SARKOFAG";

#define SARKOFAG_INC_PER_REVOLUTION  4000.0  // Liczba impulsow enkodera na obrot walu - musi byc float
#define EDP_SARKOFAG_SECTION "[edp_sarkofag]"
#define ECP_SARKOFAG_SECTION "[ecp_sarkofag]"

#define SARKOFAG_NUM_OF_SERVOS	1

} // namespace lib
} // namespace mrrocpp

#endif /* _SARKOFAG_CONST_H */
