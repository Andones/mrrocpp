#if !defined(_IRP6P_TFG_CONST_H)
#define _IRP6P_TFG_CONST_H

/*!
 * @file
 * @brief File contains constants and structures for IRp6 postument two finger gripper
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup irp6p_tfg
 */

#include "base/lib/impconst.h"

namespace mrrocpp {
namespace lib {

#define IRP6_POSTUMENT_AXIS_7_INC_PER_REVOLUTION  128.0  // Liczba impulsow enkodera na obrot walu - musi byc float
const robot_name_t ROBOT_IRP6P_TFG = "ROBOT_IRP6P_TFG";
#define EDP_IRP6P_TFG_SECTION "[edp_irp6p_tfg]"
#define ECP_IRP6P_TFG_SECTION "[ecp_irp6p_tfg]"

#define IRP6P_TFG_NUM_OF_SERVOS	1

} // namespace lib
} // namespace mrrocpp

#endif /* _IRP6P_TFG_CONST_H */
