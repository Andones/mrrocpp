// -------------------------------------------------------------------------
//                            impconst.h
// Typy i stale wykorzystywane w MRROC++
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#if !defined(_IRP6P_M_CONST_H)
#define _IRP6P_M_CONST_H

#include <stdint.h>

namespace mrrocpp {

}

using namespace mrrocpp;

namespace mrrocpp {
namespace lib {

#ifdef __cplusplus
extern "C" {
#endif

// Do podmianki
/*
 #define EDP_IRP6P_M_SECTION "[edp_irp6p_m]"
 #define ECP_IRP6P_M_SECTION "[ecp_irp6p_m]"

 #define IRP6P_M_NUM_OF_SERVOS	6
 */

#define EDP_IRP6_POSTUMENT_SECTION "[edp_irp6_postument]"
#define ECP_IRP6_POSTUMENT_SECTION "[ecp_irp6_postument]"

#define IRP6_POSTUMENT_NUM_OF_SERVOS	7

#ifdef __cplusplus
}
#endif

} // namespace lib
} // namespace mrrocpp

#endif /* _IMPCONST_H */
