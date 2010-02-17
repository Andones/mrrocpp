// -------------------------------------------------------------------------
//                            impconst.h
// Typy i stale wykorzystywane w MRROC++
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#if !defined(_IRP6OT_TFG_CONST_H)
#define _IRP6OT_TFG_CONST_H

#include <stdint.h>

namespace mrrocpp {

}

using namespace mrrocpp;

namespace mrrocpp {
namespace lib {

#ifdef __cplusplus
extern "C" {
#endif

// do podmianki
/*
 #define EDP_IRP6OT_TFG_SECTION "[edp_irp6ot_tfg]"
 #define ECP_IRP6OT_TFG_SECTION "[ecp_irp6ot_tfg]"

 #define IRP6OT_TFG_NUM_OF_SERVOS	1
 */

#define EDP_IRP6_ON_TRACK_SECTION "[edp_irp6_on_track]"
#define ECP_IRP6_ON_TRACK_SECTION "[ecp_irp6_on_track]"

#define IRP6OT_TFG_NUM_OF_SERVOS	1

#ifdef __cplusplus
}
#endif

} // namespace lib
} // namespace mrrocpp

#endif /* _IMPCONST_H */
