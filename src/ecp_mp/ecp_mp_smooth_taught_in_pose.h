#if !defined(_ECP_MP_SMOOTH_TAUGHT_IN_POSE_H)
#define  _ECP_MP_SMOOTH_TAUGHT_IN_POSE_H

#include "lib/com_buf.h"		// lib::POSE_SPECIFICATION
#include "lib/impconst.h"	// MAX_SERVOS_NR

namespace mrrocpp {
namespace ecp_mp {
namespace common {

class smooth_taught_in_pose {
public:
  lib::POSE_SPECIFICATION arm_type;
  double v_p[MAX_SERVOS_NR];
  double v_k[MAX_SERVOS_NR];
  double v[MAX_SERVOS_NR];
  double a[MAX_SERVOS_NR];
  double coordinates[MAX_SERVOS_NR];

  smooth_taught_in_pose (void);
  smooth_taught_in_pose (lib::POSE_SPECIFICATION at,
		  const double* vp,
		  const double* vk,
		  const double* vv,
		  const double* aa,
		  const double* coordinates);
}; // end:class ecp_smooth_taught_in_pose

} // namespace common
} // namespace ecp_mp
} // namespace mrrocpp

#endif /* _ECP_SMOOTH_TAUGHT_IN_POSE_H */
