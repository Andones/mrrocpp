#if !defined(_ECP_TAUGHT_IN_POSE_H)
#define  _ECP_TAUGHT_IN_POSE_H

#include "lib/com_buf.h"		// POSE_SPECIFICATION
#include "lib/impconst.h"	// MAX_SERVOS_NR

namespace mrrocpp {
namespace ecp {
namespace common {

class ecp_taught_in_pose {
public:
  POSE_SPECIFICATION arm_type;
  double motion_time;
  double coordinates[MAX_SERVOS_NR];
  
  int extra_info; // by Y uzupelnienie struktury o dodatkowe pole, do dowolnego wykorzystania
  
  ecp_taught_in_pose (void);

  ecp_taught_in_pose (POSE_SPECIFICATION at, double mt, double* c);

  ecp_taught_in_pose (POSE_SPECIFICATION at, double mt, int e_info, double* c); 
}; // end:class ecp_taught_in_pose

} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_TAUGHT_IN_POSE_H */
