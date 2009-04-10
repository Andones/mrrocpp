#if !defined(MP_R_IRP6_MECHATRONIKA_H_)
#define MP_R_IRP6_MECHATRONIKA_H_


#include "mp/mp_r_irp6s_and_conv.h"
namespace mrrocpp {
namespace mp {
namespace common {
class irp6_mechatronika_robot: public irp6s_and_conv_robot {

 public:
  irp6_mechatronika_robot (task::base &mp_object_l);
};
} // namespace common
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_IRP6_MECHATRONIKA_H_*/
