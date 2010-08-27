#if !defined(MP_R_SMB_H_)
#define MP_R_SMB_H_

/*!
 * @file mp_r_smb.h
 * @brief File contains mp robot class declaration for SwarmItFix Mobile Base
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup smb
 */

#include "base/mp/mp_r_motor_driven.h"

namespace mrrocpp {
namespace mp {
namespace robot {
class smb : public motor_driven
{
public:
	smb(task::task &mp_object_l);
};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_SMB_H_*/
