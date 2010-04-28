#if !defined(_SPKM_CONST_H)
#define _SPKM_CONST_H

#include "lib/swarmitfix.h"

namespace mrrocpp {

}

using namespace mrrocpp;

namespace mrrocpp {
namespace lib {


struct spkm_cbuffer {
	double em[6];
	double emdm[6];
	double aa[6];
	double da[6];
	double av[6];
	double tt;
	lib::EPOS_GEN_PROFILE profile_type;
};

struct spkm_rbuffer {
	double position[6];
	bool motion_in_progress[6];
	bool contact;
};

#define EDP_SPKM_SECTION "[edp_spkm]"
#define ECP_SPKM_SECTION "[ecp_spkm]"

#define SPKM_NUM_OF_SERVOS	6



} // namespace lib
} // namespace mrrocpp

#endif /* _SPKM_CONST_H */
