// -------------------------------------------------------------------------
//                            ecp.h dla QNX6
// Deklaracje struktur danych i metod dla procesow ECP 
// 
// -------------------------------------------------------------------------


#if !defined(_ECP_GEN_PP_H)
#define _ECP_GEN_PP_H

#include "common/impconst.h"
#include "common/com_buf.h"
#include "lib/mathtr.h"
#include "ecp/common/ecp_generator.h"


namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

// --------------------------------------------------------------------------
// Generator trajektorii prostoliniowej 
class progpanel : public common::generator::base 
{

protected:

	long run_counter;
	bool second_step;
    double start_joint_arm_coordinates[MAX_SERVOS_NR];
	
public:
	trajectory_description td;
	int step_no;
	
	// konstruktor
	progpanel(common::task::base& _ecp_task, int step=0);  
	
	virtual bool first_step ();
	virtual bool next_step ();

}; // end:
// --------------------------------------------------------------------------

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp


#endif
