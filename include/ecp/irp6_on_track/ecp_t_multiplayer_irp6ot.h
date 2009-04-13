#if !defined(_ECP_T_MULTIPLAYER_IRP6OT_H)
#define _ECP_T_MULTIPLAYER_IRP6OT_H

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_st_go.h"
#include "ecp/common/ecp_generator_t.h"
#include "ecp/irp6_on_track/ecp_g_vis_sac_lx.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

class ecp_task_multiplayer_irp6ot : public common::task::ecp_task
{
private:
	//generatory
	common::generator::ecp_generator_t* gt;
	common::generator::ecp_smooth_generator* sg;
	common::generator::bias_edp_force_generator* befg;
	common::generator::weight_meassure_generator* wmg;

    //podzadania
	common::task::ecp_sub_task_gripper_opening* go_st;

	generator::ecp_vis_sac_lx_generator *takeg;
   	common::generator::ecp_tff_rubik_grab_generator *rgg;


public:
    ecp_task_multiplayer_irp6ot(configurator &_config);

    // methods for ECP template to redefine in concrete classes
    void task_initialization(void);
    void main_task_algorithm(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
