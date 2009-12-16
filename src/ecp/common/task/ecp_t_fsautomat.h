// -------------------------------------------------------------------------
//                            task/ecp_t_fsautomat.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP
//
// Ostatnia modyfikacja:	sierpien 2008
// Autor:						Marek Kisiel
// -------------------------------------------------------------------------

#if !defined(_ECP_TASK_FSAUTOMAT_H)
#define _ECP_TASK_FSAUTOMAT_H

#include "ecp/common/task/ecp_task.h"
#include "ecp/common/task/ecp_st_go.h"
#include "ecp/common/generator/ecp_g_t.h"

#include "ecp_mp/Trajectory.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

//void ecp_gripper_opening (task& ecp_object, double gripper_increment, int motion_time);

class fsautomat: public task
{
	protected:
		// generatory
		common::generator::smooth* sg;
		common::generator::tool_change* tcg;
		common::generator::transparent* gt;
		common::generator::tff_nose_run* nrg;
		common::generator::tff_rubik_grab* rgg;
		common::generator::tff_gripper_approach* gag;
		common::generator::tff_rubik_face_rotate* rfrg;
		common::teach_in* tig;
		common::generator::bias_edp_force* befg;
		common::generator::weight_meassure* wmg;
		//podzadania
		common::task::ecp_sub_task_gripper_opening* go_st;

		trajectories_t * trjMap;

	public:
		// KONSTRUKTORY
		fsautomat(lib::configurator &_config);

		// methods for ECP template to redefine in concrete classes
		void main_task_algorithm(void);
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
