// -------------------------------------------------------------------------
//                            ecp.h dla QNX6
// Realizacja automatu skonczonego - ECP dla IRP6_POSTUMENT
// Ostatnia modyfikacja: 	2008
// autor: 						Marek Kisiel
// -------------------------------------------------------------------------

#if !defined(_ECP_T_FSAUTOMAT_IRP6P_H)
#define _ECP_T_FSAUTOMAT_IRP6P_H

#include <map>

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_st_go.h"
#include "ecp/common/ecp_generator_t.h"

#include "mp/Trajectory.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p {
namespace task {

class fsautomat: public common::task::task
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
		common::ecp_teach_in_generator* tig;
		common::generator::bias_edp_force* befg;
		common::generator::weight_meassure* wmg;
		//podzadania
		common::task::ecp_sub_task_gripper_opening* go_st;

		std::map<char*, mp::common::Trajectory, str_cmp>* trjMap;

	public:
		// KONSTRUKTORY
		fsautomat(lib::configurator &_config);

		// methods for ECP template to redefine in concrete classes
		void task_initialization(void);
		void main_task_algorithm(void);
//		bool loadTrajectories(char * fileName);

};

}
} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp

#endif
