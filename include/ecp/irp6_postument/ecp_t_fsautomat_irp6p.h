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

class ecp_task_fsautomat_irp6p: public ecp_task
{
	protected:
		ecp_smooth_generator* sg;
		ecp_sub_task_gripper_opening* go_st;
//		struct str_cmp{
//			bool operator()(char const *a, char const *b) const;
//		};		
		std::map<char*, Trajectory, str_cmp>* trjMap;

	public:
		// KONSTRUKTORY
		ecp_task_fsautomat_irp6p(configurator &_config);
		~ecp_task_fsautomat_irp6p();
	
		// methods for ECP template to redefine in concrete classes
		void task_initialization(void);
		void main_task_algorithm(void);
//		bool loadTrajectories(char * fileName);
	
};

#endif
