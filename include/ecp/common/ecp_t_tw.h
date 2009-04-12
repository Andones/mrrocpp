#if !defined(_ECP_T_TW_H)
#define _ECP_T_TW_H

#include "ecp/common/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace common {

class ecp_task_tw: public common::ecp_task  {
protected:
	ecp_tff_nose_run_generator* nrg;
	y_edge_follow_force_generator* yefg;
	bias_edp_force_generator* befg;
	bool save_activated;

public:
	// KONSTRUKTORY
	ecp_task_tw(configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void task_initialization(void);
	void main_task_algorithm(void);
};

} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
