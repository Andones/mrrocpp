#if !defined(_ECP_T_TFG_H)
#define _ECP_T_TFG_H

#include "base/ecp/ecp_task.h"

#include "ecp_g_tfg.h"
#include "generator/ecp/ecp_g_constant_velocity.h"

namespace mrrocpp {
namespace ecp {
namespace irp6_tfg {
namespace task {

class tfg : public common::task::task
{
protected:
	//generatory

	bool save_activated;
	common::generator::constant_velocity* cvg;

public:
	// KONSTRUKTORY
	tfg(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void mp_2_ecp_next_state_string_handler(void);
};

}
} // namespace irp6_tfg
} // namespace ecp
} // namespace mrrocpp

#endif
