#if !defined(_ECP_T_POURING_IRP6P_H)
#define _ECP_T_POURING_IRP6P_H

#include "ecp/common/task/ecp_task.h"
#include "ecp/common/task/ecp_st_go.h"
#include "ecp/common/generator/ecp_g_t.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p {
namespace task {

class pouring: public common::task::task
{
protected:
	common::generator::smooth* sg;
    //podzadania
	common::task::ecp_sub_task_gripper_opening* go_st;

public:
    // KONSTRUKTORY
    pouring(lib::configurator &_config);

    // methods for ECP template to redefine in concrete classes
    void main_task_algorithm(void);
};

}
} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp

#endif
