#if !defined(_ECP_T_HSWITALS_GENERATORE_H)
#define _ECP_T_HSWITALS_GENERATORE_H

#include "base/ecp/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class hswitals_generatore : public common::task::task
{
public:
    hswitals_generatore(lib::configurator &_config);
};

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
