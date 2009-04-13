//Zadanie kt�re realizuje dan� trajektori� u�ywaj�c smooth2 generatora

#if !defined(_ECP_T_SMOOTH2_TEST_H)
#define _ECP_T_SMOOTH2_TEST_H

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_g_smooth2.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

class smooth2_test: public common::task::base {
  
  protected:
	  common::generator::smooth2* smoothgen2;
		
	public:
		smooth2_test(lib::configurator &_config);
		~smooth2_test();

		void task_initialization(void);
		void main_task_algorithm(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif

