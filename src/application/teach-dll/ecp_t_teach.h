// -------------------------------------------------------------------------
//                            task/ecp_t_teach.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP
//
// Modyfikacje:
// 1. metody wirtualne w klasie bazowej sensor - ok. 160
// 2. bonusy do testowania
//
// Ostatnia modyfikacja: 25.06.2003
// autor modyfikacji: tkornuta
// -------------------------------------------------------------------------

#if !defined(_ECP_T_TEACH_H)
#define _ECP_T_TEACH_H

#include "ecp/common/task/ecp_task.h"
#include "ecp/common/generator/ecp_g_teach_in.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class teach: public common::task::task  {
protected:
	generator::teach_in* tig;

public:
	// KONSTRUKTORY
	teach(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void main_task_algorithm(void);

};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
