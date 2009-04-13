// -------------------------------------------------------------------------
//                            ecp_t_legobrick_conv.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP tasmociagu
// 
// Ostatnia modyfikacja: 2008.04.02
// autor: jdembek 
// -------------------------------------------------------------------------

#if !defined(_ECP_T_CONV_LEGOBRICK_H)
#define _ECP_T_CONV_LEGOBRICK_H

#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "ecp/common/ecp_task.h"
#include "ecp/conveyor/ecp_local.h"

#include "ecp/common/ecp_g_smooth.h"

namespace mrrocpp {
namespace ecp {
namespace conveyor {
namespace task {

class lego_brick: public common::task::base 
{
	double absolute_position;

public:
	// KONSTRUKTORY
	lego_brick(lib::configurator &_config);
	~lego_brick();
	
	// methods for ECP template to redefine in concrete classes
	void task_initialization(void);
	void main_task_algorithm(void);
	
};

}
} // namespace conveyor
} // namespace ecp
} // namespace mrrocpp

#endif
