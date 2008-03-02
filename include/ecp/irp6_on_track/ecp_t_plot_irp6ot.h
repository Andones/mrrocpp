#if !defined(_ECP_T_PLOT_IRP6OT_H)
#define _ECP_T_PLOT_IRP6OT_H

#include "ecp/common/ecp_task.h"

class ecp_task_plot_irp6ot: public ecp_task  {

public:
	// KONSTRUKTORY
	ecp_task_plot_irp6ot(configurator &_config);
	~ecp_task_plot_irp6ot();
	
	// methods for ECP template to redefine in concrete classes
	void task_initialization(void);
	void main_task_algorithm(void);
};

#endif
