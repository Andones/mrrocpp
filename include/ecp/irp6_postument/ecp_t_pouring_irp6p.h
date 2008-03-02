#if !defined(_ECP_T_POURING_IRP6P_H)
#define _ECP_T_POURING_IRP6P_H

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_generator_t.h"

class ecp_task_pouring_irp6p: public ecp_task  {
protected:
	ecp_smooth_generator* sg;

public:
	// KONSTRUKTORY
	ecp_task_pouring_irp6p(configurator &_config);
	~ecp_task_pouring_irp6p();
	
	// methods for ECP template to redefine in concrete classes
	void task_initialization(void);
	void main_task_algorithm(void);
	
};

#endif
