#if !defined(_ECP_T_FESTIVAL_H)
#define _ECP_T_FESTIVAL_H

#include "ecp/common/ecp_task.h"
#include "ecp/festival/ecp_g_festival.h"

class ecp_task_festival: public ecp_task  {
protected:
	festival_generator* fg;

public:
	// KONSTRUKTORY
	ecp_task_festival();
	~ecp_task_festival();
	
	// methods for ECP template to redefine in concrete classes
	void task_initialization(void);
	void main_task_algorithm(void);
	
};

#endif
