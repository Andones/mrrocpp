// -------------------------------------------------------------------------
//                            mp_task_rc.h
// Definicje struktur danych i metod dla procesow MP - zadanie vision force
// 
// Ostatnia modyfikacja: 2007
// -------------------------------------------------------------------------

#if !defined(__MP_TASK_VIS_H)
#define __MP_TASK_VIS_H

#include "mp/mp.h"

class mp_task_vis: public mp_task  {
protected:

	bool break_state;
public:

	// methods for mp template
	void task_initialization(void);
	void main_task_algorithm(void);
  
};

#endif
