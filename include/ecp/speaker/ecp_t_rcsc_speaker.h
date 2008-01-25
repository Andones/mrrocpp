// -------------------------------------------------------------------------
//                            ecp.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP
// 
// Modyfikacje:
// 1. metody wirtualne w klasie bazowej sensor - ok. 160
// 2. bonusy do testowania
// 
// Ostatnia modyfikacja: 25.06.2003
// autor modyfikacji: tkornuta
// -------------------------------------------------------------------------

#if !defined(_ECP_T_RCSC_SPEAKER_H)
#define _ECP_T_RCSC_SPEAKER_H

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_generator_t.h"
#include "ecp/speaker/ecp_g_speak.h"

class ecp_task_rcsc_speaker: public ecp_task  {
protected:
	ecp_generator_t* gt;
	speaking_generator* speak;

public:
	// KONSTRUKTORY
	ecp_task_rcsc_speaker();
	~ecp_task_rcsc_speaker();
	
	// methods for ECP template to redefine in concrete classes
	void task_initialization(void);
	void main_task_algorithm(void);
	
};

#endif
