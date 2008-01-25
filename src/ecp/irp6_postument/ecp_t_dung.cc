// ------------------------------------------------------------------------
//   ecp_t_dung.cc - przezroczyste wersja dla dowolnego z robotow 
// 
//                     EFFECTOR CONTROL PROCESS (ECP) - main()
// 
// Ostatnia modyfikacja: 2007
// ------------------------------------------------------------------------


#include <stdio.h>
#include <unistd.h>
#include <map>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"	

#include "ecp/irp6_postument/ecp_local.h"
#include "ecp/irp6_postument/ecp_t_dung.h"
#include "ecp/irp6_postument/ecp_g_dung.h"

// KONSTRUKTORY
ecp_task_dung::ecp_task_dung() : ecp_task()
{

};

ecp_task_dung::~ecp_task_dung(){};


// methods for ECP template to redefine in concrete classes
void ecp_task_dung::task_initialization(void) 
{
	ecp_m_robot = new ecp_irp6_postument_robot (*this);
	
	

	usleep(1000*100);
	
	sr_ecp_msg->message("ECP loaded");
};


void ecp_task_dung::main_task_algorithm(void)
{
	sr_ecp_msg->message("ECP DUNG - press start");
	ecp_wait_for_start();

	dung_generator dg(*this, 4);

	for(;;) { // Wewnetrzna petla nieskonczona
		
		for(;;) {
			sr_ecp_msg->message("NEW SERIES");

			Move ( dg);
		
		}
		
		// Oczekiwanie na STOP
		printf("przed wait for stop\n");
		ecp_wait_for_stop ();
		break;
	} // koniec: for(;;) wewnetrznej

};

ecp_task* return_created_ecp_task (void)
{
	return new ecp_task_dung();
};
