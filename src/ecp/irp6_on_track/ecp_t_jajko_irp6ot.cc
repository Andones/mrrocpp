// ------------------------------------------------------------------------
//   ecp_t_tran.cc - przezroczyste wersja dla dowolnego z robotow 
// 
//                     EFFECTOR CONTROL PROCESS (ECP) - main()
// 
// Ostatnia modyfikacja: 2006
// ------------------------------------------------------------------------

#include <unistd.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"	

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/common/ecp_g_force.h"
#include "ecp_mp/ecp_mp_s_schunk.h"
#include "ecp/irp6_on_track/ecp_t_jajko_irp6ot.h"



// KONSTRUKTORY
ecp_task_jajko_irp6ot::ecp_task_jajko_irp6ot() : ecp_task()
{
	yefg = NULL;
};

ecp_task_jajko_irp6ot::~ecp_task_jajko_irp6ot(){};


// methods for ECP template to redefine in concrete classes
void ecp_task_jajko_irp6ot::task_initialization(void) 
{
	ecp_m_robot = new ecp_irp6_on_track_robot (*this);
	
	// Powolanie czujnikow
	sensor_m[SENSOR_FORCE_ON_TRACK] = 
		new ecp_mp_schunk_sensor (SENSOR_FORCE_ON_TRACK, "[vsp_force_irp6ot]", *this);
				
	// Konfiguracja wszystkich czujnikow	
	for (std::map <SENSOR_ENUM, sensor*>::iterator sensor_m_iterator = sensor_m.begin();
		 sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
	{
		sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
		sensor_m_iterator->second->configure_sensor();
	}
	
	usleep(1000*100);
	sr_ecp_msg->message("ECP jajeczne irp6ot loaded");
};


void ecp_task_jajko_irp6ot::main_task_algorithm(void)
{
	sr_ecp_msg->message("ECP  jajeczne - wcisnij start");
	ecp_wait_for_start();
	for(;;) { // Wewnetrzna petla nieskonczona

		for(;;)
		{

			BYTE tryb;

			// Wybor trybu zadania - rozbijanie jajka badz nie
			tryb = choose_option ("1 - Egg destroying, 2 - Egg is safe", 2);
			if ( tryb == OPTION_ONE )
			{
				tryb=0;
			} else if ( tryb == OPTION_TWO ) 
			{
				tryb=1;
			} else if ( tryb == QUIT) 
			{
				break;
			}

			if (yefg!=NULL) delete yefg;
			yefg = new y_egg_force_generator (*this, 8, tryb);
			yefg->sensor_m = sensor_m;
		
			sr_ecp_msg->message("NOWA SERIA");
			Move ( *yefg);
		}
		
		// Oczekiwanie na STOP
		ecp_wait_for_stop();
		break;
	} // koniec: for(;;) wewnetrznej

};

ecp_task* return_created_ecp_task (void)
{
	return new ecp_task_jajko_irp6ot();
};
