// ------------------------------------------------------------------------
// transformation thread by Y 
// ostatnia modyfikacja: styczen 2005
// ------------------------------------------------------------------------

/********************************* INCLUDES *********************************/
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <stdlib.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"
#include "lib/srlib.h"
#include "lib/mis_fun.h"

// Klasa edp_speaker_effector.
#include "edp/speaker/edp_speaker_effector.h"


/********************************* GLOBALS **********************************/



extern edp_speaker_effector *master;  // by Y
extern master_trans_t_buffer mt_tt_obj;

/********************************** SIGCATCH ********************************/
void catch_signal_speak_t(int sig)
{
	switch(sig) {
		case SIGTERM :
			exit(EXIT_SUCCESS);
		break;
	} // end: switch
}

void *speak_t_thread(void *arg)
{
	set_thread_priority(pthread_self() , MAX_PRIORITY-10);
	
	if( master->init() == -1)
	{
		master->initialize_incorrect=1;
	}
	else
	{
		master->initialize_incorrect=0;
	}
	
	while(1)
	{
		mt_tt_obj.trans_t_wait_for_master_order();// oczekiwanie na zezwolenie ruchu od edp_master
		
		// przekopiowanie instrukcji z bufora watku komunikacji z ECP (edp_master)
		memcpy( &(master->current_instruction), &(master->new_instruction), sizeof(master->new_instruction) ); 
		try
		{
		
			switch (mt_tt_obj.trans_t_task)
			{
				case MT_GET_ARM_POSITION:
					// master.get_arm_position(mt_tt_obj.trans_t_tryb, &(master.current_instruction));
					master->get_spoken(mt_tt_obj.trans_t_tryb, &(master->current_instruction)); // MAC7
					mt_tt_obj.trans_t_to_master_order_status_ready();
				break;
				case MT_MOVE_ARM:
					// master.move_arm(&(master.current_instruction)); 	 // wariant dla watku edp_trans_t
					mt_tt_obj.trans_t_to_master_order_status_ready();
					master->speak(&(master->current_instruction)); // MAC7
				break;
				default: // blad: z reply_type wynika, e odpowied nie ma zawiera narzedzia
				break;
			}; // end: switch (reply_type)
		
		}
		
		// sekcja przechwytujaca bledy i przygotowujaca do ich rzucania w watku master
		
		
		catch(transformer_error::NonFatal_error_1 nfe) {
			mt_tt_obj.error_pointer= malloc(sizeof(nfe));
			memcpy(mt_tt_obj.error_pointer, &nfe, sizeof(nfe));
			mt_tt_obj.error_pointer = &nfe;
			mt_tt_obj.error = NonFatal_erroR_1;
			mt_tt_obj.trans_t_to_master_order_status_ready();
		} // end: catch(transformer_error::NonFatal_error_1 nfe)
		
		catch(transformer_error::NonFatal_error_2 nfe) {
			mt_tt_obj.error_pointer= malloc(sizeof(nfe));
			memcpy(mt_tt_obj.error_pointer, &nfe, sizeof(nfe));
			mt_tt_obj.error_pointer=&nfe;
			mt_tt_obj.error = NonFatal_erroR_2;
			mt_tt_obj.trans_t_to_master_order_status_ready();
		} // end: catch(transformer_error::NonFatal_error_2 nfe)
		
		catch(transformer_error::NonFatal_error_3 nfe) {
			mt_tt_obj.error_pointer= malloc(sizeof(nfe));
			memcpy(mt_tt_obj.error_pointer, &nfe, sizeof(nfe));
			mt_tt_obj.error_pointer=&nfe;
			mt_tt_obj.error = NonFatal_erroR_3;
			mt_tt_obj.trans_t_to_master_order_status_ready();
		} // end: catch(transformer_error::NonFatal_error_3 nfe)
		
		catch(transformer_error::NonFatal_error_4 nfe) {
			mt_tt_obj.error_pointer= malloc(sizeof(nfe));
			memcpy(mt_tt_obj.error_pointer, &nfe, sizeof(nfe));
			mt_tt_obj.error = NonFatal_erroR_4;
			mt_tt_obj.trans_t_to_master_order_status_ready();
		} // end: catch(transformer_error::NonFatal_error nfe4)
		
		catch(transformer_error::Fatal_error fe) {
			mt_tt_obj.error_pointer= malloc(sizeof(fe));
			memcpy(mt_tt_obj.error_pointer, &fe, sizeof(fe));
			mt_tt_obj.error = Fatal_erroR;
			mt_tt_obj.trans_t_to_master_order_status_ready();
		} // end: catch(transformer_error::Fatal_error fe)
				
		catch (System_error fe) {
			mt_tt_obj.error_pointer= malloc(sizeof(fe));
			memcpy(mt_tt_obj.error_pointer, &fe, sizeof(fe));
			mt_tt_obj.error = System_erroR;
			mt_tt_obj.trans_t_to_master_order_status_ready();
		} // end: catch(System_error fe)
		
		catch (...) {  // Dla zewnetrznej petli try
			printf("transformation thread uneidentified_error\n");
		
			mt_tt_obj.trans_t_to_master_order_status_ready();
			// Wylapywanie niezdefiniowanych bledow
			// printf("zlapane cos");// by Y&W
		}
		
	} // end while
	return NULL;
}

