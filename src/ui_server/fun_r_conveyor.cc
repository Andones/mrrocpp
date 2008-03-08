/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

/* Standard headers */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <strings.h>
#include <semaphore.h>
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <sys/types.h>
#include <signal.h>
#include <sys/netmgr.h>
#include <errno.h>
#include <process.h>
#include <math.h>

#include "lib/srlib.h"
#include "ui/ui_const.h"
// #include "ui/ui.h"
// Konfigurator.
// #include "lib/configurator.h"
#include "ui/ui_ecp.h"


/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"





extern ui_msg_def ui_msg;
extern ui_ecp_buffer* ui_ecp_obj;

extern ui_state_def ui_state;
extern configurator* config;

extern ui_robot_def ui_robot;
extern ui_ecp_buffer* ui_ecp_obj;

double conveyor_current_pos[CONVEYOR_NUM_OF_SERVOS];// pozycja biezaca
double conveyor_desired_pos[CONVEYOR_NUM_OF_SERVOS]; // pozycja zadana


// zamykanie okien ruchow recznych dla robota irp6_on_track



// zamykanie okien ruchow recznych dla robota conveyor

int
close_wind_conveyor_moves( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui_state.is_wind_conveyor_moves_open)
	{
		PtDestroyWidget( ABW_wnd_conveyor_moves );
	}

	return( Pt_CONTINUE );

	}



int
start_wnd_conveyor_servo_algorithm( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (	!ui_state.is_wind_conv_servo_algorithm_open) // otworz okno
	{
		ApCreateModule (ABM_wnd_conveyor_servo_algorithm, widget, cbinfo);
		ui_state.is_wind_conv_servo_algorithm_open=1;
	} else { // przelacz na okno
		PtWindowToFront (ABW_wnd_conveyor_servo_algorithm);
	}

	return( Pt_CONTINUE );

	}


int
close_wnd_conveyor_servo_algorithm( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui_state.is_wind_conv_servo_algorithm_open)
	{
		PtDestroyWidget ( ABW_wnd_conveyor_servo_algorithm );
	}

	return( Pt_CONTINUE );

	}
	
	

int
start_wind_conveyor_moves( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	

	if (	!ui_state.is_wind_conveyor_moves_open) // otworz okno
	{
		ApCreateModule (ABM_wnd_conveyor_moves, widget, cbinfo);
		ui_state.is_wind_conveyor_moves_open=true;
	} else { // przelacz na okno
		PtWindowToFront (ABW_wnd_conveyor_moves);
	}

	return( Pt_CONTINUE );

	}


	
int
clear_wind_conveyor_moves_flag( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	ui_state.is_wind_conveyor_moves_open=false;
	return( Pt_CONTINUE );

	}	



int
clear_wnd_conveyor_servo_algorithm_flag( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	ui_state.is_wind_conv_servo_algorithm_open=false;

	return( Pt_CONTINUE );

	}





// dla robota conveyor

int
wind_conveyor_moves_init( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )
{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	try
	{
	if ((ui_state.conveyor.edp.pid!=-1)&&(ui_state.is_wind_conveyor_moves_open))
	{
		if ( ui_state.conveyor.edp.is_synchronised )  // Czy robot jest zsynchronizowany?
		{
			unblock_widget(ABW_PtNumericFloat_wind_conveyor_moves_inc_pos);
			unblock_widget(ABW_PtButton_wind_conveyor_moves_inc_exec);

			unblock_widget(ABW_PtButton_wind_conveyor_moves_int_left);
			unblock_widget(ABW_PtButton_wind_conveyor_moves_int_right);
			unblock_widget(ABW_PtNumericFloat_wind_conveyor_moves_int_step);
			unblock_widget(ABW_PtNumericFloat_wind_conveyor_moves_int_pos);
			unblock_widget(ABW_PtButton_wind_conveyor_moves_int_exec);

			if (!(ui_robot.conveyor->read_motors(conveyor_current_pos))) // Odczyt polozenia walow silnikow
				printf("Blad w read motors\n");
				
			PtSetResource(ABW_PtNumericFloat_wind_conveyor_moves_read_motor_pos, Pt_ARG_NUMERIC_VALUE, &conveyor_current_pos[0] , 0);
		
			if (!(ui_robot.conveyor->read_joints(conveyor_current_pos))) // Odczyt polozenia walow silnikow
				printf("Blad w read joints\n");
				
			PtSetResource(ABW_PtNumericFloat_wind_conveyor_moves_read_int_pos, Pt_ARG_NUMERIC_VALUE, &conveyor_current_pos[0] , 0);


		} else
		{
			block_widget(ABW_PtNumericFloat_wind_conveyor_moves_inc_pos);
			block_widget(ABW_PtButton_wind_conveyor_moves_inc_exec);
			
			block_widget(ABW_PtButton_wind_conveyor_moves_int_left);
			block_widget(ABW_PtButton_wind_conveyor_moves_int_right);
			block_widget(ABW_PtNumericFloat_wind_conveyor_moves_int_step);
			block_widget(ABW_PtNumericFloat_wind_conveyor_moves_int_pos);
			block_widget(ABW_PtButton_wind_conveyor_moves_int_exec);
		}
	PtDamageWidget( ABW_wnd_conveyor_moves );
	}
	} // end try
	CATCH_SECTION_UI
	
	return( Pt_CONTINUE );
}


int
conveyor_move_to_preset_position( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )
{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	PhKeyEvent_t *my_data = NULL;

	if (cbinfo->event->type==Ph_EV_KEY)
	{
		my_data = (PhKeyEvent_t *) PhGetData(cbinfo->event);
	}


	// wychwytania ew. bledow ECP::robot
	try
	{
	
	if (ui_state.conveyor.edp.pid!=-1)
	{
		
		 if ((((ApName(ApWidget(cbinfo)) == ABN_mm_conveyor_preset_position_synchro)||
			(ApName(ApWidget(cbinfo)) == ABN_mm_all_robots_preset_position_synchro))||
			((cbinfo->event->type==Ph_EV_KEY)&&(my_data->key_cap== 0x73 ))
			)&&(ui_state.conveyor.edp.is_synchronised)) {
			// powrot do pozycji synchronizacji
			 for (int i = 0; i < CONVEYOR_NUM_OF_SERVOS; i++)
			 {
	              conveyor_desired_pos[i] = 0.0;
	          }
	       
		} else  if ((((ApName(ApWidget(cbinfo)) == ABN_mm_conveyor_preset_position_0)||
			(ApName(ApWidget(cbinfo)) == ABN_mm_all_robots_preset_position_0))||
			((cbinfo->event->type==Ph_EV_KEY)&&(my_data->key_cap== 0x30 ))
			)&&(ui_state.conveyor.edp.is_synchronised)) {
			// ruch do pozycji zadania (wspolrzedne przyjete arbitralnie)
			for(int i = 0; i < CONVEYOR_NUM_OF_SERVOS; i++) {
				 conveyor_desired_pos[i] = ui_state.conveyor.edp.preset_position[0][i];
				}
		} else  if ((((ApName(ApWidget(cbinfo)) == ABN_mm_conveyor_preset_position_1)||
			(ApName(ApWidget(cbinfo)) == ABN_mm_all_robots_preset_position_1))||
			((cbinfo->event->type==Ph_EV_KEY)&&(my_data->key_cap== 0x31 ))
			)&&(ui_state.conveyor.edp.is_synchronised)) {
			// ruch do pozycji zadania (wspolrzedne przyjete arbitralnie)
			for(int i = 0; i < CONVEYOR_NUM_OF_SERVOS; i++) {
				 conveyor_desired_pos[i] = ui_state.conveyor.edp.preset_position[1][i];
			}
		} else  if ((((ApName(ApWidget(cbinfo)) == ABN_mm_conveyor_preset_position_2)||
			(ApName(ApWidget(cbinfo)) == ABN_mm_all_robots_preset_position_2))||
			((cbinfo->event->type==Ph_EV_KEY)&&(my_data->key_cap== 0x32 ))
			)&&(ui_state.conveyor.edp.is_synchronised)) {
			// ruch do pozycji zadania (wspolrzedne przyjete arbitralnie)
			for(int i = 0; i < CONVEYOR_NUM_OF_SERVOS; i++) {
				 conveyor_desired_pos[i] = ui_state.conveyor.edp.preset_position[2][i];
			}
		} 
				
		ui_robot.conveyor->move_motors(conveyor_desired_pos);
			
		}
			
	} // end try
	CATCH_SECTION_UI
	
	return( Pt_CONTINUE );
}




int
wind_conveyor_moves_move( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )
{

	double *wektor_ptgr, conveyor_desired_pos_motors[6], conveyor_desired_pos_int[6] ;
	double *step1;

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	try
	{
	
	if (ui_state.conveyor.edp.pid!=-1)
	{
	
		// incremental
		if ((widget == ABW_PtButton_wind_conveyor_moves_inc_left) ||
			(widget == ABW_PtButton_wind_conveyor_moves_inc_right) ||
			(widget == ABW_PtButton_wind_conveyor_moves_inc_exec))
		{
								
			 if (ui_state.conveyor.edp.is_synchronised)
			{
				PtGetResource(ABW_PtNumericFloat_wind_conveyor_moves_inc_pos, Pt_ARG_NUMERIC_VALUE, &wektor_ptgr, 0 );
				conveyor_desired_pos_motors[0] = (*wektor_ptgr);
			} else {
				conveyor_desired_pos_motors[0]=0.0;
			}
			
			PtGetResource(ABW_PtNumericFloat_wind_conveyor_moves_inc_step, Pt_ARG_NUMERIC_VALUE, &step1, 0 );
		
			if (widget == ABW_PtButton_wind_conveyor_moves_inc_left)
			{
				conveyor_desired_pos_motors[0]-=(*step1);
			} else
			if (widget == ABW_PtButton_wind_conveyor_moves_inc_right)
			{
				conveyor_desired_pos_motors[0]+=(*step1);
			}
					
			ui_robot.conveyor->move_motors(conveyor_desired_pos_motors);
			
		}
	
		// internal
		if ((widget == ABW_PtButton_wind_conveyor_moves_int_left) ||
			(widget == ABW_PtButton_wind_conveyor_moves_int_right) ||
			(widget == ABW_PtButton_wind_conveyor_moves_int_exec))
		{
			 if (ui_state.conveyor.edp.is_synchronised)
			{
				PtGetResource(ABW_PtNumericFloat_wind_conveyor_moves_int_pos, Pt_ARG_NUMERIC_VALUE, &wektor_ptgr, 0 );
				conveyor_desired_pos_int[0] = (*wektor_ptgr);
			} 

			PtGetResource(ABW_PtNumericFloat_wind_conveyor_moves_int_step, Pt_ARG_NUMERIC_VALUE, &step1, 0 );				
			
			if (widget == ABW_PtButton_wind_conveyor_moves_int_left)
			{
				conveyor_desired_pos_int[0]-=(*step1);
			} else
			if (widget == ABW_PtButton_wind_conveyor_moves_int_right)
			{
				conveyor_desired_pos_int[0]+=(*step1);
			}
			ui_robot.conveyor->move_joints(conveyor_desired_pos_int);
		}
	
			// odswierzenie pozycji robota
		if ((ui_state.conveyor.edp.is_synchronised)&&(ui_state.is_wind_conveyor_moves_open))
		{
				
			PtSetResource(ABW_PtNumericFloat_wind_conveyor_moves_inc_pos, Pt_ARG_NUMERIC_VALUE, &conveyor_desired_pos_motors[0] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_conveyor_moves_int_pos, Pt_ARG_NUMERIC_VALUE, &conveyor_desired_pos_int[0] , 0);
		
		}
	}
	} // end try
	CATCH_SECTION_UI
	
	return( Pt_CONTINUE );
}

//jk
int conveyor_read_servo_algorithm()
{
	BYTE servo_alg_no[1];
	BYTE servo_par_no[1];
	ui_robot.conveyor->get_servo_algorithm(servo_alg_no, servo_par_no);
	double* v = new double[2];
	v[0] = servo_alg_no[0];
	v[1] = servo_par_no[0];
	replySend(new Message('D','B','A',2,v,NULL));
}
int conveyor_read_joints()
{
	double* v = new double[2];
	double* vv = new double[1];
	try
	{
		if (ui_state.conveyor.edp.pid!=-1)
		{
			if ( ui_state.conveyor.edp.is_synchronised )
			{
				if (!(ui_robot.conveyor->read_joints(vv))) printf("Blad w read motors\n");
				v[1] = *vv;
				if (!(ui_robot.conveyor->read_motors(vv))) printf("Blad w read motors\n");
				v[0] = *vv;
				replySend(new Message('D','A','A',2,v,NULL));
			}
		}
	}
	CATCH_SECTION_UI
}
int conveyor_read_motors()
{
	double* v = new double[2];
	double* vv = new double[1];
	try
	{
		if (ui_state.conveyor.edp.pid!=-1)
		{
			if ( ui_state.conveyor.edp.is_synchronised )
			{
				if (!(ui_robot.conveyor->read_joints(vv))) printf("Blad w read motors\n");
				v[1] = *vv;
				if (!(ui_robot.conveyor->read_motors(vv))) printf("Blad w read motors\n");
				v[0] = *vv;
				replySend(new Message('D','A','A',2,v,NULL));
			}
		}
	}
	CATCH_SECTION_UI
}

int conveyor_moves_move_motors(double* v)
{
	double conveyor_desired_pos_motors[6];
	try
	{
		if (ui_state.conveyor.edp.pid!=-1)
		{
		
				if (ui_state.conveyor.edp.is_synchronised)
				{
					conveyor_desired_pos_motors[0] = v[0];
				}
				else
				{
					conveyor_desired_pos_motors[0]=0.0;
				}
				ui_robot.conveyor->move_motors(conveyor_desired_pos_motors);
				conveyor_read_motors();
		}
	}
	CATCH_SECTION_UI
	return 0;
}

int conveyor_moves_move_joints(double* v)
{
	double conveyor_desired_pos_int[6] ;
	try
	{
		if (ui_state.conveyor.edp.pid!=-1)
		{
		
				if (ui_state.conveyor.edp.is_synchronised)
				{
					conveyor_desired_pos_int[0] = v[0];
				}
				else
				{
					conveyor_desired_pos_int[0]=0.0;
				}
				ui_robot.conveyor->move_joints(conveyor_desired_pos_int);
				conveyor_read_motors();
		}
	}
	CATCH_SECTION_UI

	return 0;
}

int conv_servo_algorithm_set(double* v)
{
	BYTE servo_alg_no_output[CONVEYOR_NUM_OF_SERVOS];
	BYTE servo_par_no_output[CONVEYOR_NUM_OF_SERVOS];

	try
	{
		if ( ui_state.conveyor.edp.is_synchronised )
		{
			for(int i=0; i<CONVEYOR_NUM_OF_SERVOS; i++)
			{
				servo_alg_no_output[i] = (char)v[2*i];
				servo_par_no_output[i] = (char)v[2*i+1];
			}
			ui_robot.conveyor->set_servo_algorithm(servo_alg_no_output, servo_par_no_output);
			conveyor_read_servo_algorithm();
		}
	}
	CATCH_SECTION_UI
	
	return 0;
}

int conveyor_move_to_synchro_position()
{
	try
	{
		if (ui_state.conveyor.edp.pid!=-1)
		{
			
			for (int i = 0; i < CONVEYOR_NUM_OF_SERVOS; i++)
			{
				conveyor_desired_pos[i] = 0.0;
		    }
			ui_robot.conveyor->move_motors(conveyor_desired_pos);
			conveyor_read_joints();
			conveyor_read_motors();
		}   
	}
	CATCH_SECTION_UI

	return 0;
}

int conveyor_move_to_position0()
{
	try
	{
		if (ui_state.conveyor.edp.pid!=-1)
		{
			
			for (int i = 0; i < CONVEYOR_NUM_OF_SERVOS; i++)
			{
				conveyor_desired_pos[i] = ui_state.conveyor.edp.preset_position[0][i];
		    }
			ui_robot.conveyor->move_motors(conveyor_desired_pos);
			conveyor_read_joints();
			conveyor_read_motors();
		}   
	}
	CATCH_SECTION_UI

	return 0;
}

int conveyor_move_to_position1()
{
	try
	{
		if (ui_state.conveyor.edp.pid!=-1)
		{
			
			for (int i = 0; i < CONVEYOR_NUM_OF_SERVOS; i++)
			{
				conveyor_desired_pos[i] = ui_state.conveyor.edp.preset_position[1][i];
		    }
			ui_robot.conveyor->move_motors(conveyor_desired_pos);
			conveyor_read_joints();
			conveyor_read_motors();
		}   
	}
	CATCH_SECTION_UI

	return 0;
}

int conveyor_move_to_position2()
{
	try
	{
		if (ui_state.conveyor.edp.pid!=-1)
		{
			
			for (int i = 0; i < CONVEYOR_NUM_OF_SERVOS; i++)
			{
				conveyor_desired_pos[i] = ui_state.conveyor.edp.preset_position[2][i];
		    }
			ui_robot.conveyor->move_motors(conveyor_desired_pos);
			conveyor_read_joints();
			conveyor_read_motors();
		}   
	}
	CATCH_SECTION_UI

	return 0;
}

int EDP_conveyor_synchronise()
{
	try
	{
		if ((ui_state.conveyor.edp.state > 0) && (ui_state.conveyor.edp.is_synchronised == false))
		{
			ui_robot.conveyor->synchronise();
			ui_state.conveyor.edp.is_synchronised = ui_robot.conveyor->is_synchronised();
		}
		if ((ui_state.conveyor.edp.state > 0) && (ui_state.conveyor.edp.is_synchronised == true)) replySend(new Message('D','E','A',0,NULL,NULL));
	}
	CATCH_SECTION_UI
	
	manage_interface();
	return 0;
}

int EDP_conveyor_create()
{
	short tmp;
	char tmp_string[100];
	char tmp2_string[100];
	FILE* file;
	controller_state_typedef robot_controller_initial_state_tmp;

	try {
		if (ui_state.conveyor.edp.state == 0)
		{
			strcpy(tmp_string, "/dev/name/global/");
			strcat(tmp_string, ui_state.conveyor.edp.hardware_busy_attach_point);
			
			strcpy(tmp2_string, "/dev/name/global/");
			strcat(tmp2_string, ui_state.conveyor.edp.network_resourceman_attach_point);
			if((!(ui_state.conveyor.edp.test_mode)) && ( access(tmp_string, R_OK)== 0  )
				|| (access(tmp2_string, R_OK)== 0 )
			)
			{
				ui_msg.ui->message("edp_conveyor already exists");
			} else {
				ui_state.conveyor.edp.node_nr = config->return_node_number(ui_state.conveyor.edp.node_name);
				
				ui_robot.conveyor = new ui_conveyor_robot(&ui_state.irp6_on_track.edp, *config, ui_msg.all_ecp);
				ui_state.conveyor.edp.pid = ui_robot.conveyor->get_EDP_pid();

				if (ui_state.conveyor.edp.pid<0)
				{
					fprintf( stderr, "EDP spawn failed: %s\n", strerror( errno ));
					delete ui_robot.conveyor;
				} else {  // jesli spawn sie powiodl
					
					 tmp = 0;
				 	// kilka sekund  (~1) na otworzenie urzadzenia
					while((ui_state.conveyor.edp.reader_fd = name_open(ui_state.conveyor.edp.network_reader_attach_point, 
						NAME_FLAG_ATTACH_GLOBAL))  < 0)
						if((tmp++)<20)
							delay(50);
						else{
						   perror("blad odwolania do READER_C\n");
		   				   break;
						};
					
					// odczytanie poczatkowego stanu robota (komunikuje sie z EDP)	
					ui_robot.conveyor->get_controller_state(&robot_controller_initial_state_tmp);
		
					ui_state.conveyor.edp.state = 1; // edp wlaczone reader czeka na start
					replySend(new Message('D','C','A',0,NULL,NULL));
					if (!robot_controller_initial_state_tmp.is_synchronised) // jesli robot nie jest zsynchronizowany
					{
						ui_state.conveyor.edp.is_synchronised = false; // edp wlaczone reader czeka na start
					} else { // jesli robot jest zsynchronizowany
						ui_state.conveyor.edp.is_synchronised = true; // edp wlaczone reader czeka na start
					}
						
				}
			}
		}
	}
	CATCH_SECTION_UI
manage_interface();
	return 0;
}
	
int EDP_conveyor_slay()
{
	if (ui_state.conveyor.edp.state>0)
	 { // jesli istnieje EDP
		name_close(ui_state.conveyor.edp.reader_fd);
		delete ui_robot.conveyor;
		SignalKill(ui_state.conveyor.edp.node_nr, ui_state.conveyor.edp.pid, 0, SIGTERM, 0, 0);
		ui_state.conveyor.edp.state = 0; // edp wylaczone
		ui_state.conveyor.edp.is_synchronised = false;
		ui_state.conveyor.edp.pid = -1;
		ui_state.conveyor.edp.reader_fd = -1;
	}

	replySend(new Message('D','D','A',0,NULL,NULL));
	manage_interface();
	return 0;
}
//~jk

int
EDP_conveyor_synchronise( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	try
	{
	// dla robota irp6_on_track

		if ((ui_state.conveyor.edp.state > 0) &&
			(ui_state.conveyor.edp.is_synchronised == false))
		{
			ui_robot.conveyor->synchronise();
			ui_state.conveyor.edp.is_synchronised = ui_robot.conveyor->is_synchronised();
		} else {
			// 	printf("EDP conveyor niepowolane, synchronizacja niedozwolona\n");
		}

	} // end try
	CATCH_SECTION_UI

	// modyfikacje menu
	manage_interface();

	return( Pt_CONTINUE );

	}
	


int
init_wnd_conveyor_servo_algorithm( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	BYTE servo_alg_no[CONVEYOR_NUM_OF_SERVOS];
	BYTE servo_par_no[CONVEYOR_NUM_OF_SERVOS];

	// wychwytania ew. bledow ECP::robot
	try
	{
	if (ui_state.conveyor.edp.pid!=-1)
	{
		if ( ui_state.conveyor.edp.is_synchronised )  // Czy robot jest zsynchronizowany?
		{
			if (!(ui_robot.conveyor->get_servo_algorithm(servo_alg_no, servo_par_no))) // Odczyt polozenia walow silnikow
				printf("Blad w conveyor get_servo_algorithm\n");

			PtSetResource(ABW_PtNumericInteger_wnd_conv_servo_algorithm_read_alg_1, Pt_ARG_NUMERIC_VALUE, servo_alg_no[0] , 0);
					
			PtSetResource(ABW_PtNumericInteger_wnd_conv_servo_algorithm_read_par_1, Pt_ARG_NUMERIC_VALUE, servo_par_no[0] , 0);
			
		} else
		{

		}
	}
	} // end try
	CATCH_SECTION_UI

	return( Pt_CONTINUE );

	}


int
conv_servo_algorithm_set( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	BYTE *servo_alg_no_tmp [CONVEYOR_NUM_OF_SERVOS];
	BYTE servo_alg_no_output[CONVEYOR_NUM_OF_SERVOS];
	BYTE *servo_par_no_tmp [CONVEYOR_NUM_OF_SERVOS];
	BYTE servo_par_no_output[CONVEYOR_NUM_OF_SERVOS];
	
	// wychwytania ew. bledow ECP::robot
	try
	{
	if ( ui_state.conveyor.edp.is_synchronised )
	{
	
		PtGetResource(ABW_PtNumericInteger_wnd_conv_servo_algorithm_alg_1, Pt_ARG_NUMERIC_VALUE, &servo_alg_no_tmp[0], 0 );

		PtGetResource(ABW_PtNumericInteger_wnd_conv_servo_algorithm_par_1, Pt_ARG_NUMERIC_VALUE, &servo_par_no_tmp[0], 0 );
			
		for(int i=0; i<CONVEYOR_NUM_OF_SERVOS; i++)
		{
			servo_alg_no_output[i] = *servo_alg_no_tmp[i];
			servo_par_no_output[i] = *servo_par_no_tmp[i];
		}
	
		// zlecenie wykonania ruchu
		ui_robot.conveyor->set_servo_algorithm(servo_alg_no_output, servo_par_no_output);
		
	}
	else
	{
	}
	} // end try
	CATCH_SECTION_UI


	return( Pt_CONTINUE );

	}
	
	
	


int
EDP_conveyor_create( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	short tmp;
	char tmp_string[100];
	char tmp2_string[100];
	FILE* file;					// do sprawdzenia czy istnieje /net/node_name/dev/TWOJ_ROBOT
	controller_state_typedef robot_controller_initial_state_tmp;

	try { // dla bledow robot :: ECP_error
	
	// dla robota conveyor
	if (ui_state.conveyor.edp.state == 0)
	{
		strcpy(tmp_string, "/dev/name/global/");
		strcat(tmp_string, ui_state.conveyor.edp.hardware_busy_attach_point);
		
		strcpy(tmp2_string, "/dev/name/global/");
		strcat(tmp2_string, ui_state.conveyor.edp.network_resourceman_attach_point);
		// sprawdzeie czy nie jest juz zarejestrowany zarzadca zasobow
		if((!(ui_state.conveyor.edp.test_mode)) && ( access(tmp_string, R_OK)== 0  )
			|| (access(tmp2_string, R_OK)== 0 )
		)
		{
			ui_msg.ui->message("edp_conveyor already exists");
		} else {
			ui_state.conveyor.edp.node_nr = config->return_node_number(ui_state.conveyor.edp.node_name);
			
			ui_robot.conveyor = new ui_conveyor_robot(&ui_state.irp6_on_track.edp, *config, ui_msg.all_ecp);
			ui_state.conveyor.edp.pid = ui_robot.conveyor->get_EDP_pid();

			if (ui_state.conveyor.edp.pid<0)
			{
				fprintf( stderr, "EDP spawn failed: %s\n", strerror( errno ));
				delete ui_robot.conveyor;
			} else {  // jesli spawn sie powiodl
				
				 tmp = 0;
			 	// kilka sekund  (~1) na otworzenie urzadzenia
				while((ui_state.conveyor.edp.reader_fd = name_open(ui_state.conveyor.edp.network_reader_attach_point, 
					NAME_FLAG_ATTACH_GLOBAL))  < 0)
					if((tmp++)<20)
						delay(50);
					else{
					   perror("blad odwolania do READER_C\n");
	   				   break;
					};
				
				// odczytanie poczatkowego stanu robota (komunikuje sie z EDP)	
				ui_robot.conveyor->get_controller_state(&robot_controller_initial_state_tmp);
	
				ui_state.conveyor.edp.state = 1; // edp wlaczone reader czeka na start
				if (!robot_controller_initial_state_tmp.is_synchronised) // jesli robot nie jest zsynchronizowany
				{
					ui_state.conveyor.edp.is_synchronised = false; // edp wlaczone reader czeka na start
				} else { // jesli robot jest zsynchronizowany
					ui_state.conveyor.edp.is_synchronised = true; // edp wlaczone reader czeka na start
				}
					
			}
		}
	}
	

	} // end try
	CATCH_SECTION_UI

	manage_interface();

	return( Pt_CONTINUE );

	}



int
EDP_conveyor_slay( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// dla robota conveyor
	if (ui_state.conveyor.edp.state>0)
	 { // jesli istnieje EDP
		name_close(ui_state.conveyor.edp.reader_fd);

		delete ui_robot.conveyor;
		SignalKill(ui_state.conveyor.edp.node_nr, ui_state.conveyor.edp.pid, 0, SIGTERM, 0, 0);
		ui_state.conveyor.edp.state = 0; // edp wylaczone
		ui_state.conveyor.edp.is_synchronised = false;
	
		ui_state.conveyor.edp.pid = -1;
		ui_state.conveyor.edp.reader_fd = -1;
		
		close_wind_conveyor_moves(NULL, NULL, NULL);
		close_wnd_conveyor_servo_algorithm (NULL, NULL, NULL);
	}

	// modyfikacja menu
	manage_interface();

	return( Pt_CONTINUE );

	}



int
pulse_reader_conv_start( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (pulse_reader_conv_start_exec_pulse()) process_control_window_init(widget, apinfo, cbinfo);

	return( Pt_CONTINUE );

	}
	
//jk
int
pulse_reader_conv_start()
{
	pulse_reader_conv_start_exec_pulse();

	return 0;
}
//~jk	



bool pulse_reader_conv_start_exec_pulse ()
{

	if (ui_state.conveyor.edp.state == 1)
	{
		pulse_reader_execute( ui_state.conveyor.edp.reader_fd, READER_START, 0);
		ui_state.conveyor.edp.state = 2;
		return true;
	}
	
	return false;	
}



int
pulse_reader_conv_stop( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	
	if (pulse_reader_conv_stop_exec_pulse()) process_control_window_init(widget, apinfo, cbinfo);
	
	return( Pt_CONTINUE );

	}

//jk
int
pulse_reader_conv_stop()
{
	pulse_reader_conv_stop_exec_pulse();
	
	return 0;
}
//~jk

bool pulse_reader_conv_stop_exec_pulse ()
{

	if (ui_state.conveyor.edp.state == 2)
	{
		pulse_reader_execute( ui_state.conveyor.edp.reader_fd, READER_STOP, 0);
		ui_state.conveyor.edp.state = 1;
		return true;
	}
	
	return false;	
}



int
pulse_reader_conv_trigger( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	
	if (pulse_reader_conv_trigger_exec_pulse()) process_control_window_init(widget, apinfo, cbinfo);

	return( Pt_CONTINUE );

	}

//jk
int
pulse_reader_conv_trigger()
{
	pulse_reader_conv_trigger_exec_pulse();

	return 0;
}
//~jk
	
bool pulse_reader_conv_trigger_exec_pulse ()
{

	if (ui_state.conveyor.edp.state == 2)
	{
		pulse_reader_execute( ui_state.conveyor.edp.reader_fd, READER_TRIGGER, 0);

		return true;
	}
	
	return false;	
}


int
pulse_ecp_conveyor( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{
	
	char pulse_code = ECP_TRIGGER;
	long pulse_value = 1;

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui_state.conveyor.edp.is_synchronised>0)
	 { // o ile ECP dziala (sprawdzanie poprzez dzialanie odpowiedniego EDP)
		if (ui_state.conveyor.ecp.trigger_fd < 0)
		 {
		
			 short tmp = 0;
		 	// kilka sekund  (~1) na otworzenie urzadzenia
		 	// zabezpieczenie przed zawieszeniem poprzez wyslanie sygnalu z opoznieniem
		 	ualarm( (useconds_t)( SIGALRM_TIMEOUT), 0);
			while( (ui_state.conveyor.ecp.trigger_fd = name_open(ui_state.conveyor.ecp.network_trigger_attach_point, NAME_FLAG_ATTACH_GLOBAL)) < 0)
			{
				if (errno == EINTR) break;
				if((tmp++)<20)
					delay(50);
				else{
				   perror("blad odwolania do ECP_TRIGGER\n");
				};
			}
			// odwolanie alarmu
			ualarm( (useconds_t)( 0), 0);
		}

		if (ui_state.conveyor.ecp.trigger_fd >= 0) {
			if (MsgSendPulse (ui_state.conveyor.ecp.trigger_fd, sched_get_priority_min(SCHED_FIFO),  pulse_code,  pulse_value)==-1) {
				
				fprintf( stderr, "Blad w wysylaniu pulsu do ecp error: %s \n",  strerror( errno ) );
				delay(1000);
			}	
		} else {
			printf("W PULS ECP:  BLAD name_open \n");
		}
	}

	return( Pt_CONTINUE );

	}

	
//jk
int
pulse_ecp_conveyor()
{
	char pulse_code = ECP_TRIGGER;
	long pulse_value = 1;

	if (ui_state.conveyor.edp.is_synchronised>0)
	{ // o ile ECP dziala (sprawdzanie poprzez dzialanie odpowiedniego EDP)
		if (ui_state.conveyor.ecp.trigger_fd < 0)
		{
		
			 short tmp = 0;
		 	// kilka sekund  (~1) na otworzenie urzadzenia
		 	// zabezpieczenie przed zawieszeniem poprzez wyslanie sygnalu z opoznieniem
		 	ualarm( (useconds_t)( SIGALRM_TIMEOUT), 0);
			while( (ui_state.conveyor.ecp.trigger_fd = name_open(ui_state.conveyor.ecp.network_trigger_attach_point, NAME_FLAG_ATTACH_GLOBAL)) < 0)
			{
				if (errno == EINTR) break;
				if((tmp++)<20)
					delay(50);
				else{
				   perror("blad odwolania do ECP_TRIGGER\n");
				};
			}
			// odwolanie alarmu
			ualarm( (useconds_t)( 0), 0);
		}

		if (ui_state.conveyor.ecp.trigger_fd >= 0) {
			if (MsgSendPulse (ui_state.conveyor.ecp.trigger_fd, sched_get_priority_min(SCHED_FIFO),  pulse_code,  pulse_value)==-1) {
				
				fprintf( stderr, "Blad w wysylaniu pulsu do ecp error: %s \n",  strerror( errno ) );
				delay(1000);
			}	
		} else {
			printf("W PULS ECP:  BLAD name_open \n");
		}
	}

	return 0;
}
//~jk

// aktualizacja ustawien przyciskow
int
process_control_window_conveyor_section_init (bool &wlacz_PtButton_wnd_processes_control_all_reader_start,
	bool &wlacz_PtButton_wnd_processes_control_all_reader_stop,
	bool &wlacz_PtButton_wnd_processes_control_all_reader_trigger)
{
double* tmp = new double[2];
	if (ui_state.conveyor.edp.state<=0) {// edp wylaczone
		tmp[0] = 2;
		tmp[1] = 0;
		replySend(new Message('A','H','A',2,tmp,NULL));
	} else {
		if (ui_state.conveyor.edp.state==1) {// edp wlaczone reader czeka na start
			wlacz_PtButton_wnd_processes_control_all_reader_start=true;
			tmp[0] = 2;
			tmp[1] = 4;
			replySend(new Message('A','H','A',2,tmp,NULL));
		} else if (ui_state.conveyor.edp.state==2) {// edp wlaczone reader czeka na stop
			wlacz_PtButton_wnd_processes_control_all_reader_stop=true;
			wlacz_PtButton_wnd_processes_control_all_reader_trigger=true;
			tmp[0] = 2;
			tmp[1] = 3;
			replySend(new Message('A','H','A',2,tmp,NULL));
		}
	}
	return 1;
}



int 
reload_conveyor_configuration ()
{


	// jesli conveyor ma byc aktywny
	if ((ui_state.conveyor.is_active = config->return_int_value("is_conveyor_active")) == 1) 
	{
	
		//ui_state.is_any_edp_active = true;
		
		if (ui_state.is_mp_and_ecps_active)
		{
			delete [] ui_state.conveyor.ecp.network_trigger_attach_point;
			ui_state.conveyor.ecp.network_trigger_attach_point =config->return_attach_point_name 
				(configurator::CONFIG_SERVER, "trigger_attach_point", ui_state.conveyor.ecp.section_name);
			
	 		ui_state.conveyor.ecp.pid = -1;
	 		ui_state.conveyor.ecp.trigger_fd = -1;
	 	}
		
		switch (ui_state.conveyor.edp.state)
		{
			case -1:
			case 0:

				ui_state.conveyor.edp.pid = -1;
				ui_state.conveyor.edp.reader_fd = -1;
				ui_state.conveyor.edp.state = 0;

				if (config->exists("preset_position_0", ui_state.conveyor.edp.section_name))
					ui_state.conveyor.edp.preset_position[0][0] = config->return_double_value ("preset_position_0", ui_state.conveyor.edp.section_name);
				if (config->exists("preset_position_1", ui_state.conveyor.edp.section_name))	
					ui_state.conveyor.edp.preset_position[1][0] = config->return_double_value ("preset_position_1", ui_state.conveyor.edp.section_name);
				if (config->exists("preset_position_2", ui_state.conveyor.edp.section_name))
					ui_state.conveyor.edp.preset_position[2][0] = config->return_double_value ("preset_position_2", ui_state.conveyor.edp.section_name);

				if (config->exists("test_mode", ui_state.conveyor.edp.section_name))
					ui_state.conveyor.edp.test_mode = config->return_int_value("test_mode", ui_state.conveyor.edp.section_name);
				else 
					ui_state.conveyor.edp.test_mode = 0;
				
				delete [] ui_state.conveyor.edp.hardware_busy_attach_point;
				ui_state.conveyor.edp.hardware_busy_attach_point = config->return_string_value 
					("hardware_busy_attach_point", ui_state.conveyor.edp.section_name);



				delete [] ui_state.conveyor.edp.network_resourceman_attach_point;
				ui_state.conveyor.edp.network_resourceman_attach_point = config->return_attach_point_name 
					(configurator::CONFIG_SERVER, "resourceman_attach_point", ui_state.conveyor.edp.section_name);

				delete [] ui_state.conveyor.edp.network_reader_attach_point;
				ui_state.conveyor.edp.network_reader_attach_point = config->return_attach_point_name
					(configurator::CONFIG_SERVER, "reader_attach_point", ui_state.conveyor.edp.section_name);

				delete [] ui_state.conveyor.edp.node_name;
				ui_state.conveyor.edp.node_name = config->return_string_value ("node_name", ui_state.conveyor.edp.section_name);
				
			break;
			case 1:
			case 2:
				// nie robi nic bo EDP pracuje
			break;
			default:
			break;
		}
	
	} else // jesli  conveyor ma byc nieaktywny
	{

		switch (ui_state.conveyor.edp.state) 
		{
			case -1:
			case 0:
				ui_state.conveyor.edp.state=-1;
			break;
			case 1:
			case 2:
				// nie robi nic bo EDP pracuje
			break;
			default:
			break;
		}
	} // end conveyor

	return 1;
}



int 
manage_interface_conveyor ()
{
	switch (ui_state.conveyor.edp.state)
	{
		case -1:
			ApModifyItemState( &robot_menu, AB_ITEM_DIM, ABN_mm_conveyor, NULL);
			replySend(new Message('D','E','A',0,NULL,NULL));
		break;
		case 0:
			ApModifyItemState( &robot_menu, AB_ITEM_DIM, ABN_mm_conveyor_edp_unload, ABN_mm_conveyor_synchronisation,
				ABN_mm_conveyor_move,  ABN_mm_conveyor_preset_positions, ABN_mm_conveyor_servo_algorithm, NULL);
			ApModifyItemState( &robot_menu, AB_ITEM_NORMAL, ABN_mm_conveyor, ABN_mm_conveyor_edp_load, NULL);
			replySend(new Message('D','E','B',0,NULL,NULL));

		break;
		case 1:
		case 2:
			ApModifyItemState( &robot_menu, AB_ITEM_NORMAL, ABN_mm_conveyor, NULL);
			replySend(new Message('D','E','C',0,NULL,NULL));
			
			// jesli robot jest zsynchronizowany
			if (	ui_state.conveyor.edp.is_synchronised)			
			{
				ApModifyItemState( &robot_menu, AB_ITEM_DIM, ABN_mm_conveyor_synchronisation, NULL);
				ApModifyItemState( &all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_preset_positions, NULL);
				replySend(new Message('D','E','D',0,NULL,NULL));
				switch (ui_state.mp.state)
				{
					case UI_MP_NOT_PERMITED_TO_RUN:
					case UI_MP_PERMITED_TO_RUN:
						ApModifyItemState( &robot_menu, AB_ITEM_NORMAL,  ABN_mm_conveyor_edp_unload, 
							ABN_mm_conveyor_move, ABN_mm_conveyor_preset_positions, ABN_mm_conveyor_servo_algorithm, NULL);
						ApModifyItemState( &robot_menu, AB_ITEM_DIM, ABN_mm_conveyor_edp_load, NULL);
						replySend(new Message('D','E','E',0,NULL,NULL));
					break;
					case UI_MP_WAITING_FOR_START_PULSE:
						ApModifyItemState( &robot_menu, AB_ITEM_NORMAL, 
							ABN_mm_conveyor_move, ABN_mm_conveyor_preset_positions, ABN_mm_conveyor_servo_algorithm, NULL);
						ApModifyItemState( &robot_menu, AB_ITEM_DIM, ABN_mm_conveyor_edp_load, ABN_mm_conveyor_edp_unload, NULL);
						replySend(new Message('D','E','F',0,NULL,NULL));
					break;
					case UI_MP_TASK_RUNNING:
					case UI_MP_TASK_PAUSED:
						ApModifyItemState( &robot_menu, AB_ITEM_DIM, // modyfikacja menu - ruchy reczne zakazane
							ABN_mm_conveyor_move, ABN_mm_conveyor_preset_positions, ABN_mm_conveyor_servo_algorithm, NULL);
							replySend(new Message('D','E','G',0,NULL,NULL));
					break;
					default:
					break;
				}
			} else		// jesli robot jest niezsynchronizowany
			{
				ApModifyItemState( &robot_menu, AB_ITEM_NORMAL, ABN_mm_conveyor_edp_unload,
					ABN_mm_conveyor_synchronisation, ABN_mm_conveyor_move, NULL);
				ApModifyItemState( &robot_menu, AB_ITEM_DIM, ABN_mm_conveyor_edp_load, NULL);
				ApModifyItemState( &all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_synchronisation, NULL);
				replySend(new Message('D','E','H',0,NULL,NULL));
				replySend(new Message('A','F','A',0,NULL,NULL));
			}
		break;
		default:
		break;
	}

	return 1;
}
