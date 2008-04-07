#include <iostream>
#include <unistd.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"
#include <fstream>

#include "ecp_mp/ecp_mp_s_force.h"
#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"	

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_postument/ecp_local.h"
#include "ecp/common/ecp_t_tzu_cs_irp6ot.h"
#include "lib/mathtr.h"
#include "ecp/common/ecp_g_smooth.h"

using namespace std;
/** konstruktor **/
ecp_task_tzu_cs_irp6ot::ecp_task_tzu_cs_irp6ot(configurator &_config) : ecp_task(_config)
{
	sg = NULL;
	befg = NULL;
	ftcg = NULL;
	tcg = NULL;
};

/** destruktor **/
ecp_task_tzu_cs_irp6ot::~ecp_task_tzu_cs_irp6ot()
{
};


// methods for ECP template to redefine in concrete classes
void ecp_task_tzu_cs_irp6ot::task_initialization(void) 
{
	// ecp_m_robot = new ecp_irp6_on_track_robot (*this);
	if (strcmp(config.section_name, "[ecp_irp6_on_track]") == 0)
	{
		ecp_m_robot = new ecp_irp6_on_track_robot (*this);
		trajectories[0] = "../trj/tzu/tzu_1_on_track.trj";
		trajectories[1] = "../trj/tzu/tzu_2_on_track.trj";
		trajectories[2] = "../trj/tzu/tzu_3_on_track.trj";
	}
	else if (strcmp(config.section_name, "[ecp_irp6_postument]") == 0)
	{
		ecp_m_robot = new ecp_irp6_postument_robot (*this);
		trajectories[0] = "../trj/tzu/tzu_1_postument.trj";
		trajectories[1] = "../trj/tzu/tzu_2_postument.trj";
		trajectories[2] = "../trj/tzu/tzu_3_postument.trj";
	}
	
	sg = new ecp_smooth_generator (*this, true, false);
	befg = new bias_edp_force_generator(*this);
	wmg = new weight_meassure_generator(*this, 1);
	fmg = new force_meassure_generator(*this);	
	ftcg = new ecp_force_tool_change_generator(*this);
	tcg = new ecp_tool_change_generator(*this,true);
	sr_ecp_msg->message("ECP loaded");
};

void ecp_task_tzu_cs_irp6ot::main_task_algorithm(void)
{
	sr_ecp_msg->message("ECP cs irp6ot  - pushj start in tzu");
	ecp_wait_for_start();
	
	/* stworzenie generatora ruchu */
	
	while(true)			
	{ 
		// ETAP PIERWSZY - chwytka skierowany pionowo do dolu, biasowanie odczytow, 
		// zmiana narzedzia kinematycznego, ustawienie end-effector frame E jako sensor frame S
		sg->load_file_with_path(trajectories[TRAJECTORY_VERTICAL_DOWN]);
		sg->Move ();		
		befg->Move();
		ftcg->Move();
		tcg->set_tool_parameters(0,0,0.09);
		tcg->Move();
	
		// ETAP DRUGI - chwytak skierowany pionowo do gory, odczyt i obliczenie trzech pierwszych parametrow
		// wagi, parametrow translacji?!?
		sg->load_file_with_path(trajectories[TRAJECTORY_VERTCAL_UP]);
		sg->Move ();
		fmg->Move();
		weight = fmg->weight[FORCE_Z]/2;
		P_x = fmg->weight[TORQUE_Y]/(2*weight);
		P_y = -fmg->weight[TORQUE_X]/(2*weight);
		cout<<"torque_x: "<<fmg->weight[TORQUE_X]<<endl;
		cout<<"torque_y: "<<fmg->weight[TORQUE_Y]<<endl;
		//cout<<"test funkcji: "<<*(fmg->get_meassurement())<<endl;
//		for(int i = 0 ; i < 10 ; i++)
//		{
//			fmg->Move();
//			weight = fmg->weight[2]/2;
//			std::cout<<"pomiar 1: "<<fmg->weight<<std::endl;
//			std::cout<<"weight_1"<<": "<<fmg->weight[2]/2<<std::endl;
//			sleep(1);
//		}
		
		// ETAP TRZECI - chwytak skierowany horyzontalnie, obliczenie ostatniego z parametr�w modelu
		sg->load_file_with_path(trajectories[TRAJECTORY_HORIZONTAL]);
		sg->Move ();
		fmg->Move();
		P_z = -fmg->weight[TORQUE_Y]/weight;
		cout<<"torque_y: "<<fmg->weight[TORQUE_Y]<<endl;
//		for(int i = 0 ; i < 10 ; i++)
//		{
//			fmg->Move();
//			std::cout<<"pomiar 2: "<<fmg->weight<<std::endl;
//			std::cout<<"t1: "<<-(fmg->weight[4]/weight)<<std::endl;
//			sleep(1);
//		}
		cout<<"Parametry modelu �rodka ci�ko�ci narz�dzia"<<endl
			<<"weight: "<<weight<<endl<<"P_x: "<<P_x<<endl<<"P_y: "<<P_y<<endl<<"P_z: "<<P_z<<endl; 
		
		ecp_termination_notice();
		ecp_wait_for_stop();
		break;
	}
	std::cout<<"end\n"<<std::endl;
};

ecp_task* return_created_ecp_task (configurator &_config)
{
	return new ecp_task_tzu_cs_irp6ot(_config);
};


force_meassure_generator::force_meassure_generator(ecp_task& _ecp_task, int _sleep_time, int _meassurement_count) :
	ecp_generator(_ecp_task)
{
	sleep_time = _sleep_time; 
	meassurement_count = _meassurement_count;
}

bool force_meassure_generator::set_configuration(int _sleep_time, int _meassurement_count)
{
	sleep_time = _sleep_time; 
	meassurement_count = _meassurement_count;
}

bool force_meassure_generator::first_step()
{
	the_robot->EDP_data.instruction_type = GET;
	the_robot->EDP_data.get_type = ARM_DV;
	the_robot->EDP_data.get_arm_type = FRAME;
	the_robot->EDP_data.next_interpolation_type
			= TCIM;

	return true;
}

Ft_v_vector *force_meassure_generator::get_meassurement()
{
	return &weight;
}

bool force_meassure_generator::next_step()
{
	//Ft_v_vector average_meassurement[meassurement_count];
	for(int i = 0 ; i < meassurement_count ; i++)
	{
		Homog_matrix current_frame_wo_offset(the_robot->EDP_data.current_arm_frame);
		current_frame_wo_offset.remove_translation();
	
		Ft_v_vector force_torque(the_robot->EDP_data.current_force_xyz_torque_xyz);
		weight += force_torque;
		sleep(sleep_time);
	}
	double test1 = 22.2;
	int test2 = 2;
	cout<<"dzielenie: "<<test1/test2<<endl;
	for(int i = 0 ; i < 6 ; i++)
		weight[i] = weight[i]/meassurement_count;
	return false;
}

