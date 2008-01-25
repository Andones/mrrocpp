// -------------------------------------------------------------------------
//                            mp.h
// Definicje struktur danych i metod dla procesow MP - generatory silowe
// 
// Ostatnia modyfikacja: 2005
// -------------------------------------------------------------------------

#if !defined(__MP_GEN_VIS_SAC_LX_H)
#define __MP_GEN_VIS_SAC_LX_H

#include "lib/mathtr.h"

// --------------------------------------------------------------------------
// Generator trajektorii dla zadan z wodzeniem za nos w tff ze zmiana orientacji

class mp_vis_sac_lx_generator : public mp_generator 
{
protected:
  int idle_step_counter; // Licznik jalowych krokow sterowania (bez wykonywania ruchu)
   int node_counter;               // biezacy wezel interpolacji
  
    mp_robot *irp6ot, *irp6p;
    sensor *vsp_force_irp6ot, *vsp_force_irp6p, *vsp_vis_sac;
    
    // do konfiguracji pracy generatora
    unsigned short irp6ot_con, irp6p_con;
 

     trajectory_description td;   
 
public:
	int step_no;
	double delta[6];
     
     frame_tab my_goal_frame_m;
     
	Homog_matrix C_Tx_G;
	Homog_matrix C_Tx_E;
	Homog_matrix O_Tx_E;
	Homog_matrix O_Tx_Ep;
	Homog_matrix O_Tx_G;
	Homog_matrix G_Tx_G2;
	
	Homog_matrix G_Tx_S;
	
	Homog_matrix O_Tx_C;
	Homog_matrix O_Tx_E__C;
	Homog_matrix O_Tx_G__C;

	Homog_matrix E_Tx_G;
	Homog_matrix E_Tx_Ep;
	
	Homog_matrix E_Tx_G__O;
	
	
	double O_r_E[3][6];
	double O_r_Ep[3][6];
	double O_r_Ep_d[3][6]; //roznica 1szego
	double O_r_Ep_d2[3][6]; //2giego stopnia
	
	double O_r_G[3][6];
	double E_r_G[3][6];
	double E_r_Ep[3][6];
	
	double O_reul_Ep[3][6];
	
	double O_eps_EG[3][6];
	double E_eps_EG[3][6]; //E_r_G; - prawdopodobnie to samo
	
    // konstruktor
    mp_vis_sac_lx_generator(mp_task& _mp_task, int step=0);  
	
	virtual bool first_step ();    
	virtual bool next_step ();    

}; // end : class MP_nose_run_force_generator

#endif
