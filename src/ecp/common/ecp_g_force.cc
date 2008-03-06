// -------------------------------------------------------------------------
//                            ecp.cc
//            Effector Control Process (ECP) - force methods
// Funkcje do tworzenia procesow ECP z wykorzystaniem sily
//
// Ostatnia modyfikacja: 2004r.
// -------------------------------------------------------------------------

#include <stdio.h>
#include <fstream>
#include <time.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp/common/ecp_g_force.h"






y_nose_run_force_generator::y_nose_run_force_generator(ecp_task& _ecp_task, int step):
        ecp_generator (_ecp_task, true)
{
    step_no = step;
};


bool y_nose_run_force_generator::first_step ( )
{


    for (int i=0; i<6; i++)
    {
        delta[i]=0.0;
    }



    td.interpolation_node_no = 1;
    td.internode_step_no = step_no;
    td.value_in_step_no = td.internode_step_no - 2;


    the_robot->EDP_data.instruction_type = GET;
    the_robot->EDP_data.get_type = ARM_DV; // arm - ORYGINAL
    the_robot->EDP_data.set_type = ARM_DV;

    //		the_robot->EDP_data.force_move_mode=2; // z regulacja silowa po query
    //		the_robot->EDP_data.position_set_mode=1; // przyrostowo

    //		the_robot->EDP_data.force_axis_quantity=3; // DOBRZE

    the_robot->EDP_data.set_arm_type = POSE_FORCE_TORQUE_AT_FRAME;
    the_robot->EDP_data.get_arm_type = POSE_FORCE_TORQUE_AT_FRAME;
    the_robot->EDP_data.motion_type = PF_VELOCITY;
    the_robot->EDP_data.motion_steps = td.internode_step_no;
    the_robot->EDP_data.value_in_step_no = td.value_in_step_no;
    /*
    		the_robot->EDP_data.ECPtoEDP_force_coordinates[0]=0.0;
    		the_robot->EDP_data.ECPtoEDP_force_coordinates[1]=0.0;
    		the_robot->EDP_data.ECPtoEDP_force_coordinates[2]=0.0;
     
    		for (int j=0; j<6 ; j++)	{
    			the_robot->EDP_data.position_increment[j]=0.0;
    		}
     
    		the_robot->EDP_data.dyslocation_matrix[0][0]=1;
    		the_robot->EDP_data.dyslocation_matrix[1][1]=1;
    		the_robot->EDP_data.dyslocation_matrix[2][2]=1;
    		the_robot->EDP_data.dyslocation_matrix[3][3]=0;
    		the_robot->EDP_data.dyslocation_matrix[4][4]=0;
    		the_robot->EDP_data.dyslocation_matrix[5][5]=0;
    */

    return true;
}
; // end: bool y_nose_run_force_generator::first_step ( )
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
bool y_nose_run_force_generator::next_step ( )
{
    struct timespec start[9];

    if (ecp_t.pulse_check())
    { // Koniec odcinka
        return false;
    }



    // Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
    the_robot->EDP_data.instruction_type = SET_GET;

    // Obliczenie zadanej pozycji posredniej w tym kroku ruchu
    // (okreslenie kolejnego wezla interpolacji)
    if (((node_counter)%1000)==1)
    {
        if( clock_gettime( CLOCK_REALTIME , &start[0]) == -1 )
        {
            printf("blad pomiaru czasu");
        }
        printf("ECP pomiarow: %d,  czas: %ld\n",node_counter , start[0].tv_sec%100);
    }


    return true;
}
; // end: bool y_nose_run_force_generator::next_step ( )


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////
//
// 			y_egg_force_generator
//
// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////



y_egg_force_generator::y_egg_force_generator(ecp_task& _ecp_task, int step, int mode):
        ecp_generator (_ecp_task, true)
{
    step_no = step;
    int_mode = mode;
};


bool y_egg_force_generator::first_step ( )
{



    for (int i=0; i<6; i++)
        delta[i]=0.0;

    gen_state = next_gen_state = 1; // jazda w powietrzu
    prev_gen_state = 0;


    td.interpolation_node_no = 1;
    td.internode_step_no = step_no;
    td.value_in_step_no = td.internode_step_no - 2;



    the_robot->EDP_data.instruction_type = GET;
    // 		the_robot->EDP_data.get_type = ARM_DV; // arm - ORYGINAL
    the_robot->EDP_data.get_type = ARM_DV+OUTPUTS_DV; // arm_inputs DEBUG
    the_robot->EDP_data.set_type = ARM_DV;
    /*
    		the_robot->EDP_data.force_move_mode=2; // z regulacja silowa po query
    		the_robot->EDP_data.position_set_mode=1; // przyrostowo
     
    		the_robot->EDP_data.force_axis_quantity=3; // DOBRZE
     
    		the_robot->EDP_data.set_arm_type = POSE_FORCE_LINEAR;
    		the_robot->EDP_data.get_arm_type = POSE_FORCE_LINEAR;
    		the_robot->EDP_data.motion_type = ABSOLUTE;
    		the_robot->EDP_data.motion_steps = td.internode_step_no;
    		the_robot->EDP_data.value_in_step_no = td.value_in_step_no;
     
    		the_robot->EDP_data.ECPtoEDP_force_coordinates[0]=0.0;
    		the_robot->EDP_data.ECPtoEDP_force_coordinates[1]=0.0;
    		the_robot->EDP_data.ECPtoEDP_force_coordinates[2]=0.0;
     
    		for (int j=0; j<6 ; j++)	{
    			the_robot->EDP_data.position_increment[j]=0.0;
    		}
     
    		the_robot->EDP_data.dyslocation_matrix[0][0]=1;
    		the_robot->EDP_data.dyslocation_matrix[1][1]=1;
    		the_robot->EDP_data.dyslocation_matrix[2][2]=1;
    		the_robot->EDP_data.dyslocation_matrix[3][3]=0;
    		the_robot->EDP_data.dyslocation_matrix[4][4]=0;
    		the_robot->EDP_data.dyslocation_matrix[5][5]=0;
    */


    return true;
}
; // end: bool y_egg_force_generator::first_step ( )
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
bool y_egg_force_generator::next_step ( )
{

    // Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
    the_robot->EDP_data.instruction_type = SET_GET;

    // Obliczenie zadanej pozycji posredniej w tym kroku ruchu
    // (okreslenie kolejnego wezla interpolacji)
    // 	printf("odczyt: %d, sila: %f\n", the_robot->EDP_data.analog_input[1], (sensor_m.begin())->second->image.force.rez[2]);

    gen_state=next_gen_state;



    switch (gen_state)
    {
    case 0:
        break;
    case 1: // wodzenie za nos

        if (prev_gen_state != gen_state)
        {
            sr_ecp_msg.message("Wodzenie za nos - aby zakonczyc nacisnij ECP_TRIGGER");
        }

        td.interpolation_node_no = 1;
        td.internode_step_no = step_no;
        td.value_in_step_no = td.internode_step_no - 2;
        /*
        		the_robot->EDP_data.force_axis_quantity=3; // DOBRZE
        		the_robot->EDP_data.motion_steps = td.internode_step_no;
        		the_robot->EDP_data.value_in_step_no = td.value_in_step_no;
         
        		the_robot->EDP_data.ECPtoEDP_force_coordinates[0]=0.0;
        		the_robot->EDP_data.ECPtoEDP_force_coordinates[1]=0.0;
        		the_robot->EDP_data.ECPtoEDP_force_coordinates[2]=0.0;
         
        		for (int j=0; j<6 ; j++)	{
        			the_robot->EDP_data.position_increment[j]=0.0;
        		}
        */
        // inkrementacja numeru iteracji dla biezacego stanu
        in_state_iteration++;

        if (ecp_t.pulse_check())
        {
            in_state_iteration=0;
            next_gen_state = 2;
        }

        break;
    case 2:  // oczekiwanie na jajko (brak ruchu)

        if (prev_gen_state != gen_state)
        {
            sr_ecp_msg.message("Wloz jajko i nacisnij ECP_TRIGGER");
        }

        td.interpolation_node_no = 1;
        td.internode_step_no = step_no;
        td.value_in_step_no = td.internode_step_no - 2;
        /*
        		the_robot->EDP_data.force_axis_quantity=0; // ruch czysto pozycyjny
         
        		the_robot->EDP_data.motion_steps = td.internode_step_no;
        		the_robot->EDP_data.value_in_step_no = td.value_in_step_no;
         
        		// inkrementacja numeru iteracji dla biezacego stanu
        		in_state_iteration++;
         
        		// bezruch
        			for (int j=0; j<6 ; j++)	{
        			the_robot->EDP_data.position_increment[j]=0.0;
        		}
        */
        if (ecp_t.pulse_check())
        {
            in_state_iteration=0;
            next_gen_state = 3;
        }

        break;

    case 3:  // uwzglednianie ciezaru jajka - zerowanie odczytow czujnika (bias)

        // zerowanie odczytow
        (sensor_m.begin())->second->to_vsp.parameters=1;
        (sensor_m.begin())->second->configure_sensor();

        next_gen_state = 4;

        break;

    case 4: // jazda w dol (pozycyjna)

        if (prev_gen_state != gen_state)
        {
            sr_ecp_msg.message("Pozycyjny ruch w dol");
        }

        td.interpolation_node_no = 1;
        td.internode_step_no = step_no;
        td.value_in_step_no = td.internode_step_no - 2;
        /*
        		the_robot->EDP_data.force_axis_quantity = 0;
         
        		the_robot->EDP_data.motion_steps = td.internode_step_no;
        		the_robot->EDP_data.value_in_step_no = td.value_in_step_no;
         
        		the_robot->EDP_data.position_increment[0]=0.0;
        		the_robot->EDP_data.position_increment[1]=0.0;
        		the_robot->EDP_data.position_increment[2]=-POS_Z_AXIS_MACROSTEP_INC;
        		the_robot->EDP_data.position_increment[3]=0.0;
        		the_robot->EDP_data.position_increment[4]=0.0;
        		the_robot->EDP_data.position_increment[5]=0.0;
        */
        in_state_iteration++;

        if (
            (in_state_iteration>INIT_ITER_NUMBER)&&(
                /*
                ((the_robot->EDP_data.analog_input[1]<PROG_ODLEGLOSCI_PODCZERWIEN_EGG)
                &&(the_robot->EDP_data.analog_input[1]>ODLEGLOSCI_PODCZERWIEN_MIN_VALUE))|| // sieje zerami bugami
                */
                ((sensor_m.begin())->second->image.force.rez[2]>SILA_KONTAKTU_EGG)
            ))
        {
            in_state_iteration=0;
            if (int_mode==0) // rozbijamy
            {
                next_gen_state = 5;
            } else // nierozbijamy
            {
                next_gen_state = 6;
            }

        }

        break;

    case 5: // jazda w dol (pozycyjna, po zetknieciu z jajkiem) - stan do rozbijania

        if (prev_gen_state != gen_state)
        {
            sr_ecp_msg.message("Pozycyjny ruch w dol, po zetknieciu z jajkiem");
        }

        td.interpolation_node_no = 1;
        td.internode_step_no = step_no;
        td.value_in_step_no = td.internode_step_no - 2;
        /*
        		the_robot->EDP_data.force_axis_quantity = 0;
         
        		the_robot->EDP_data.motion_steps = td.internode_step_no;
        		the_robot->EDP_data.value_in_step_no = td.value_in_step_no;
         
        		the_robot->EDP_data.position_increment[0]=0.0;
        		the_robot->EDP_data.position_increment[1]=0.0;
        		the_robot->EDP_data.position_increment[2]=-POS_Z_AXIS_MACROSTEP_INC;
        		the_robot->EDP_data.position_increment[3]=0.0;
        		the_robot->EDP_data.position_increment[4]=0.0;
        		the_robot->EDP_data.position_increment[5]=0.0;
        */
        in_state_iteration++;

        if (in_state_iteration>IMPACT_ITERATIONS_NUMBER)
        {
            in_state_iteration=0;
            next_gen_state = 6;
        }

        break;


    case 6: // silowy ruch w dol

        if (prev_gen_state != gen_state)
        {
            sr_ecp_msg.message("Silowy ruch w dol - aby zakonczyc nacisnij ECP_TRIGGER");
        }

        td.interpolation_node_no = 1;
        td.internode_step_no = step_no;
        td.value_in_step_no = td.internode_step_no - 2;

        /*
        		the_robot->EDP_data.force_axis_quantity=1;
         
        		the_robot->EDP_data.relative_force_vector[0]=0.0;
        		the_robot->EDP_data.relative_force_vector[1]=0.0;
        		the_robot->EDP_data.relative_force_vector[2]=1.0;
         
        		the_robot->EDP_data.motion_steps = td.internode_step_no;
        		the_robot->EDP_data.value_in_step_no = td.value_in_step_no;
        		the_robot->EDP_data.ECPtoEDP_force_coordinates[2]=SILA_DOCISKUEDP_OPADANIE_EGG;
         
        		in_state_iteration++;
         
        		for (int j=0; j<6 ; j++)	{
        			the_robot->EDP_data.position_increment[j]=0.0;
        		}
        */
        if (ecp_t.pulse_check())
        {
            in_state_iteration=0;
            next_gen_state = 7;
        }


        break;
    case 7: // krotki ruch w gore

        if (prev_gen_state != gen_state)
        {
            sr_ecp_msg.message("Unoszenie");
        }

        td.interpolation_node_no = 1;
        td.internode_step_no = 1000;
        td.value_in_step_no = td.internode_step_no - 2;
        /*
        		the_robot->EDP_data.force_axis_quantity=0;
         
        		the_robot->EDP_data.motion_steps = td.internode_step_no;
        		the_robot->EDP_data.value_in_step_no = td.value_in_step_no;
         
        		the_robot->EDP_data.position_increment[0]=0.0;
        		the_robot->EDP_data.position_increment[1]=0.0;
        		the_robot->EDP_data.position_increment[2]=0.05;
        		the_robot->EDP_data.position_increment[3]=0.0;
        		the_robot->EDP_data.position_increment[4]=0.0;
        		the_robot->EDP_data.position_increment[5]=0.0;
        */
        next_gen_state = 1;

        break;
    default:

        break;
    }

    prev_gen_state=gen_state;


    return true;
}
; // end: bool y_egg_force_generator::next_step ( )





bias_edp_force_generator::bias_edp_force_generator(ecp_task& _ecp_task):
        ecp_generator (_ecp_task, true)
{}



bool bias_edp_force_generator::first_step ( )
{
    the_robot->EDP_data.instruction_type = SET;
    the_robot->EDP_data.set_type = RMODEL_DV;
    the_robot->EDP_data.set_rmodel_type = FORCE_BIAS;
    ecp_t.set_ecp_reply (TASK_TERMINATED);

    return true;
}

// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
bool bias_edp_force_generator::next_step ( )
{
    return false;
}



// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////
//
// 			y_edge_follow_force_generator
//
// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////


y_edge_follow_force_generator::y_edge_follow_force_generator(ecp_task& _ecp_task, int step):
        ecp_teach_in_generator (_ecp_task), tool_frame(0.0, 0.0, 0.25)
{
    step_no=step;
};


bool y_edge_follow_force_generator::first_step ( )
{

    for (int i=0; i<6; i++)
        delta[i]=0.0;

    create_pose_list_head(emptyps, 0.0, 2, delta);


    td.interpolation_node_no = 1;
    td.internode_step_no = step_no;
    td.value_in_step_no = td.internode_step_no - 2;


    the_robot->EDP_data.instruction_type = GET;
    the_robot->EDP_data.get_type = ARM_DV; // arm - ORYGINAL
    the_robot->EDP_data.set_type = ARM_DV | RMODEL_DV;
    //	the_robot->EDP_data.set_type = ARM_DV;
    the_robot->EDP_data.set_rmodel_type = TOOL_FRAME;
    the_robot->EDP_data.get_rmodel_type = TOOL_FRAME;
    the_robot->EDP_data.set_arm_type = POSE_FORCE_TORQUE_AT_FRAME;
    the_robot->EDP_data.get_arm_type = POSE_FORCE_TORQUE_AT_FRAME;
    the_robot->EDP_data.motion_type = PF_VELOCITY;
    the_robot->EDP_data.motion_steps = td.internode_step_no;
    the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

    tool_frame.get_frame_tab(the_robot->EDP_data.next_tool_frame);

    for(int i=0;i<3;i++)
    {
        the_robot->EDP_data.ECPtoEDP_inertia[i] = FORCE_INERTIA;
        the_robot->EDP_data.ECPtoEDP_inertia[i+3] = TORQUE_INERTIA;
    }


    for(int i=0;i<6;i++)
    {
        the_robot->EDP_data.ECPtoEDP_position_velocity[i] = 0;
        the_robot->EDP_data.ECPtoEDP_force_xyz_torque_xyz[i] = 0;
        //	the_robot->EDP_data.ECPtoEDP_reciprocal_damping[i] = 0.0;
        the_robot->EDP_data.ECPtoEDP_behaviour[i] = UNGUARDED_MOTION;
    }

    the_robot->EDP_data.ECPtoEDP_reciprocal_damping[0] = FORCE_RECIPROCAL_DAMPING;
    the_robot->EDP_data.ECPtoEDP_behaviour[0] = CONTACT;
    // Sila dosciku do rawedzi
    the_robot->EDP_data.ECPtoEDP_force_xyz_torque_xyz[0] = -4;

    return true;
}
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
bool y_edge_follow_force_generator::next_step ( )
{
    // tablice pomocnicze do utworzenia przyrostowej trajektorii ruchu do zapisu do pliku
    double inc_delta[6], tmp_delta[6];
    // static int count;
    // struct timespec start[9];
    if (ecp_t.pulse_check())
    {
        return false;
    }



    // 	wstawienie nowego przyrostu pozyji do przyrostowej trajektorii ruchu do zapisu do pliku
    Homog_matrix tmp_matrix (the_robot->EDP_data.current_beggining_arm_frame);
    tmp_matrix.get_xyz_euler_zyz (inc_delta);

    for (int i=0; i<6;i++)
        inc_delta[i] = -inc_delta[i];


    tmp_matrix.set_frame_tab (the_robot->EDP_data.current_beggining_arm_frame);
    tmp_matrix.get_xyz_euler_zyz (tmp_delta);

    for (int i=0; i<6;i++)
        inc_delta[i]+=tmp_delta[i];

    insert_pose_list_element(emptyps, 0.0, 2, inc_delta);

    // wyznaczenie nowej macierzy referencyjnej i predkosci ruchu

    the_robot->EDP_data.instruction_type = SET_GET;

    the_robot->EDP_data.next_gripper_coordinate = the_robot->EDP_data.current_gripper_coordinate;

    // sprowadzenie sil do ukladu kisci
    Ft_v_vector force_torque = Ft_v_tr (!(Homog_matrix(the_robot->EDP_data.current_present_arm_frame)), Ft_v_tr::FT) *
                               Ft_v_vector (the_robot->EDP_data.EDPtoECP_force_xyz_torque_xyz);


    double wx = force_torque[0];
    double wy = force_torque[1];

    double v = sqrt (wx*wx + wy*wy);

    double s_alfa = wy / v;
    double c_alfa = - wx / v;

    the_robot->EDP_data.ECPtoEDP_position_velocity[1] = -0.002*v;
    //	the_robot->EDP_data.ECPtoEDP_position_velocity[1] = 0.0;

    // basic_rot_frame = Homog_matrix(c_alfa, s_alfa, 0.0,	-s_alfa, c_alfa, 0.0,	0.0, 0.0, 1,	0.0, 0.0, 0.0);
    basic_rot_frame = Homog_matrix(c_alfa, -s_alfa, 0.0, 0.0, 	 s_alfa, c_alfa, 0.0, 0.0,		0.0, 0.0, 1, 0.0);


    // dodatkowa macierz obracajaca kierunek wywieranej sily tak aby stabilizowac jej wartosc
    double alfa_r = 0.2*(v-4);
    double s_alfa_r = sin (alfa_r);
    double c_alfa_r = cos (alfa_r);

    // ex_rot_frame = Homog_matrix(c_alfa_r, s_alfa_r, 0.0,	-s_alfa_r, c_alfa_r, 0.0,	0.0, 0.0, 1,	0.0, 0.0, 0.0);
    ex_rot_frame = Homog_matrix(c_alfa_r, -s_alfa_r, 0.0, 0.0,		s_alfa_r,  c_alfa_r, 0.0,	0.0,		0.0, 0.0, 1, 0.0);

    // obrocenie pierwotnej macierzy
    basic_rot_frame = basic_rot_frame  * ex_rot_frame;


    basic_rot_frame = !basic_rot_frame;

    tool_frame = tool_frame * basic_rot_frame;
    // basic_rot_frame.set_translation_vector(0, 0, 0.25);

    tool_frame.get_frame_tab(the_robot->EDP_data.next_tool_frame);



    //	ECPtoEDP_ref_frame.get_frame_tab(the_robot->EDP_data.ECPtoEDP_reference_frame);

    /*
    	the_robot->EDP_data.ECPtoEDP_reference_frame[0][0] = c_alfa;
    the_robot->EDP_data.ECPtoEDP_reference_frame[0][1] = s_alfa;

    the_robot->EDP_data.ECPtoEDP_reference_frame[1][0] = -s_alfa;
    the_robot->EDP_data.ECPtoEDP_reference_frame[1][1] = c_alfa;
    */

    printf("sensor: x: %+d, y: %+d, v:%+d, %f\n", (int) round(wx),  (int) round(wy), (int) round(v),
           atan2(s_alfa, c_alfa)*DEGREES_TO_RADIANS);



    return true;

}
; // end: bool y_edge_follow_force_generator::next_step ( )



// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////
//
// 			y_drawing_teach_in_force_generator
//
// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////


y_drawing_teach_in_force_generator::y_drawing_teach_in_force_generator(ecp_task& _ecp_task, int step):
ecp_teach_in_generator (_ecp_task)
{
    step_no=step;
};

bool y_drawing_teach_in_force_generator::first_step ( )
{

    if (teach_or_move == YG_MOVE)
    {


        for (int i=0; i<6; i++)
            delta[i]=0.0;

        initiate_pose_list();

        td.interpolation_node_no = 1;
        td.internode_step_no = step_no;
        td.value_in_step_no = td.internode_step_no - 2;



        the_robot->EDP_data.instruction_type = GET;
        the_robot->EDP_data.get_type = ARM_DV;
        the_robot->EDP_data.set_type = ARM_DV;

        /*
        		the_robot->EDP_data.force_axis_quantity=1;
         
        		the_robot->EDP_data.relative_force_vector[0]=0.0;
        		the_robot->EDP_data.relative_force_vector[1]=0.0;
        		the_robot->EDP_data.relative_force_vector[2]=1.0;
         
        		normalize_vector(the_robot->EDP_data.relative_force_vector,the_robot->EDP_data.relative_force_vector,3);
         
        		the_robot->EDP_data.set_arm_type = POSE_FORCE_LINEAR;
        		the_robot->EDP_data.get_arm_type = POSE_FORCE_LINEAR;
        		the_robot->EDP_data.motion_type = ABSOLUTE;
        		the_robot->EDP_data.motion_steps = td.internode_step_no;
        		the_robot->EDP_data.value_in_step_no = td.value_in_step_no;
         
        		the_robot->EDP_data.ECPtoEDP_force_coordinates[2]=SILA_DOCISKUEDP;
         
        		the_robot->EDP_data.dyslocation_matrix[0][0]=1;
        		the_robot->EDP_data.dyslocation_matrix[1][1]=1;
        		the_robot->EDP_data.dyslocation_matrix[2][2]=1;
        		the_robot->EDP_data.dyslocation_matrix[3][3]=0;
        		the_robot->EDP_data.dyslocation_matrix[4][4]=0;
        		the_robot->EDP_data.dyslocation_matrix[5][5]=0;
        */

        return true;

        // UCZENIE

    }
    else if (teach_or_move == YG_TEACH)
    {



        for (int i=0; i<6; i++)
            delta[i]=0.0;

        create_pose_list_head(emptyps,0.0,delta);

        td.interpolation_node_no = 1;
        td.internode_step_no = step_no;
        td.value_in_step_no = td.internode_step_no - 2;


        the_robot->EDP_data.instruction_type = GET;
        the_robot->EDP_data.get_type = ARM_DV;
        the_robot->EDP_data.set_type = ARM_DV;
        /*
        		the_robot->EDP_data.force_move_mode=2;
        		the_robot->EDP_data.position_set_mode=1; // przyrostowo
         
        		the_robot->EDP_data.force_axis_quantity=3;
         
        		the_robot->EDP_data.set_arm_type = POSE_FORCE_LINEAR;
        		the_robot->EDP_data.get_arm_type = POSE_FORCE_LINEAR;
        		the_robot->EDP_data.motion_type = ABSOLUTE;
        		the_robot->EDP_data.motion_steps = td.internode_step_no;
        		the_robot->EDP_data.value_in_step_no = td.value_in_step_no;
         
        		the_robot->EDP_data.ECPtoEDP_force_coordinates[0]=0;
        		the_robot->EDP_data.ECPtoEDP_force_coordinates[1]=0;
        		the_robot->EDP_data.ECPtoEDP_force_coordinates[2]=SILA_DOCISKUEDP;
         
        		for (int j=0; j<6 ; j++)	{
        			the_robot->EDP_data.position_increment[j]=0.0;
        		}
        		// zerowy przyrost pozycji
         
        		the_robot->EDP_data.dyslocation_matrix[0][0]=1;
        		the_robot->EDP_data.dyslocation_matrix[1][1]=1;
        		the_robot->EDP_data.dyslocation_matrix[2][2]=1;
        		the_robot->EDP_data.dyslocation_matrix[3][3]=0;
        		the_robot->EDP_data.dyslocation_matrix[4][4]=0;
        		the_robot->EDP_data.dyslocation_matrix[5][5]=0;
        */


        return true;


    }
    return true;

}
; // end: bool y_drawing_teach_in_force_generator::first_step ( )
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
bool y_drawing_teach_in_force_generator::next_step ( )
{

    if (teach_or_move == YG_MOVE)
    {


        ecp_taught_in_pose tip;      // Nauczona pozycja

        if (!(is_pose_list_element()))
        { // Koniec odcinka
            return false;
            // flush_pose_list();

        }


        // Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
        the_robot->EDP_data.instruction_type = SET;

        // Obliczenie zadanej pozycji posredniej w tym kroku ruchu
        // (okreslenie kolejnego wezla interpolacji)

        get_pose (tip);

        delta[0]=tip.coordinates[0];
        delta[1]=tip.coordinates[1];
        /*
        	the_robot->EDP_data.position_increment[0]=delta[0];
        	the_robot->EDP_data.position_increment[1]=delta[1];
        	the_robot->EDP_data.position_increment[2]=0.0;
        	the_robot->EDP_data.position_increment[3]=0.0;
        	the_robot->EDP_data.position_increment[4]=0.0;
        	the_robot->EDP_data.position_increment[5]=0.0;
        */
        next_pose_list_ptr ();




        return true;

        // UCZENIE

    }
    else if (teach_or_move == YG_TEACH)
    {

        int i; // licznik kolejnych wspolrzednych wektora [0..6]

        double inc_delta[6]={0.0,0.0,0.0,0.0,0.0,0.0};

        if (ecp_t.pulse_check())
        { // Koniec odcinka
            initiate_pose_list();
            return false;
        }


        for (i=0; i<6;i++)
        {
            inc_delta[i]=-the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i];
        }



        for (i=0; i<6;i++)
        {
            inc_delta[i]+=the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i];
        }

        // Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
        the_robot->EDP_data.instruction_type = SET;


        insert_pose_list_element(emptyps,0.0,inc_delta);



        return true;

    }

    return true;

}
; // end: bool y_drawing_teach_in_force_generator::next_step ( )





// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////
//
// 			y_advanced_drawing_teach_in_force_generator
//
// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////


y_advanced_drawing_teach_in_force_generator::y_advanced_drawing_teach_in_force_generator(ecp_task& _ecp_task, int step):
        y_drawing_teach_in_force_generator (_ecp_task, step)
{}
;

bool y_advanced_drawing_teach_in_force_generator::first_step ( )
{

    if (teach_or_move == YG_MOVE)
    {



        for (int i=0; i<6; i++)
            delta[i]=0.0;

        initiate_pose_list();

        gen_state = next_gen_state = 4; // jazda w powietrzu
        prev_gen_state = 0;


        td.interpolation_node_no = 1;
        td.internode_step_no = step_no;
        td.value_in_step_no = td.internode_step_no - 2;



        the_robot->EDP_data.instruction_type = GET;
        the_robot->EDP_data.get_type = ARM_DV;
        the_robot->EDP_data.set_type = ARM_DV;


        /*
        		the_robot->EDP_data.force_axis_quantity=1;
         
        		the_robot->EDP_data.relative_force_vector[0]=0.0;
        		the_robot->EDP_data.relative_force_vector[1]=0.0;
        		the_robot->EDP_data.relative_force_vector[2]=1.0;
         
        		normalize_vector(the_robot->EDP_data.relative_force_vector,the_robot->EDP_data.relative_force_vector,3);
         
        		the_robot->EDP_data.set_arm_type = POSE_FORCE_LINEAR;
        		the_robot->EDP_data.get_arm_type = POSE_FORCE_LINEAR;
        		the_robot->EDP_data.motion_type = ABSOLUTE;
        		the_robot->EDP_data.motion_steps = td.internode_step_no;
        		the_robot->EDP_data.value_in_step_no = td.value_in_step_no;
         
        		the_robot->EDP_data.ECPtoEDP_force_coordinates[2]=SILA_DOCISKUEDP;
         
        		the_robot->EDP_data.dyslocation_matrix[0][0]=1;
        		the_robot->EDP_data.dyslocation_matrix[1][1]=1;
        		the_robot->EDP_data.dyslocation_matrix[2][2]=1;
        		the_robot->EDP_data.dyslocation_matrix[3][3]=0;
        		the_robot->EDP_data.dyslocation_matrix[4][4]=0;
        		the_robot->EDP_data.dyslocation_matrix[5][5]=0;
        */

        return true;

        // UCZENIE

    }
    else if (teach_or_move == YG_TEACH)
    {


        // zerowanie odczytow
        (sensor_m.begin())->second->to_vsp.parameters=1;
        (sensor_m.begin())->second->configure_sensor();

        (sensor_m.begin())->second->to_vsp.parameters=4;
        (sensor_m.begin())->second->configure_sensor();

        gen_state = 0; // nie zapisuje trajektorii dopoki nie osiagnie chociaz raz podloza

        for (int i=0; i<6; i++)
            delta[i]=0.0;

        create_pose_list_head(emptyps, 0.0, 2, delta);


        td.interpolation_node_no = 1;
        td.internode_step_no = step_no;
        td.value_in_step_no = td.internode_step_no - 2;


        the_robot->EDP_data.instruction_type = GET;
        the_robot->EDP_data.get_type = ARM_DV;
        the_robot->EDP_data.set_type = ARM_DV;
        /*
        		the_robot->EDP_data.force_move_mode=2;
        		the_robot->EDP_data.position_set_mode=1; // przyrostowo
         
        		the_robot->EDP_data.force_axis_quantity=3;
         
        		the_robot->EDP_data.set_arm_type = POSE_FORCE_LINEAR;
        		the_robot->EDP_data.get_arm_type = POSE_FORCE_LINEAR;
        		the_robot->EDP_data.motion_type = ABSOLUTE;
        		the_robot->EDP_data.motion_steps = td.internode_step_no;
        		the_robot->EDP_data.value_in_step_no = td.value_in_step_no;
         
        		the_robot->EDP_data.ECPtoEDP_force_coordinates[0]=0;
        		the_robot->EDP_data.ECPtoEDP_force_coordinates[1]=0;
        		the_robot->EDP_data.ECPtoEDP_force_coordinates[2]=SILA_DOCISKUEDP;
         
        		for (int j=0; j<6 ; j++)	{
        		the_robot->EDP_data.position_increment[j]=0.0;}
        		// zerowy przyrost pozycji
         
        		the_robot->EDP_data.dyslocation_matrix[0][0]=1;
        		the_robot->EDP_data.dyslocation_matrix[1][1]=1;
        		the_robot->EDP_data.dyslocation_matrix[2][2]=1;
        		the_robot->EDP_data.dyslocation_matrix[3][3]=0;
        		the_robot->EDP_data.dyslocation_matrix[4][4]=0;
        		the_robot->EDP_data.dyslocation_matrix[5][5]=0;
        */


        return true;

    }

    return true;
}
; // end: bool y_advanced_drawing_teach_in_force_generator::first_step ( )
// --------------------------------------------------------------------------



// --------------------------------------------------------------------------
bool y_advanced_drawing_teach_in_force_generator::next_step ( )
{

    if (teach_or_move == YG_MOVE)
    {


        ecp_taught_in_pose tip;      // Nauczona pozycja

        if (!(is_pose_list_element()))
        { // Koniec odcinka
            (sensor_m.begin())->second->to_vsp.parameters = 6;
            (sensor_m.begin())->second->configure_sensor();
            return false;

            // flush_pose_list();
            //		return false;
        }


        // Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
        the_robot->EDP_data.instruction_type = SET;

        get_pose (tip);

        gen_state=next_gen_state;

        switch (gen_state)
        {
        case 0:
            break;
        case 1:
            break;
        case 2:  // powierzchnia

            if (prev_gen_state != gen_state)
            {
                sr_ecp_msg.message("ECP Powierzchnia");
            }

            td.interpolation_node_no = 1;
            td.internode_step_no = step_no;
            td.value_in_step_no = td.internode_step_no - 2;
            /*
            		the_robot->EDP_data.force_axis_quantity=1;
             
            		the_robot->EDP_data.motion_steps = td.internode_step_no;
            		the_robot->EDP_data.value_in_step_no = td.value_in_step_no;
             
            		// inkrementacja numeru iteracji dla biezacego stanu
            		in_state_iteration++;
             
            		#define ST2_LOW_SEGMENT 20
             
            		if (in_state_iteration < ST2_LOW_SEGMENT)
            		{
            			the_robot->EDP_data.ECPtoEDP_force_coordinates[2]= (short) (MIN_SILA_DOCISKUEDP +
            				(in_state_iteration)*(SILA_DOCISKUEDP - MIN_SILA_DOCISKUEDP) /	(ST2_LOW_SEGMENT));
            		} else
            		{
            			the_robot->EDP_data.ECPtoEDP_force_coordinates[2]=SILA_DOCISKUEDP;
            		}
             
            		the_robot->EDP_data.position_increment[0]=tip.coordinates[0];
            		the_robot->EDP_data.position_increment[1]=tip.coordinates[1];
            		the_robot->EDP_data.position_increment[2]=0.0;
            		the_robot->EDP_data.position_increment[3]=0.0;
            		the_robot->EDP_data.position_increment[4]=0.0;
            		the_robot->EDP_data.position_increment[5]=0.0;
            */
            if ((tip.extra_info == 3)||(tip.extra_info == 4)||(tip.extra_info == 5))
            {
                // nauczona trajektoria przeszla w unoszenie
                next_gen_state = 3;
                in_state_iteration = 0;
            }
            else
            {
                next_pose_list_ptr ();
            }

            break;

        case 3:  // unoszenie
            if (prev_gen_state != gen_state)
            {
                sr_ecp_msg.message("ECP Unoszenie");
            }

            td.interpolation_node_no = 1;
            td.internode_step_no = 500;
            td.value_in_step_no = td.internode_step_no - 2;
            /*
            		the_robot->EDP_data.force_axis_quantity=0;
             
            		the_robot->EDP_data.motion_steps = td.internode_step_no;
            		the_robot->EDP_data.value_in_step_no = td.value_in_step_no;
             
            		the_robot->EDP_data.position_increment[0]=0.0;
            		the_robot->EDP_data.position_increment[1]=0.0;
            		the_robot->EDP_data.position_increment[2]=0.01;
            		the_robot->EDP_data.position_increment[3]=0.0;
            		the_robot->EDP_data.position_increment[4]=0.0;
            		the_robot->EDP_data.position_increment[5]=0.0;
            */
            next_gen_state = 4;

            break;

        case 4: // uniesienie

            if (prev_gen_state != gen_state)
            {
                sr_ecp_msg.message("ECP Uniesienie");
            }

            td.interpolation_node_no = 1;
            td.internode_step_no = step_no;
            td.value_in_step_no = td.internode_step_no - 2;
            /*
            		the_robot->EDP_data.force_axis_quantity = 0;
             
            		the_robot->EDP_data.motion_steps = td.internode_step_no;
            		the_robot->EDP_data.value_in_step_no = td.value_in_step_no;
             
            		the_robot->EDP_data.position_increment[0]=tip.coordinates[0];
            		the_robot->EDP_data.position_increment[1]=tip.coordinates[1];
            		the_robot->EDP_data.position_increment[2]=0.0;
            		the_robot->EDP_data.position_increment[3]=0.0;
            		the_robot->EDP_data.position_increment[4]=0.0;
            		the_robot->EDP_data.position_increment[5]=0.0;
            */
            if (tip.extra_info == 2)
            { // nauczona trajektoria osiagnela powierzchnie
                next_gen_state = 5;
                (sensor_m.begin())->second->to_vsp.parameters = 5;
                (sensor_m.begin())->second->configure_sensor();
            }
            else
            {
                next_pose_list_ptr ();
            }
            break;

        case 5: // opuszczanie

            if (prev_gen_state != gen_state)
            {
                sr_ecp_msg.message("ECP Opuszczanie");
            }

            if ((sensor_m.begin())->second->image.force.event_type == 2)
            {
                // czujnik wyczul powierzchnie
                next_gen_state = 6;
                in_state_iteration=0;
            }
            else
            {

                td.interpolation_node_no = 1;
                td.internode_step_no = step_no;
                td.value_in_step_no = td.internode_step_no - 2;
                /*
                the_robot->EDP_data.force_axis_quantity=1;
                the_robot->EDP_data.motion_steps = td.internode_step_no;
                the_robot->EDP_data.value_in_step_no = td.value_in_step_no;
                the_robot->EDP_data.ECPtoEDP_force_coordinates[2]=SILA_DOCISKUEDP_OPADANIE;

                the_robot->EDP_data.position_increment[0]=0.0;
                the_robot->EDP_data.position_increment[1]=0.0;
                the_robot->EDP_data.position_increment[2]=0.0;
                the_robot->EDP_data.position_increment[3]=0.0;
                the_robot->EDP_data.position_increment[4]=0.0;
                the_robot->EDP_data.position_increment[5]=0.0;
                */
            }

            break;
        case 6: // zetkniecie z kartka

            if (prev_gen_state != gen_state)
            {
                sr_ecp_msg.message("ECP Zetkniecie");
            }

            in_state_iteration++;

            td.interpolation_node_no = 1;
            td.internode_step_no = step_no;
            td.value_in_step_no = td.internode_step_no - 2;
            /*
            the_robot->EDP_data.force_axis_quantity=1;
            the_robot->EDP_data.motion_steps = td.internode_step_no;
            the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

            the_robot->EDP_data.position_increment[0]=0.0;
            the_robot->EDP_data.position_increment[1]=0.0;
            the_robot->EDP_data.position_increment[2]=0.0;
            the_robot->EDP_data.position_increment[3]=0.0;
            the_robot->EDP_data.position_increment[4]=0.0;
            the_robot->EDP_data.position_increment[5]=0.0;

            #define ST6_LOW_SEGMENT 100

            // dobor sily docisku - chcemy ladnie uderzyc w powierzchnie
            if (in_state_iteration<ST6_LOW_SEGMENT)
        {
            	the_robot->EDP_data.ECPtoEDP_force_coordinates[2]=MIN_SILA_DOCISKUEDP;
        } else
        {
            	next_gen_state = 2;
            	in_state_iteration=0;
        }
            */
            break;
        default:

            break;
        }

        prev_gen_state=gen_state;




        return true;

        // UCZENIE

    }
    else if (teach_or_move == YG_TEACH)
    {

        int i; // licznik kolejnych wspolrzednych wektora [0..6]

        double inc_delta[6]={0.0,0.0,0.0,0.0,0.0,0.0};

        if (ecp_t.pulse_check())
        { // Koniec odcinka
            initiate_pose_list();
            return false;
        }


        for (i=0; i<6;i++)
        {
            inc_delta[i]=-the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i];
        }



        for (i=0; i<6;i++)
        {
            inc_delta[i]+=the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i];
        }

        // Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
        the_robot->EDP_data.instruction_type = SET;

        if ((sensor_m.begin())->second->image.force.event_type==2)
        {
            gen_state = 1;
        }

        if (gen_state == 1)
        {
            insert_pose_list_element(emptyps, 0.0, (sensor_m.begin())->second->image.force.event_type, inc_delta);
        }



        return true;

    }

    return true;

}
; // end: bool y_advanced_drawing_teach_in_force_generator::next_step ( )
// --------------------------------------------------------------------------



ecp_tff_nose_run_generator::ecp_tff_nose_run_generator(ecp_task& _ecp_task, int step):
        ecp_generator (_ecp_task, true)
{
    step_no = step;
    // domyslnie wszytkie osie podatne a pulse_check nieaktywne
    configure (true, true, true, true, true, true, false);
};

// decyduje ktore osie maja byc podatne
void ecp_tff_nose_run_generator::configure(bool x, bool y, bool z, bool g, bool b, bool a, bool pulse_check_activated_l)
{

    selection_vector_l[0] = x;
    selection_vector_l[1] = y;
    selection_vector_l[2] = z;
    selection_vector_l[3] = g;
    selection_vector_l[4] = b;
    selection_vector_l[5] = a;
    pulse_check_activated = pulse_check_activated_l;
}

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool ecp_tff_nose_run_generator::first_step ()
{
    // Generacja trajektorii prostoliniowej o zadany przyrost polozenia i oreintacji
    // Funkcja zwraca false gdy koniec generacji trajektorii
    // Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
    // cout << "first_step" << endl;

    td.interpolation_node_no = 1;
    td.internode_step_no = step_no;
    td.value_in_step_no = td.internode_step_no - 2;

    Homog_matrix tool_frame(0.0, 0.0, 0.25);
    tool_frame.get_frame_tab(the_robot->EDP_data.next_tool_frame);


    the_robot->EDP_data.instruction_type = GET;
    the_robot->EDP_data.get_type = ARM_DV; // arm - ORYGINAL
    the_robot->EDP_data.set_type = ARM_DV | RMODEL_DV;
    //	the_robot->EDP_data.set_type = ARM_DV;
    the_robot->EDP_data.set_rmodel_type = TOOL_FRAME;
    the_robot->EDP_data.get_rmodel_type = TOOL_FRAME;
    the_robot->EDP_data.set_arm_type = POSE_FORCE_TORQUE_AT_FRAME;
    the_robot->EDP_data.get_arm_type = POSE_FORCE_TORQUE_AT_FRAME;
    the_robot->EDP_data.motion_type = PF_VELOCITY;
    the_robot->EDP_data.motion_steps = td.internode_step_no;
    the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

    for (int i=0;i<6;i++)
    {
        the_robot->EDP_data.ECPtoEDP_position_velocity[i] = 0;
        the_robot->EDP_data.ECPtoEDP_force_xyz_torque_xyz[i] = 0;
        //		the_robot->EDP_data.selection_vector[i] = selection_vector_l[i];
        //	the_robot->EDP_data.selection_vector[i] = POSE_SV_AX;
    }

    for (int i=0;i<3;i++)
    {
        if (selection_vector_l[i])
        {
            the_robot->EDP_data.ECPtoEDP_reciprocal_damping[i] = FORCE_RECIPROCAL_DAMPING;
            the_robot->EDP_data.ECPtoEDP_behaviour[i] = CONTACT;
        }
        else
        {
            the_robot->EDP_data.ECPtoEDP_behaviour[i] = UNGUARDED_MOTION;
        }

        if (selection_vector_l[i+3])
        {
            the_robot->EDP_data.ECPtoEDP_reciprocal_damping[i+3] = TORQUE_RECIPROCAL_DAMPING;
            the_robot->EDP_data.ECPtoEDP_behaviour[i+3] = CONTACT;
        }
        else
        {
            //	the_robot->EDP_data.ECPtoEDP_reciprocal_damping[i+3] = 0;
            the_robot->EDP_data.ECPtoEDP_behaviour[i+3] = UNGUARDED_MOTION;
        }

        the_robot->EDP_data.ECPtoEDP_inertia[i] = FORCE_INERTIA;
        the_robot->EDP_data.ECPtoEDP_inertia[i+3] = TORQUE_INERTIA;

    }



    return true;
}
; // end: ecp_tff_nose_run_generator::first_step()

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool ecp_tff_nose_run_generator::next_step ()
{
    // Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
    // Funkcja zwraca false gdy koniec generacji trajektorii
    // Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
    // UWAGA: dzialamy na jednoelementowej liscie robotow
    // cout << "next_step" << endl;

    if (pulse_check_activated && ecp_t.pulse_check())
    { // Koniec odcinka
        //	ecp_t.set_ecp_reply (TASK_TERMINATED);

        return false;
    }


    // Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
    the_robot->EDP_data.instruction_type = SET_GET;

    // Obliczenie zadanej pozycji posredniej w tym kroku ruchu

    if (node_counter==1)
    {
        the_robot->EDP_data.next_gripper_coordinate = the_robot->EDP_data.current_gripper_coordinate;
    }


    return true;

}
; // end: bool ecp_tff_nose_run_generator::next_step ()



ecp_tff_rubik_grab_generator::ecp_tff_rubik_grab_generator(ecp_task& _ecp_task, int step):
        ecp_generator (_ecp_task, true)
{
    step_no = step;
};


void ecp_tff_rubik_grab_generator::configure(double l_goal_position, double l_position_increment,
        int l_min_node_counter, bool l_both_axes_running)
{
    goal_position = l_goal_position;
    position_increment = l_position_increment;
    min_node_counter = l_min_node_counter;
    both_axes_running = l_both_axes_running;
};


// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool ecp_tff_rubik_grab_generator::first_step ()
{
    // Generacja trajektorii prostoliniowej o zadany przyrost polozenia i oreintacji
    // Funkcja zwraca false gdy koniec generacji trajektorii
    // Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
    // cout << "first_step" << endl;

    td.interpolation_node_no = 1;
    td.internode_step_no = step_no;
    td.value_in_step_no = td.internode_step_no - 2;

    Homog_matrix tool_frame(0.0, 0.0, 0.25);
    tool_frame.get_frame_tab(the_robot->EDP_data.next_tool_frame);


    the_robot->EDP_data.instruction_type = GET;
    the_robot->EDP_data.get_type = ARM_DV; // arm - ORYGINAL
    the_robot->EDP_data.set_type = ARM_DV | RMODEL_DV;
    //	the_robot->EDP_data.set_type = ARM_DV;
    the_robot->EDP_data.set_rmodel_type = TOOL_FRAME;
    the_robot->EDP_data.get_rmodel_type = TOOL_FRAME;
    the_robot->EDP_data.set_arm_type = POSE_FORCE_TORQUE_AT_FRAME;
    the_robot->EDP_data.get_arm_type = POSE_FORCE_TORQUE_AT_FRAME;
    the_robot->EDP_data.motion_type = PF_VELOCITY;
    the_robot->EDP_data.motion_steps = td.internode_step_no;
    the_robot->EDP_data.value_in_step_no = td.value_in_step_no;


    for(int i=0;i<6;i++)
    {
        the_robot->EDP_data.ECPtoEDP_force_xyz_torque_xyz[i] = 0;
        the_robot->EDP_data.ECPtoEDP_position_velocity[i] = 0;
    }

    for(int i=0;i<3;i++)
    {
        the_robot->EDP_data.ECPtoEDP_inertia[i] = FORCE_INERTIA;
        the_robot->EDP_data.ECPtoEDP_inertia[i+3] = TORQUE_INERTIA;
    }


    if (both_axes_running)
        for(int i=0;i<2;i++)
        {
            the_robot->EDP_data.ECPtoEDP_reciprocal_damping[i] = FORCE_RECIPROCAL_DAMPING;
            the_robot->EDP_data.ECPtoEDP_behaviour[i] = CONTACT;
        }
    else
    {
        the_robot->EDP_data.ECPtoEDP_reciprocal_damping[1] = FORCE_RECIPROCAL_DAMPING;
        the_robot->EDP_data.ECPtoEDP_behaviour[1] = CONTACT;
        the_robot->EDP_data.ECPtoEDP_behaviour[0] = UNGUARDED_MOTION;
    }

    for(int i=2;i<6;i++)
    {
        the_robot->EDP_data.ECPtoEDP_behaviour[i] = UNGUARDED_MOTION;
    }




    return true;
}
; // end: ecp_tff_rubik_grab_generator::first_step()



// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool ecp_tff_rubik_grab_generator::next_step ()
{
    // Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
    // Funkcja zwraca false gdy koniec generacji trajektorii
    // Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
    // UWAGA: dzialamy na jednoelementowej liscie robotow
    //	  cout << "next_step" << endl;



    // Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
    the_robot->EDP_data.instruction_type = SET_GET;

    // Obliczenie zadanej pozycji posredniej w tym kroku ruchu

    if (node_counter==1)
    {
        the_robot->EDP_data.next_gripper_coordinate = the_robot->EDP_data.current_gripper_coordinate;
    }


    if ((the_robot->EDP_data.next_gripper_coordinate > goal_position) || (node_counter < min_node_counter))
        the_robot->EDP_data.next_gripper_coordinate -= position_increment;
    else
    {
        ecp_t.ecp_termination_notice();
        return false;
    }


    return true;

}
; // end: bool ecp_tff_rubik_grab_generator::next_step ()




ecp_tff_rubik_face_rotate_generator::ecp_tff_rubik_face_rotate_generator(ecp_task& _ecp_task, int step):
        ecp_generator (_ecp_task, true)
{
    step_no = step;
};

void ecp_tff_rubik_face_rotate_generator::configure(double l_turn_angle)
{
    turn_angle = l_turn_angle;
};


// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool ecp_tff_rubik_face_rotate_generator::first_step ()
{
    // Generacja trajektorii prostoliniowej o zadany przyrost polozenia i oreintacji
    // Funkcja zwraca false gdy koniec generacji trajektorii
    // Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
    // cout << "first_step" << endl;


    td.interpolation_node_no = 1;
    td.internode_step_no = step_no;
    td.value_in_step_no = td.internode_step_no - 2;

    Homog_matrix tool_frame(0.0, 0.0, 0.25);
    tool_frame.get_frame_tab(the_robot->EDP_data.next_tool_frame);


    the_robot->EDP_data.instruction_type = GET;
    the_robot->EDP_data.get_type = ARM_DV; // arm - ORYGINAL
    the_robot->EDP_data.set_type = ARM_DV | RMODEL_DV;
    //	the_robot->EDP_data.set_type = ARM_DV;
    the_robot->EDP_data.set_rmodel_type = TOOL_FRAME;
    the_robot->EDP_data.get_rmodel_type = TOOL_FRAME;
    the_robot->EDP_data.set_arm_type = POSE_FORCE_TORQUE_AT_FRAME;
    the_robot->EDP_data.get_arm_type = POSE_FORCE_TORQUE_AT_FRAME;
    the_robot->EDP_data.motion_type = PF_VELOCITY;
    the_robot->EDP_data.motion_steps = td.internode_step_no;
    the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

    for(int i=0;i<6;i++)
    {
        the_robot->EDP_data.ECPtoEDP_force_xyz_torque_xyz[i] = 0;
        the_robot->EDP_data.ECPtoEDP_position_velocity[i] = 0;
    }

    for (int i=0;i<3;i++)
    {
        the_robot->EDP_data.ECPtoEDP_inertia[i] = FORCE_INERTIA;
        the_robot->EDP_data.ECPtoEDP_inertia[i+3] = TORQUE_INERTIA;
    }

    the_robot->EDP_data.ECPtoEDP_reciprocal_damping[5] = TORQUE_RECIPROCAL_DAMPING/4;
    the_robot->EDP_data.ECPtoEDP_behaviour[5] = CONTACT;

    if (-0.1 < turn_angle && turn_angle < 0.1)
    {
        for(int i=0;i<6;i++)
            the_robot->EDP_data.ECPtoEDP_behaviour[i] = UNGUARDED_MOTION;
    }
    else
    {
        for(int i=0;i<3;i++)
        {
            the_robot->EDP_data.ECPtoEDP_reciprocal_damping[i] = FORCE_RECIPROCAL_DAMPING;
            the_robot->EDP_data.ECPtoEDP_behaviour[i] = CONTACT;
        }
        for(int i=3;i<5;i++)
            the_robot->EDP_data.ECPtoEDP_behaviour[i] = UNGUARDED_MOTION;
        the_robot->EDP_data.ECPtoEDP_reciprocal_damping[5] = TORQUE_RECIPROCAL_DAMPING;
        the_robot->EDP_data.ECPtoEDP_behaviour[5] = CONTACT;
        if(turn_angle > 0.0)
            the_robot->EDP_data.ECPtoEDP_force_xyz_torque_xyz[5] = -5;
        if(turn_angle < 0.0)
            the_robot->EDP_data.ECPtoEDP_force_xyz_torque_xyz[5] = 5;
    }



    return true;
}
; // end: ecp_tff_rubik_face_rotate_generator::first_step()



// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool ecp_tff_rubik_face_rotate_generator::next_step ()
{
    // Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
    // Funkcja zwraca false gdy koniec generacji trajektorii
    // Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
    // UWAGA: dzialamy na jednoelementowej liscie robotow
    // cout << "next_step" << endl;



    // Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
    the_robot->EDP_data.instruction_type = SET_GET;

    // Obliczenie zadanej pozycji posredniej w tym kroku ruchu

    if (node_counter==1)
    {
        the_robot->EDP_data.next_gripper_coordinate = the_robot->EDP_data.current_gripper_coordinate;
        if(turn_angle < -0.1 || 0.1 < turn_angle)
        {
            Homog_matrix frame(the_robot->EDP_data.current_beggining_arm_frame);
            double xyz_eul_zyz[6];
            frame.get_xyz_euler_zyz(xyz_eul_zyz);
            double angle_to_move = (turn_angle / 180.0) * M_PI;
            if (xyz_eul_zyz[5] + angle_to_move < -M_PI)
            {
                stored_gamma = 2 * M_PI + xyz_eul_zyz[5] + angle_to_move;
                range_change = true;
            }
            else if(xyz_eul_zyz[5] + angle_to_move > M_PI)
            {
                stored_gamma = -2 * M_PI + xyz_eul_zyz[5] + angle_to_move;
                range_change = true;
            }
            else
            {
                stored_gamma = xyz_eul_zyz[5] + angle_to_move;
                range_change = false;
            }
        }
    }
    else
    {


        if (turn_angle < -0.1 || 0.1 < turn_angle)
        {
            Homog_matrix current_frame(the_robot->EDP_data.current_predicted_arm_frame);
            double xyz_eul_zyz[6];
            current_frame.get_xyz_euler_zyz(xyz_eul_zyz);
            double current_gamma = xyz_eul_zyz[5];
            if(!range_change)
            {
                if((	turn_angle < 0.0 && stored_gamma > current_gamma) || (
                            turn_angle > 0.0 && stored_gamma < current_gamma))
                {
                    ecp_t.ecp_termination_notice();
                    return false;
                }
            }
            else
            {
                if((	turn_angle < 0.0 && stored_gamma < current_gamma) || (
                            turn_angle > 0.0 && stored_gamma > current_gamma))
                {
                    range_change = false;
                }
            }
        }


    }


    return true;

}
; // end: bool ecp_tff_rubik_face_rotate_generator::next_step ()





ecp_tff_gripper_approach_generator::ecp_tff_gripper_approach_generator(ecp_task& _ecp_task, int step):
        ecp_generator (_ecp_task, true)
{
    step_no = step;
};


void ecp_tff_gripper_approach_generator::	configure (double l_speed, int l_motion_time)
{
    speed = l_speed;
    motion_time = l_motion_time;
};


// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool ecp_tff_gripper_approach_generator::first_step ()
{
    // Generacja trajektorii prostoliniowej o zadany przyrost polozenia i oreintacji
    // Funkcja zwraca false gdy koniec generacji trajektorii
    // Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
    // cout << "first_step" << endl;




    td.interpolation_node_no = 1;
    td.internode_step_no = step_no;
    td.value_in_step_no = td.internode_step_no - 2;

    Homog_matrix tool_frame(0.0, 0.0, 0.25);
    tool_frame.get_frame_tab(the_robot->EDP_data.next_tool_frame);


    the_robot->EDP_data.instruction_type = GET;
    the_robot->EDP_data.get_type = ARM_DV; // arm - ORYGINAL
    the_robot->EDP_data.set_type = ARM_DV | RMODEL_DV;
    //	the_robot->EDP_data.set_type = ARM_DV;
    the_robot->EDP_data.set_rmodel_type = TOOL_FRAME;
    the_robot->EDP_data.get_rmodel_type = TOOL_FRAME;
    the_robot->EDP_data.set_arm_type = POSE_FORCE_TORQUE_AT_FRAME;
    the_robot->EDP_data.get_arm_type = POSE_FORCE_TORQUE_AT_FRAME;
    the_robot->EDP_data.motion_type = PF_VELOCITY;
    the_robot->EDP_data.motion_steps = td.internode_step_no;
    the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

    for(int i=0;i<6;i++)
    {
        the_robot->EDP_data.ECPtoEDP_force_xyz_torque_xyz[i] = 0;
        the_robot->EDP_data.ECPtoEDP_position_velocity[i] = 0;
    }

    for (int i=0;i<6;i++)
    {
        the_robot->EDP_data.ECPtoEDP_inertia[i] = 0;
    }

    the_robot->EDP_data.ECPtoEDP_inertia[2] = FORCE_INERTIA / 4;
    the_robot->EDP_data.ECPtoEDP_force_xyz_torque_xyz[2] = -speed;

    for (int i=0;i<6;i++)
    {
        the_robot->EDP_data.ECPtoEDP_behaviour[i] = UNGUARDED_MOTION;
        //		the_robot->EDP_data.ECPtoEDP_reciprocal_damping[i] = 0;
    }

    the_robot->EDP_data.ECPtoEDP_behaviour[2] = CONTACT;
    the_robot->EDP_data.ECPtoEDP_reciprocal_damping[2] = FORCE_RECIPROCAL_DAMPING / 2;



    return true;
}
; // end: ecp_tff_gripper_approach_generator::first_step()



// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool ecp_tff_gripper_approach_generator::next_step ()
{
    // Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
    // Funkcja zwraca false gdy koniec generacji trajektorii
    // Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
    // UWAGA: dzialamy na jednoelementowej liscie robotow
    // cout << "next_step" << endl;


    // Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
    the_robot->EDP_data.instruction_type = SET_GET;

    // Obliczenie zadanej pozycji posredniej w tym kroku ruchu

    if (node_counter==1)
    {
        the_robot->EDP_data.next_gripper_coordinate = the_robot->EDP_data.current_gripper_coordinate;
    }
    else
    {
        if (node_counter > motion_time)
        {
            ecp_t.ecp_termination_notice();
            return false;
        }
    }

    return true;

}
; // end: bool ecp_tff_gripper_approach_generator::next_step ()
