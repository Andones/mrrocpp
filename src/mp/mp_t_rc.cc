// -------------------------------------------------------------------------
//                              mp_task_rk.cc
//
// MP Master Process - methods�for rubik cube solver task
//
// -------------------------------------------------------------------------


#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <list>
#include <map>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "mp/mp.h"
#include "mp/mp_g_force.h"
#include "mp/mp_g_vis.h"
#include "mp/mp_common_generators.h"
#include "mp/mp_t_rc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"
#include "ecp_mp/ecp_mp_s_vis.h"
#include "ecp_mp/ecp_mp_tr_rc_windows.h"

void mp_task_rubik_cube_solver::initiate(CUBE_COLOR up_is, CUBE_COLOR down_is, CUBE_COLOR front_is,
		CUBE_COLOR rear_is, CUBE_COLOR left_is, CUBE_COLOR right_is)
{
	cube_state = new CubeState(up_is, down_is, front_is, rear_is, left_is, right_is);

	cube_initial_state = NULL;
};


mp_task_rubik_cube_solver::mp_task_rubik_cube_solver(configurator &_config) : mp_task(_config)
{
}
;

mp_task_rubik_cube_solver::~mp_task_rubik_cube_solver()
{
	delete cube_state;
}

void mp_task_rubik_cube_solver::identify_colors() //DO WIZJI (przekladanie i ogladanie scian)
{

	//sekwencja poczatkowa w kolejnosci: UP, DOWN, FRONT, BACK, LEFT, RIGHT
	//cube_initial_state=BGROWY

	// manianka
	cube_state->set_state(BLUE, GREEN, RED, ORANGE, WHITE, YELLOW);

	CUBE_TURN_ANGLE changing_order[]={CL_0, CL_0, CL_180, CL_0, CL_180, CL_0};

	for(int k=0; k<6; k++)
	{

		face_turn_op(CL_0);

		usleep(1000*5000); //30 000 - OK //unrem		   //3000 - na lato na zime 5000
		sensor_m[SENSOR_CAMERA_ON_TRACK]->initiate_reading();
		usleep(1000*1000);
		sensor_m[SENSOR_CAMERA_ON_TRACK]->get_reading();

		for(int i=0; i<3; i++)
			for(int j=0; j<3; j++)
				cube_state->cube_tab[k][3*i+j]=(char)sensor_m[SENSOR_CAMERA_ON_TRACK]->image.sensor_union.cube_face.colors[3*i+j];


		printf("\nFACE FACE %d:\n",k);
		for(int i=0; i<9; i++)
		{
			switch (cube_state->cube_tab[k][i])
			{
			case 1:
				cube_state->cube_tab[k][i]='r';
				printf("R");
				break;
			case 2:
				cube_state->cube_tab[k][i]='o';
				printf("O");
				break;
			case 3:
				cube_state->cube_tab[k][i]='y';
				printf("Y");
				break;
			case 4:
				cube_state->cube_tab[k][i]='g';
				printf("G");
				break;
			case 5:
				cube_state->cube_tab[k][i]='b';
				printf("B");
				break;
			case 6:
				cube_state->cube_tab[k][i]='w';
				printf("W");
				break;
			default:
				cube_state->cube_tab[k][i]='o';
				printf("?");
				break;
			}
		}
		printf("\n");

		usleep(1000*1000); //unrem
		face_change_op(changing_order[k]);

	} //for

}


bool mp_task_rubik_cube_solver::communicate_with_windows_solver()
{
	char c_up;
	char c_right;
	char c_front;
	char c_down;
	char c_left;
	char c_back;
	//	char face_c, rot_c, curr_c;
	int s;
	int str_size;
	char cube_tab_send[54];
	char manipulation_sequence[200];

	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
			cube_tab_send[2*9+3*i+j]=cube_state->cube_tab[0][3*i+j]; //rot cl 0

			for(int i=0; i<3; i++)
				for(int j=0; j<3; j++)
					cube_tab_send[1*9+3*j+2-i]=cube_state->cube_tab[1][3*i+j]; //rot cl 90

			for(int i=0; i<3; i++)
				for(int j=0; j<3; j++)
					cube_tab_send[3*9+3*(2-j)+i]=cube_state->cube_tab[2][3*i+j]; //rot ccl 90

			for(int i=0; i<3; i++)
				for(int j=0; j<3; j++)
					cube_tab_send[5*9+3*i+j]=cube_state->cube_tab[3][3*i+j]; //rot cl 0

			for(int i=0; i<3; i++)
				for(int j=0; j<3; j++)
					cube_tab_send[4*9+3*j+2-i]=cube_state->cube_tab[4][3*i+j]; //rot cl 90

			for(int i=0; i<3; i++)
				for(int j=0; j<3; j++)
					cube_tab_send[0*9+3*j+2-i]=cube_state->cube_tab[5][3*i+j]; //rot cl 90

			printf("SEQ IN COLOR : %s\n",cube_tab_send);

			c_up=cube_tab_send[4];
			c_right=cube_tab_send[13];
			c_front=cube_tab_send[22];
			c_down=cube_tab_send[31];
			c_left=cube_tab_send[40];
			c_back=cube_tab_send[49];

			printf("%c %c %c %c %c %c\n", c_up, c_right, c_front, c_down, c_left, c_back);

			for(int i=0; i<54; i++)
			{
				if (cube_tab_send[i] == c_up)
					cube_tab_send[i]='u';
				else if (cube_tab_send[i] == c_down)
					cube_tab_send[i]='d';
				else if (cube_tab_send[i] == c_front)
					cube_tab_send[i]='f';
				else if (cube_tab_send[i] == c_back)
					cube_tab_send[i]='b';
				else if (cube_tab_send[i] == c_right)
					cube_tab_send[i]='r';
				else if (cube_tab_send[i] == c_left)
					cube_tab_send[i]='l';
			}

			/*
    for(int i=0; i<54; i++)
{
    	switch (cube_tab_send[i])
    	 {
    	  	case 'b': cube_tab_send[i]='u'; break;
    	  	case 'g': cube_tab_send[i]='d'; break;
    	  	case 'o': cube_tab_send[i]='f'; break;
    	  	case 'r': cube_tab_send[i]='b'; break;
    	  	case 'y': cube_tab_send[i]='r'; break;
    	  	case 'w': cube_tab_send[i]='l'; break;
    	 }
}		
			 */

			cube_tab_send[54]='\0';

			printf("SEQ FROM VIS : %s\n",cube_tab_send);

			//reszta
			// struktura pomiocnicza
			SingleManipulation single_manipulation;

			// czyszczenie listy
			manipulation_list.clear();

			for(int i=0; i<54; i++)
			{
				transmitter_m[TRANSMITTER_RC_WINDOWS]->to_va.rc_windows.rc_state[i]=cube_tab_send[i];
			}
			//transmitter_m[TRANSMITTER_RC_WINDOWS]->to_va.rc_windows.rc_state[i]=patternx[i];
			transmitter_m[TRANSMITTER_RC_WINDOWS]->to_va.rc_windows.rc_state[54]='\0';

			transmitter_m[TRANSMITTER_RC_WINDOWS]->t_write();
			transmitter_m[TRANSMITTER_RC_WINDOWS]->t_read(true);

			printf ("OPS: %s", transmitter_m[TRANSMITTER_RC_WINDOWS]->from_va.rc_windows.sequence);

			strcpy (manipulation_sequence,transmitter_m[TRANSMITTER_RC_WINDOWS]->from_va.rc_windows.sequence);

			if ((manipulation_sequence[0]=='C') && (manipulation_sequence[1]=='u') && (manipulation_sequence[2]=='b') && (manipulation_sequence[3]=='e'))
			{
				printf("Jam jest daltonista. ktory Ci nie ulozy kostki\n");
				return true;
			}

			//sekwencja poczatkowa w kolejnosci: UP, DOWN, FRONT, BACK, LEFT, RIGHT
			//cube_initial_state=BGROWY


			s=0;
			str_size=0;
			for (unsigned int char_i=0; char_i < strlen(transmitter_m[TRANSMITTER_RC_WINDOWS]->from_va.rc_windows.sequence)-1; char_i ++)
			{
				if (s==0)
				{
					switch (transmitter_m[TRANSMITTER_RC_WINDOWS]->from_va.rc_windows.sequence[char_i])
					{
					case 'U':
						manipulation_sequence[str_size] = 'B';
						break;
					case 'D':
						manipulation_sequence[str_size] = 'G';
						break;
					case 'F':
						manipulation_sequence[str_size] = 'O';
						break;
					case 'B':
						manipulation_sequence[str_size] = 'R';
						break;
					case 'L':
						manipulation_sequence[str_size] = 'W';
						break;
					case 'R':
						manipulation_sequence[str_size] = 'Y';
						break;
					}
					s=1;
					str_size++;
				}
				else if (s==1)
				{
					switch (transmitter_m[TRANSMITTER_RC_WINDOWS]->from_va.rc_windows.sequence[char_i])
					{
					case ' ':
						manipulation_sequence[str_size] = '1';
						s=0;
						break;
					case '2':
						manipulation_sequence[str_size] = '2';
						s=2;
						break;
					case '\'':
						manipulation_sequence[str_size] = '3';
						s=2;
						break;
					}
					str_size++;
				}
				else if (s==2)
				{
					s=0;
				}

			}

			if (s==1)
			{
				str_size--;
				manipulation_sequence[str_size] = '1';
				str_size++;
			}
			manipulation_sequence[str_size]='\0';

			printf ("\n%d %d\n",str_size,strlen(manipulation_sequence));
			printf ("SEQ from win %s\n",transmitter_m[TRANSMITTER_RC_WINDOWS]->from_va.rc_windows.sequence);
			printf ("\nSEQ2 %s\n",manipulation_sequence);

			//pocztaek ukladania
			// dodawanie manipulacji do listy
			for (unsigned int char_i=0; char_i < strlen(manipulation_sequence)-1; char_i += 2)
			{
				single_manipulation.set_state(read_cube_color(manipulation_sequence[char_i]),
						read_cube_turn_angle(manipulation_sequence[char_i+1]));
				manipulation_list.push_back(single_manipulation);
			}

			return false;
}


void mp_task_rubik_cube_solver::execute_manipulation_sequence()
{
	for(std::list<SingleManipulation>::iterator manipulation_list_iterator = manipulation_list.begin();
	manipulation_list_iterator != manipulation_list.end(); manipulation_list_iterator++)
	{
		manipulate(manipulation_list_iterator->face_to_turn, manipulation_list_iterator->turn_angle);
	}
}



void mp_task_rubik_cube_solver::manipulate(CUBE_COLOR face_to_turn, CUBE_TURN_ANGLE turn_angle )
{

	if (face_to_turn == cube_state->up)
	{
		// printf("cube_state->up\n");
		face_change_op(CL_90);
		face_turn_op(turn_angle);
	}
	else if (face_to_turn == cube_state->down)
	{
		// printf("cube_state->down\n");
		face_change_op(CCL_90);
		face_turn_op(turn_angle);

	}
	else if (face_to_turn == cube_state->front)
	{
		// printf("cube_state->front\n");
		face_change_op(CL_0);
		face_turn_op(CL_0);
		face_change_op(CL_90);
		face_turn_op(turn_angle);
	}
	else if (face_to_turn == cube_state->rear)
	{
		// printf("cube_state->rear\n");
		face_change_op(CL_0);
		face_turn_op(CL_0);
		face_change_op(CCL_90);
		face_turn_op(turn_angle);
	}
	else if (face_to_turn == cube_state->left)
	{
		// printf("cube_state->left\n");
		face_change_op(CL_0);
		face_turn_op(turn_angle);
	}
	else if (face_to_turn == cube_state->right)
	{
		// printf("cube_state->right\n");
		face_change_op(CL_180);
		face_turn_op(turn_angle);
	}
};



// obrot sciany
void mp_task_rubik_cube_solver::face_turn_op(CUBE_TURN_ANGLE turn_angle)
{

	// zblizenie chwytakow

	mp_teach_in_generator mp_ti1_gen(*this);
	mp_ti1_gen.robot_m = robot_m;


	switch (turn_angle)
	{
	case CL_90:
		mp_ti1_gen.load_file_with_path ("../trj/rc/fturn_ap_cl_90.trj", 2);
		break;
	case CL_0:
		mp_ti1_gen.load_file_with_path ("../trj/rc/fturn_ap_cl_0.trj", 2);
		break;
	case CCL_90:
		mp_ti1_gen.load_file_with_path ("../trj/rc/fturn_ap_ccl_90.trj", 2);
		break;
	case CL_180:
		mp_ti1_gen.load_file_with_path ("../trj/rc/fturn_ap_cl_180.trj", 2);
		break;
	default:
		break;
	}

	mp_ti1_gen.initiate_pose_list();

	mp_ti1_gen.Move();


	// zacisniecie postumenta na kostce


	for (std::map <SENSOR_ENUM, sensor*>::iterator sensor_m_iterator = sensor_m.begin();
	sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
	{
		sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
		sensor_m_iterator->second->configure_sensor();
	}

	mp_tff_rubik_grab_generator mp_tff_rg_gen(*this, 10);
	mp_tff_rg_gen.robot_m = robot_m;
	mp_tff_rg_gen.sensor_m[SENSOR_FORCE_ON_TRACK] = sensor_m[SENSOR_FORCE_ON_TRACK];
	mp_tff_rg_gen.sensor_m[SENSOR_FORCE_POSTUMENT] = sensor_m[SENSOR_FORCE_POSTUMENT];

	mp_tff_rg_gen.configure(0, 1, 0.057, 0.00005, 0);
	mp_tff_rg_gen.Move();


	mp_tff_rg_gen.configure(0, 1, 0.057, 0.00005, 50);
	mp_tff_rg_gen.Move();


	// obrot kostki

	mp_tff_rubik_face_rotate_generator mp_tff_rf_gen(*this, 10);
	mp_tff_rf_gen.robot_m = robot_m;
	mp_tff_rf_gen.sensor_m[SENSOR_FORCE_ON_TRACK] = sensor_m[SENSOR_FORCE_ON_TRACK];
	mp_tff_rf_gen.sensor_m[SENSOR_FORCE_POSTUMENT] = sensor_m[SENSOR_FORCE_POSTUMENT];

	switch (turn_angle)
	{
	case CL_90:
		mp_tff_rf_gen.configure (90.0, 0.0);
		mp_tff_rf_gen.Move();
		break;
	case CL_0:
		break;
	case CCL_90:
		mp_tff_rf_gen.configure (-90.0, 0.0);
		mp_tff_rf_gen.Move();
		break;
	case CL_180:
		mp_tff_rf_gen.configure (180.0, 0.0);
		mp_tff_rf_gen.Move();
		break;
	default:
		break;
	}

	// rozwarcie chwytaka tracka

	gripper_opening (0.02, 0.0, 1000);

	// odejscie tracka od postumenta

	mp_teach_in_generator mp_ti2_gen (*this);
	mp_ti2_gen.robot_m = robot_m;

	mp_ti2_gen.load_file_with_path ("../trj/rc/fturn_de.trj", 2);
	mp_ti2_gen.initiate_pose_list();


	mp_ti2_gen.Move();


};


// zmiana sciany (przelozenie kostki)
void mp_task_rubik_cube_solver::face_change_op(CUBE_TURN_ANGLE turn_angle)
{

	// zblizenie chwytakow

	mp_teach_in_generator mp_ti1_gen(*this);
	mp_ti1_gen.robot_m = robot_m;


	switch (turn_angle)
	{
	case CL_90:
		mp_ti1_gen.load_file_with_path ("../trj/rc/fchange_ap_cl_90.trj", 2);
		break;
	case CL_0:
		mp_ti1_gen.load_file_with_path ("../trj/rc/fchange_ap_cl_0.trj", 2);
		break;
	case CCL_90:
		mp_ti1_gen.load_file_with_path ("../trj/rc/fchange_ap_ccl_90.trj", 2);
		break;
	case CL_180:
		mp_ti1_gen.load_file_with_path ("../trj/rc/fchange_ap_cl_180.trj", 2);
		break;
	default:
		break;
	}

	mp_ti1_gen.initiate_pose_list();

	mp_ti1_gen.Move();


	// zacisniecie tracka na kostce

	for (std::map <SENSOR_ENUM, sensor*>::iterator sensor_m_iterator = sensor_m.begin();
	sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
	{
		sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
		sensor_m_iterator->second->configure_sensor();
	}

	mp_tff_rubik_grab_generator mp_tff_rg_gen(*this, 10);
	// mp_tff_rg_gen.wait_for_ECP_pulse = true;
	mp_tff_rg_gen.robot_m = robot_m;
	mp_tff_rg_gen.sensor_m[SENSOR_FORCE_ON_TRACK] = sensor_m[SENSOR_FORCE_ON_TRACK];
	mp_tff_rg_gen.sensor_m[SENSOR_FORCE_POSTUMENT] = sensor_m[SENSOR_FORCE_POSTUMENT];


	mp_tff_rg_gen.configure(1, 0, 0.072, 0.00005, 0, false, false);
	mp_tff_rg_gen.Move();

	mp_tff_rg_gen.configure(1, 0, 0.062, 0.00005, 0);
	mp_tff_rg_gen.Move();


	// docisniecie chwytaka tracka do kostki
	mp_tff_gripper_approach_generator mp_tff_ga_gen(*this, 10);
	mp_tff_ga_gen.robot_m = robot_m;
	mp_tff_ga_gen.sensor_m[SENSOR_FORCE_ON_TRACK] = sensor_m[SENSOR_FORCE_ON_TRACK];
	mp_tff_ga_gen.sensor_m[SENSOR_FORCE_POSTUMENT] = sensor_m[SENSOR_FORCE_POSTUMENT];
	//	mp_tff_ga_gen.transmitter_m[TRANSMITTER_RC_WINDOWS] = transmitter_m[TRANSMITTER_RC_WINDOWS];
	mp_tff_ga_gen.configure(5.0, 0.0, 50);
	mp_tff_ga_gen.Move();

	// zacisniecie tracka na kostce

	mp_tff_rg_gen.configure(1, 0, 0.057, 0.00005, 0);
	mp_tff_rg_gen.Move();


	// wstepne rozwarcie chwytaka postumenta

	gripper_opening(0.0, 0.002, 1000);

	// dalsze zacisniecie tracka na kostce

	mp_tff_rg_gen.configure(1,0, 0.057, 0.00005, 50);
	mp_tff_rg_gen.Move();

	// dalsze rozwarcie chwytaka postumenta

	gripper_opening(0.0, 0.02, 1000);

	// odejscie tracka od postumenta

	mp_teach_in_generator mp_ti2_gen(*this);
	mp_ti2_gen.robot_m = robot_m;

	switch (turn_angle)
	{
	case CL_90:
		mp_ti2_gen.load_file_with_path ("../trj/rc/fchange_de_cl_90.trj", 2);
		break;
	case CL_0:
		mp_ti2_gen.load_file_with_path ("../trj/rc/fchange_de_cl_0.trj", 2);
		break;
	case CCL_90:
		mp_ti2_gen.load_file_with_path ("../trj/rc/fchange_de_ccl_90.trj", 2);
		break;
	case CL_180:
		mp_ti2_gen.load_file_with_path ("../trj/rc/fchange_de_cl_180.trj", 2);
		break;
	default:
		break;
	}

	mp_ti2_gen.initiate_pose_list();

	mp_ti2_gen.Move();

	// zmiana stanu kostki

	CubeState tmp_cube_state;

	switch (turn_angle)
	{
	case CL_90:
		// printf("CL_90\n");
		tmp_cube_state.set_state(cube_state->left, cube_state->right, cube_state->up, cube_state->down,
				cube_state->front, cube_state->rear);
		break;
	case CL_0:
		// printf("CL_0\n");
		tmp_cube_state.set_state(cube_state->front, cube_state->rear, cube_state->left, cube_state->right,
				cube_state->up, cube_state->down);
		break;
	case CCL_90:
		// printf("CCL_90\n");
		tmp_cube_state.set_state(cube_state->right, cube_state->left, cube_state->down, cube_state->up,
				cube_state->front, cube_state->rear);
		break;
	case CL_180:
		// printf("CL_180\n");
		tmp_cube_state.set_state(cube_state->front, cube_state->rear, cube_state->right, cube_state->left,
				cube_state->down, cube_state->up);
		break;
	default:
		break;
	}

	*cube_state = tmp_cube_state;

	//	cube_state->print_cube_colors();

};


// dojscie
void mp_task_rubik_cube_solver::approach_op(int mode)
{


	mp_tff_nose_run_generator mp_tff_fr_gen(*this, 10);
	mp_tff_fr_gen.robot_m = robot_m;
	mp_tff_fr_gen.sensor_m[SENSOR_FORCE_ON_TRACK] = sensor_m[SENSOR_FORCE_ON_TRACK];
	mp_tff_fr_gen.sensor_m[SENSOR_FORCE_POSTUMENT] = sensor_m[SENSOR_FORCE_POSTUMENT];
	// mp_tff_fr_gen.sensor_m = sensor_m;
	mp_tff_fr_gen.configure(1, 0);
	sr_ecp_msg->message("Track podatny do czasu wcisniecia mp_trigger");
	mp_tff_fr_gen.Move();

	sr_ecp_msg->message("Odtwarzanie trajektorii");
	mp_teach_in_generator mp_ti1_gen(*this);
	mp_ti1_gen.robot_m = robot_m;

	mp_ti1_gen.load_file_with_path ("../trj/rc/ap_1.trj", 2);
	mp_ti1_gen.initiate_pose_list();

	mp_ti1_gen.Move();

	mp_teach_in_generator mp_ti2_gen(*this);
	mp_ti2_gen.robot_m = robot_m;

	mp_ti2_gen.load_file_with_path ("../trj/rc/ap_2.trj", 2);
	mp_ti2_gen.initiate_pose_list();

	mp_ti2_gen.Move();

	/*
    	// docisniecie chwytaka do kostki

    	mp_tff_gripper_approach_generator mp_tff_ga_gen(*this, 10); 
    	mp_tff_ga_gen.configure(10.0 , 0.0, 100);
    	if (Move ( mp_tff_ga_gen)) {
    		return true;
         }
	 */

	// opcjonalne serwo wizyjne
	if (mode)
	{
		mp_seven_eye_generator eyegen(*this, 4);
		eyegen.robot_m = robot_m; // mozna przydzielic tylko postumenta
		eyegen.sensor_m[SENSOR_CAMERA_SA] = sensor_m[SENSOR_CAMERA_SA];

		eyegen.Move();
	}

	// zacisniecie chwytaka na kostce


	mp_tff_rubik_grab_generator mp_tff_rg_gen(*this, 10);
	mp_tff_rg_gen.robot_m = robot_m;
	mp_tff_rg_gen.sensor_m[SENSOR_FORCE_ON_TRACK] = sensor_m[SENSOR_FORCE_ON_TRACK];
	mp_tff_rg_gen.sensor_m[SENSOR_FORCE_POSTUMENT] = sensor_m[SENSOR_FORCE_POSTUMENT];

	mp_tff_rg_gen.configure(1,0, 0.057, 0.00005, 0);
	mp_tff_rg_gen.Move();


	mp_tff_rg_gen.configure(1,0, 0.057, 0.00005, 50);
	mp_tff_rg_gen.Move();

	/*
        	mp_teach_in_generator mp_ti3_gen(*this);
    	mp_load_file_with_path (mp_ti3_gen, "../trj/rc/ap_3.trj", 2); 
    	mp_ti3_gen.initiate_pose_list();


    	if (Move ( mp_ti3_gen)){
              return true;
        }
	 */

};


// odejscie
void mp_task_rubik_cube_solver::departure_op()
{

	mp_teach_in_generator mp_ti1_gen(*this);
	mp_ti1_gen.robot_m = robot_m;

	mp_ti1_gen.load_file_with_path ("../trj/rc/de_1.trj", 2);
	mp_ti1_gen.initiate_pose_list();

	mp_ti1_gen.Move();

};


void mp_task_rubik_cube_solver::gripper_opening(double track_increment, double postument_increment, int motion_time)
{

	trajectory_description tdes;

	tdes.arm_type = XYZ_EULER_ZYZ;
	tdes.interpolation_node_no = 1;
	tdes.internode_step_no = motion_time;
	tdes.value_in_step_no = tdes.internode_step_no - 2;
	// Wspolrzedne kartezjanskie XYZ i katy Eulera ZYZ
	tdes.coordinate_delta[0] = 0.0; // przyrost wspolrzednej X
	tdes.coordinate_delta[1] = 0.0;// przyrost wspolrzednej Y
	tdes.coordinate_delta[2] = 0.0;   // przyrost wspolrzednej Z
	tdes.coordinate_delta[3] = 0.0;   // przyrost wspolrzednej FI
	tdes.coordinate_delta[4] = 0.0;   // przyrost wspolrzednej TETA
	tdes.coordinate_delta[5] = 0.0;   // przyrost wspolrzednej PSI
	//	tdes.coordinate_delta[6] = 0.0;   // przyrost wspolrzednej PSI
	tdes.coordinate_delta[6] = track_increment;   // przyrost wspolrzednej PSI

	trajectory_description tdes2;

	tdes2.arm_type = XYZ_EULER_ZYZ;
	tdes2.interpolation_node_no = 1;
	tdes2.internode_step_no = motion_time;
	tdes2.value_in_step_no = tdes.internode_step_no - 2;
	// Wspolrzedne kartezjanskie XYZ i katy Eulera ZYZ
	tdes2.coordinate_delta[0] = 0.0; // przyrost wspolrzednej X
	tdes2.coordinate_delta[1] = 0.0;// przyrost wspolrzednej Y
	tdes2.coordinate_delta[2] = 0.0;   // przyrost wspolrzednej Z
	tdes2.coordinate_delta[3] = 0.0;   // przyrost wspolrzednej FI
	tdes2.coordinate_delta[4] = 0.0;   // przyrost wspolrzednej TETA
	tdes2.coordinate_delta[5] = 0.0;   // przyrost wspolrzednej PSI
	//	tdes2.coordinate_delta[6] = 0.02;   // przyrost wspolrzednej PSI
	tdes2.coordinate_delta[6] = postument_increment;   // przyrost wspolrzednej PSI

	// Generator trajektorii prostoliniowej
	mp_tight_coop_generator tcg(*this, tdes, tdes2);
	tcg.robot_m = robot_m;

	tcg.Move();

}



mp_task* return_created_mp_task (configurator &_config)
{
	return new mp_task_rubik_cube_solver(_config);
}



// methods fo mp template to redefine in concete class
void mp_task_rubik_cube_solver::task_initialization(void)
{
	// Powolanie czujnikow
	sensor_m[SENSOR_FORCE_ON_TRACK] =
		new ecp_mp_schunk_sensor (SENSOR_FORCE_ON_TRACK, "[vsp_force_irp6ot]", *this);

	sensor_m[SENSOR_FORCE_POSTUMENT] =
		new ecp_mp_schunk_sensor (SENSOR_FORCE_POSTUMENT, "[vsp_force_irp6p]", *this);

	sensor_m[SENSOR_CAMERA_ON_TRACK] =
		new ecp_mp_vis_sensor (SENSOR_CAMERA_ON_TRACK, "[vsp_vis_eih]", *this);

	sensor_m[SENSOR_CAMERA_SA] =
		new ecp_mp_vis_sensor (SENSOR_CAMERA_SA, "[vsp_vis_sac]", *this);

	// Konfiguracja wszystkich czujnikow
	for (std::map <SENSOR_ENUM, sensor*>::iterator sensor_m_iterator = sensor_m.begin();
	sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
	{
		sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
		sensor_m_iterator->second->configure_sensor();
	}

	usleep(1000*100);

	// dodanie transmitter'a
	transmitter_m[TRANSMITTER_RC_WINDOWS] =
		new rc_windows_transmitter (TRANSMITTER_RC_WINDOWS, "[transmitter_rc_windows]", *this);

	sr_ecp_msg->message("MP rc loaded");
};


void mp_task_rubik_cube_solver::main_task_algorithm(void)
{


	// odczyt konfiguracji manipulacji
	if (cube_initial_state)
		delete[] cube_initial_state;
	cube_initial_state = config.return_string_value("cube_initial_state");

	//	enum CUBE_COLOR {UKNOWN, RED, YELLOW, GREEN, BLUE, ORANGE, WHITE};
	//	 cube_state::set_state(CUBE_COLOR up_is, CUBE_COLOR down_is, CUBE_COLOR front_is,
	//		CUBE_COLOR rear_is, CUBE_COLOR left_is, CUBE_COLOR right_is)

	initiate (read_cube_color(cube_initial_state[0]),
			read_cube_color(cube_initial_state[1]), read_cube_color(cube_initial_state[2]),  read_cube_color(cube_initial_state[3]),
			read_cube_color(cube_initial_state[4]), read_cube_color(cube_initial_state[5]));




		// Zlecenie wykonania kolejnego makrokroku
		// printf("po start all \n");
		for(;;)
		{
			sr_ecp_msg->message("Nowa seria");
			for (std::map <SENSOR_ENUM, sensor*>::iterator sensor_m_iterator = sensor_m.begin();
			sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
			{
				sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
				sensor_m_iterator->second->configure_sensor();
			}


			// przechwycenie kostki
			approach_op( config.return_int_value("vis_servoing"));

			// IDENTIFY COLORS
			identify_colors();

			if (communicate_with_windows_solver())
			{
				departure_op();
				break;
			}

			// wykonanie sekwencji manipulacji
			face_turn_op(CL_0);

			/*
            	char* manipulation_sequence = config->return_string_value("manipulation_sequence");
            	for (int char_i=0; char_i < strlen(manipulation_sequence)-1; char_i += 2)
        {
            single_manipulation.set_state(read_cube_color(manipulation_sequence[char_i]),
            read_cube_turn_angle(manipulation_sequence[char_i+1]));
            manipulation_list.push_back(single_manipulation);
        }
			 */

			execute_manipulation_sequence();

			// zakonczenie zadania

			printf ("przed face change\n");

			face_change_op(CL_0);

			departure_op();

		}


	
};
