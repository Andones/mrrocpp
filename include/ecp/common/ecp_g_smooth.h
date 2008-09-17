// -------------------------------------------------------------------------
//                            ecp_g_smooth.h dla QNX6
// Deklaracje generatora plynnego ruchu dla ECP
// 
// -------------------------------------------------------------------------

#if !defined(_ECP_GEN_SMOOTH_H)
# define _ECP_GEN_SMOOTH_H

#include "common/impconst.h"
#include "common/com_buf.h"

#include <list>
#include "ecp/common/ecp_smooth_taught_in_pose.h"

#include "ecp/common/ecp_g_jarosz.h"
#include "mp/Trajectory.h"

class ecp_smooth_generator : public ecp_delta_generator
{

protected:
	std::list<ecp_smooth_taught_in_pose> pose_list;
 	std::list<ecp_smooth_taught_in_pose>::iterator pose_list_iterator;
 	
	double a_max_motor[MAX_SERVOS_NR], a_max_joint[MAX_SERVOS_NR], a_max_zyz[MAX_SERVOS_NR], a_max_aa[MAX_SERVOS_NR];
	double v_max_motor[MAX_SERVOS_NR], v_max_joint[MAX_SERVOS_NR], v_max_zyz[MAX_SERVOS_NR], v_max_aa[MAX_SERVOS_NR];
	double v_grip, v_grip_min;

	int first_interval;
	trajectory_description td;
	
	int k[MAX_SERVOS_NR]; //kierunek ruchu (+/-)
	
	double t_max; //nadluzszy czas ruchu
	double v_p[MAX_SERVOS_NR], v_k[MAX_SERVOS_NR]; //predkosc poczatkowa i koncowa
	double v[MAX_SERVOS_NR], a [MAX_SERVOS_NR]; //wczytywane z pliku
	double v_r[MAX_SERVOS_NR], a_r[MAX_SERVOS_NR]; //predkosc i przyspieszenie w danym ruchu

	double final_position[MAX_SERVOS_NR]; //pozycja koncowa
	double start_position[MAX_SERVOS_NR]; //pozycja poczatkowa
	double next_position[MAX_SERVOS_NR]; //pozycja nastepego makrokroku
	bool is_synchronised;
	bool debug; //czy maja byc wyswietlane debugi
	
	double przysp[MAX_SERVOS_NR];  //moment, w ktorym konczy sie przyspieszanie
	double jedn[MAX_SERVOS_NR];  //moment, w ktorym konczy sie ruch jednostajny
	
	double s_przysp[MAX_SERVOS_NR];  //droga po etapie przyspieszania
	double s_jedn[MAX_SERVOS_NR];  //droga po etapie ruchu jednostajnego
	
	int type; //1 - polozenie bezwzgledne , 2 - polozenie przyrostowe
	bool first_coordinate;

public:
	ecp_smooth_generator(ecp_task& _ecp_task, bool _is_synchronised);
	ecp_smooth_generator(ecp_task& _ecp_task, bool _is_synchronised, bool _debug);
	
	void calculate(void);
	void generate_next_coords(void);
	
	void flush_pose_list ( void );
	void initiate_pose_list(void);
	void next_pose_list_ptr (void);
	void get_pose (void);
	void set_pose (POSE_SPECIFICATION ps, double v_p[MAX_SERVOS_NR], double v_k[MAX_SERVOS_NR], double v[MAX_SERVOS_NR], double a[MAX_SERVOS_NR], double coordinates[MAX_SERVOS_NR]);
	bool is_pose_list_element ( void );
	bool is_last_list_element ( void );
	void create_pose_list_head (POSE_SPECIFICATION ps, double v_p[MAX_SERVOS_NR], double v_k[MAX_SERVOS_NR], double v[MAX_SERVOS_NR], double a[MAX_SERVOS_NR], double coordinates[MAX_SERVOS_NR]);
	void insert_pose_list_element (POSE_SPECIFICATION ps, double v_p[MAX_SERVOS_NR], double v_k[MAX_SERVOS_NR], double v[MAX_SERVOS_NR], double a[MAX_SERVOS_NR], double coordinates[MAX_SERVOS_NR]);
	int pose_list_length(void);

	void set_relative(void); //zmiana na tryb przyrostowy
	void set_absolute(void); //zmiana na tryb bezwzgledny
	void reset(void);
	
	bool load_a_v_max(char* file_name);
	bool load_a_v_min (char* file_name);
	bool load_file_with_path (char* file_name);
	
	bool load_trajectory_from_xml(char* fileName, char* nodeName);
	bool load_trajectory_from_xml(Trajectory &trajectory);

	void load_coordinates(POSE_SPECIFICATION,double,double,double,double,double,double,double,double);
	void load_coordinates(POSE_SPECIFICATION,double[],double[],double[],double[],double[]);


	virtual bool first_step();
	virtual bool next_step();
};

class ecp_tool_change_generator : public ecp_smooth_generator
{
protected:
	double tool_parameters[3];
	
public:
	ecp_tool_change_generator(ecp_task& _ecp_task, bool _is_synchronised);
	ecp_tool_change_generator(ecp_task& _ecp_task, bool _is_synchronised, bool _debug);
	void set_tool_parameters(double x, double y, double z);
	
	virtual bool first_step();
	virtual bool next_step();
};

#endif
