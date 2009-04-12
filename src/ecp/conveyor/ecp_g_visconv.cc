#include <stdio.h>
#include <time.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp/conveyor/ecp_local.h"
//#include "ecp/conveyor/ecp_g_legobrick.h"
#include "ecp/conveyor/ecp_g_visconv.h"
#include "iostream.h"
#include "math.h"

namespace mrrocpp {
namespace ecp {
namespace conveyor {

conveyor_incremental_move::conveyor_incremental_move(common::ecp_task& _ecp_task, double inc_move):
		ecp_generator (_ecp_task), move_length(inc_move) {}

bool conveyor_incremental_move::first_step ( )
{
	td.interpolation_node_no = 1;
	td.internode_step_no = 50;//((int)move_length)*20;
	td.value_in_step_no = td.internode_step_no - 2;


			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV; // arm - ORYGINAL
			the_robot->EDP_data.set_type = ARM_DV;

			the_robot->EDP_data.set_arm_type = JOINT;
			the_robot->EDP_data.get_arm_type = JOINT;
			the_robot->EDP_data.motion_type = ABSOLUTE;
			the_robot->EDP_data.next_interpolation_type = MIM;
			the_robot->EDP_data.motion_steps = td.internode_step_no;
			the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

			first=1;
			stepno=ecp_t.config.return_int_value("steps");//100;

	return true;
}

bool conveyor_incremental_move::next_step ( )
{
	//struct timespec start[9];
	if (check_and_null_trigger()) { // Koniec odcinka
		return false;
	} 
	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji

	// nie aktualizujemy pozycjio na podstawie odczytu z EDP
	the_robot->EDP_data.instruction_type = SET;
	the_robot->EDP_data.get_type = NOTHING_DV;
	the_robot->EDP_data.get_arm_type = INVALID_END_EFFECTOR;
	
	if(first)
	{
		current_pose=the_robot->EDP_data.current_joint_arm_coordinates[0];
		begin_pose=current_pose;
		step=1;
		first=0;
	}
	
	
	//std::cout<<"ABS " << current_pose << std::endl; 
	
	// Obliczenie zadanej pozycji posredniej w tym kroku ruchu
	// (okreslenie kolejnego wezla interpolacji)
	/*if (((node_counter)%100)==0) {
		if( clock_gettime( CLOCK_REALTIME , &start[0]) == -1 ) {
			printf("blad pomiaru czasu");
		}
		printf("ECP conv pomiarow: %d,  czas: %ld, pozycja %f\n", node_counter,
		       start[0].tv_sec%100, the_robot->EDP_data.current_motor_arm_coordinates[0]);
	}*/

	
	//td.coordinate_delta[0] = -0.01; //0.01//move_length/td.internode_step_no;//((double)(((node_counter%2)*2)-1))/20;   // przyrost wspolrzednej X


	//next_pose = current_pose -0.005;
	next_pose=begin_pose-0.1*(1-cos(3.14*2*step/((double)(stepno))));
	//next_pose=0.005*cos(3.14*0.01*step);
	//std::cout << "BGN: " << begin_pose << "ABS: " << next_pose << std::endl;
	
	gettimeofday(&acctime,NULL);
	std::cout << acctime.tv_sec << " " << acctime.tv_usec << " " << next_pose << " ";
	std::cout << std::endl;
	
	the_robot->EDP_data.next_joint_arm_coordinates[0] = next_pose;//the_robot->EDP_data.current_joint_arm_coordinates[0]; //next_pose;
	//current_pose=next_pose;	
	
	step++;
	step%=stepno;
	
	//   the_robot->EDP_data.current_joint_arm_coordinates[0]	+ td.coordinate_delta[0];
	//}
	
	return true;
}
} // namespace conveyor
} // namespace ecp
} // namespace mrrocpp


