/*
 * bclike_mp.cc
 *
 *  Created on: 06-07-2010
 *      Author: kszkudla
 */

#include "bclike_mp.h"
#include "bcl_t_switcher.h"
#include "ecp_mp_st_smooth_move.h"

namespace mrrocpp {

namespace mp {

namespace task {

#ifdef IRP6_OT
	const lib::robot_name_t actual_robot = lib::ROBOT_IRP6OT_M;
#endif

#ifdef IRP6_P
	const lib::robot_name_t actual_robot = lib::ROBOT_IRP6P_M;
#endif


bclike_mp::bclike_mp(lib::configurator &_config) :
	task(_config) {

	second_task = config.value<std::string>("fradia_task", "[vsp_second_task]");
	std::cout << "DRUGI TASK: " << second_task << std::endl;

	//TODO: Wczytywanie z konfiga informacji o drugim zadaniu

}

bclike_mp::~bclike_mp() {
}

void bclike_mp::main_task_algorithm(void){

	sr_ecp_msg->message("MP start");

	char* tab;
	std::vector<double> vec;

//	Set robot to start position (center)
	vec.clear();
	vec.assign(ecp::common::task::left, ecp::common::task::left + VEC_SIZE);
	tab = msg.trajectoryToString(vec);

	set_next_ecps_state (ecp_mp::task::BCL_MOTION_DIR_STR, (int)mrrocpp::ecp::common::task::START, tab, 300, 1, actual_robot.c_str());
	sr_ecp_msg->message("MOVE left");
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, actual_robot.c_str(), actual_robot.c_str());


#ifdef TEST_MODE
	int i = 0;

	while(1){
		switch(i){
			case 0:
				vec.clear();
				vec.assign(ecp::common::task::left, ecp::common::task::left + VEC_SIZE);
				tab = msg.trajectoryToString(vec);
				sr_ecp_msg->message("RIGHT send");
				break;
			case 1:
				vec.clear();
				vec.assign(ecp::common::task::right, ecp::common::task::right + VEC_SIZE);
				tab = msg.trajectoryToString(vec);
				sr_ecp_msg->message("LEFT send");
				break;
			case 2:
				vec.clear();
				vec.assign(ecp::common::task::start, ecp::common::task::start + VEC_SIZE);
				tab = msg.trajectoryToString(vec);
				sr_ecp_msg->message("START send");
				break;
		}

		i++;
		i = i % 3;

		set_next_ecps_state (ecp_mp::task::BCL_MOTION_DIR_STR, (int)mrrocpp::ecp::common::task::START, tab, 300, 1, actual_robot.c_str());
		run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, actual_robot.c_str(), actual_robot.c_str());

		sr_ecp_msg->message("MP end loop");

	}

	sr_ecp_msg->message("MP end");

	return;
#endif

#ifdef SINGLE_MOVE

	vec.clear();
	vec.assign(ecp::common::task::right, ecp::common::task::right + VEC_SIZE);
	tab = msg.trajectoryToString(vec);

	set_next_ecps_state (ecp_mp::task::ECP_ST_SMOOTH_MOVE, (int)mrrocpp::ecp::common::task::START, tab, 300, 1, actual_robot.c_str());
	sr_ecp_msg->message("MOVE right");
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, actual_robot.c_str(), actual_robot.c_str());


	while(strcmp(robot_m[actual_robot]->ecp_reply_package.ecp_2_mp_string, "KONIEC")){

		msg.stringToECPOrder(robot_m[actual_robot]->ecp_reply_package.ecp_2_mp_string, regions);

		set_next_ecps_state (ecp_mp::task::ECP_ST_SMOOTH_MOVE, (int)mrrocpp::ecp::common::task::START, tab, 300, 1, actual_robot.c_str());
		sr_ecp_msg->message("MOVE right");
		run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, actual_robot.c_str(), actual_robot.c_str());
	}


	std::vector<ecp::common::task::mrrocpp_regions>::iterator it;

	for(it = regions.begin(); it != regions.end(); ++it){
		//TODO: przelaczyc zadanie FrDIA + wywolac subtaks Marcina
	}

	#else

	bool run = true;

	vec.clear();
	vec.assign(ecp::common::task::right, ecp::common::task::right + VEC_SIZE);
	tab = msg.trajectoryToString(vec);

	while(run){

		set_next_ecps_state (ecp_mp::task::ECP_ST_SMOOTH_MOVE, (int)mrrocpp::ecp::common::task::START, tab, 300, 1, actual_robot.c_str());
		run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, actual_robot.c_str(), actual_robot.c_str());

		if(!strcmp(robot_m[actual_robot]->ecp_reply_package.ecp_2_mp_string, "KONIEC")){
			run = false;
		}else{

			pos = msg.stringToFradiaOrder(robot_m[actual_robot]->ecp_reply_package.ecp_2_mp_string, reg);

			for(int i = 0; i < reg.num_found; ++i){
				//TODO: wywołać Marcina subtask + przelaczyc zadanie FrDIA
			}

			//TODO: wrocic z zadaniem do mojego
		}

	}

#endif

	sr_ecp_msg->message("KONIEC");

}


task* return_created_mp_task (lib::configurator &_config)
{
return new bclike_mp(_config);
}


}

}

}
