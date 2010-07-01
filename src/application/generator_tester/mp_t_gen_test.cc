
#include "mp_t_gen_test.h"

//#include "subtask/ecp_mp_st_bias_edp_force.h"
//#include "subtask/ecp_mp_st_tff_nose_run.h"
//#include "generator/ecp/ecp_mp_g_tfg.h"

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config)
{
	return new gen_test(_config);
}

gen_test::gen_test(lib::configurator &_config) :
	task(_config)
{
}

void gen_test::main_task_algorithm(void)
{

	sr_ecp_msg->message("Gen Test (MP)");

	/*std::stringstream ss(std::stringstream::in | std::stringstream::out);

	lib::Xyz_Euler_Zyz_vector rel_eu(0, 0, 0, -1.57, 1.57, 1.57);
	lib::Homog_matrix tmp_hm(rel_eu);
	lib::Xyz_Angle_Axis_vector rel_aa;
	tmp_hm.get_xyz_angle_axis(rel_aa);

	ss << rel_aa;

	sr_ecp_msg->message(ss.str().c_str());

	// wybor manipulatora do sterowania na podstawie konfiguracji
    */
	lib::robot_name_t manipulator_name;
	lib::robot_name_t gripper_name;

	// ROBOT IRP6_ON_TRACK
	if (config.value <int> ("is_irp6ot_m_active", UI_SECTION)) {
		manipulator_name = lib::ROBOT_IRP6OT_M;
		if (config.value <int> ("is_irp6ot_tfg_active", UI_SECTION)) {
			gripper_name = lib::ROBOT_IRP6OT_TFG;
		} else {
			// TODO: throw
		}
	} else if (config.value <int> ("is_irp6p_m_active", UI_SECTION)) {
		manipulator_name = lib::ROBOT_IRP6P_M;
		if (config.value <int> ("is_irp6p_tfg_active", UI_SECTION)) {
			gripper_name = lib::ROBOT_IRP6P_TFG;
		} else {
			// TODO: throw
		}
	} else {
		// TODO: throw
	}

	// sekwencja generator na wybranym chwytaku

	//	char tmp_string[MP_2_ECP_STRING_SIZE];

	//	lib::tfg_command mp_ecp_tfg_command;

	//	mp_ecp_tfg_command.desired_position = 0.078;

	//	memcpy(tmp_string, &mp_ecp_tfg_command, sizeof(mp_ecp_tfg_command));

	//set_next_ecps_state(ecp_mp::common::generator::ECP_GEN_TFG, (int) 5, tmp_string, sizeof(mp_ecp_tfg_command), 1, gripper_name.c_str());
	/*
	 run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(
	 1, 1, gripper_name, gripper_name.c_str());
	 */
	// sekwencja generator na wybranym manipulatorze

	set_next_ecps_state(ecp_mp::task::ECP_ST_CONST_VEL_GEN_TEST, (int) 5, "", 0, 1, manipulator_name.c_str());

	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, manipulator_name.c_str(), manipulator_name.c_str());

	sr_ecp_msg->message("END");

}

} // namespace task
} // namespace mp
} // namespace mrrocpp
