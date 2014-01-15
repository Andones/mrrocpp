#include <cstdio>
#include <unistd.h>
#include <cstring>
#include <list>
#include <map>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"

#include "base/ecp_mp/ecp_mp_sensor.h"
#include "generator/ecp/smooth_file_from_mp/ecp_mp_g_smooth_file_from_mp.h"

#include "base/mp/mp_task.h"
#include "base/mp/generator/mp_g_wait_for_task_termination.h"
#include "mp_rubic_cube_hswitals.h"

#include "robot/irp6ot_m/const_irp6ot_m.h"

#include "generator/ecp/bias_edp_force/ecp_mp_g_bias_edp_force.h"
#include "generator/ecp/tff_nose_run/ecp_mp_g_tff_nose_run.h"
#include "generator/ecp/newsmooth/ecp_mp_g_newsmooth.h"
#include "generator/ecp/constant_velocity/ecp_mp_g_constant_velocity.h"
#include "generator/ecp/weight_measure/ecp_mp_g_weight_measure.h"

#include "generator/ecp/force_tool_change/ecp_mp_g_force_tool_change.h"
#include "generator/ecp/tff_gripper_approach/ecp_mp_g_tff_gripper_approach.h"
#include "generator/ecp/tff_rubik_face_rotate/ecp_mp_g_tff_rubik_face_rotate.h"

#include "generator/ecp/hswitals_generatore/ecp_mp_g_hswitals_generatore.h"

#include "robot/irp6ot_m/mp_r_irp6ot_m.h"
#include "robot/irp6ot_tfg/mp_r_irp6ot_tfg.h"


#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

namespace mrrocpp {
namespace mp {
namespace task {

rubik_cube::rubik_cube(lib::configurator &_config) : task(_config)
{
}

rubik_cube::~rubik_cube()
{
}

void rubik_cube::configure_edp_force_sensor()
{
    set_next_ecp_state(ecp_mp::generator::ECP_GEN_BIAS_EDP_FORCE, 0, "", lib::irp6ot_m::ROBOT_NAME);
    wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
}

void rubik_cube::main_task_algorithm(void)
{
    sr_ecp_msg->message("Zaczynamy");

    configure_edp_force_sensor();


//        set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_NOSE_RUN, (int) ecp_mp::generator::tff_nose_run::behaviour_specification, ecp_mp::generator::tff_nose_run::behaviour_specification_data_type(), lib::irp6ot_m::ROBOT_NAME);
//        sr_ecp_msg->message("--------Podatnosc w osi x DONE--------");

//    set_next_ecp_state(ecp_mp::generator::ECP_GEN_HSWITALS_GENERATORE, (int) ecp_mp::generator::hswitals_generatore::specification, ecp_mp::generator::hswitals_generatore::specification_data_type(), lib::irp6ot_m::ROBOT_NAME);
//    sr_ecp_msg->message("--------Podatnosc w osi x DONE--------");

    set_next_ecp_state(ecp_mp::generator::ECP_GEN_CONSTANT_VELOCITY, (int) lib::ABSOLUTE, 0.075, lib::irp6ot_tfg::ROBOT_NAME);
    wait_for_task_termination(false, lib::irp6ot_tfg::ROBOT_NAME);
    sr_ecp_msg->message("--------Rozluznienie chwytaka DONE--------");

    //wait_ms(2000);

    set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/hswitals/trj/irp6ot_test.trj", lib::irp6ot_m::ROBOT_NAME);
    wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
    sr_ecp_msg->message("--------Podjazd z pozycji wyjscowej DONE--------");

    //wait_ms(2000);

    set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, (int) lib::RELATIVE, "../../src/application/hswitals/trj/irp6ot_cube_approach.trj", lib::irp6ot_m::ROBOT_NAME);
    wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
    sr_ecp_msg->message("--------Podjazd do kostki DONE--------");

    wait_ms(5000);

    configure_edp_force_sensor();

   // send_end_motion_to_ecps(lib::irp6ot_m::ROBOT_NAME);

//    set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_NOSE_RUN, (int) ecp_mp::generator::tff_nose_run::behaviour_specification, ecp_mp::generator::tff_nose_run::behaviour_specification_data_type(true, true, false, false, false, false), lib::irp6ot_m::ROBOT_NAME);
//    sr_ecp_msg->message("--------Podatnosc w osi x DONE--------");

    set_next_ecp_state(ecp_mp::generator::ECP_GEN_HSWITALS_GENERATORE, (int) ecp_mp::generator::hswitals_generatore::specification, ecp_mp::generator::hswitals_generatore::specification_data_type(true, true, false, false, false, false, lib::FORCE_INERTIA, lib::FORCE_INERTIA, lib::FORCE_INERTIA, lib::TORQUE_INERTIA, lib::TORQUE_INERTIA, lib::TORQUE_INERTIA, lib::FORCE_RECIPROCAL_DAMPING, lib::FORCE_RECIPROCAL_DAMPING, lib::FORCE_RECIPROCAL_DAMPING, lib::TORQUE_RECIPROCAL_DAMPING, lib::TORQUE_RECIPROCAL_DAMPING, lib::TORQUE_RECIPROCAL_DAMPING), lib::irp6ot_m::ROBOT_NAME);
    sr_ecp_msg->message("--------Podatnosc w osiach x i y DONE--------");

    set_next_ecp_state(ecp_mp::generator::ECP_GEN_CONSTANT_VELOCITY, (int) lib::ABSOLUTE, 0.0565, lib::irp6ot_tfg::ROBOT_NAME);
    wait_for_task_termination(false, lib::irp6ot_tfg::ROBOT_NAME);
    send_end_motion_to_ecps(lib::irp6ot_m::ROBOT_NAME);
    sr_ecp_msg->message("--------Zaciskanie do 0.068 DONE--------");

    //wait_ms(2000);

//    set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_NOSE_RUN, (int) ecp_mp::generator::tff_nose_run::behaviour_specification, ecp_mp::generator::tff_nose_run::behaviour_specification_data_type(true, true, false, false, false, false), lib::irp6ot_m::ROBOT_NAME);
//    sr_ecp_msg->message("--------Podatnosci w osiach x i y DONE--------");

////    set_next_ecp_state(ecp_mp::generator::ECP_GEN_HSWITALS_GENERATORE, (int) ecp_mp::generator::hswitals_generatore::specification, ecp_mp::generator::hswitals_generatore::specification_data_type(true, true, false, false, false, false, lib::FORCE_INERTIA, lib::FORCE_INERTIA, lib::FORCE_INERTIA, lib::TORQUE_INERTIA, lib::TORQUE_INERTIA, lib::TORQUE_INERTIA, lib::FORCE_RECIPROCAL_DAMPING, lib::FORCE_RECIPROCAL_DAMPING, lib::FORCE_RECIPROCAL_DAMPING, lib::TORQUE_RECIPROCAL_DAMPING, lib::TORQUE_RECIPROCAL_DAMPING, lib::TORQUE_RECIPROCAL_DAMPING), lib::irp6ot_m::ROBOT_NAME);
////    sr_ecp_msg->message("--------Podatnosc w osiach x i y DONE--------");

//    set_next_ecp_state(ecp_mp::generator::ECP_GEN_CONSTANT_VELOCITY, (int) lib::ABSOLUTE, 0.0565, lib::irp6ot_tfg::ROBOT_NAME);
//    wait_for_task_termination(false, lib::irp6ot_tfg::ROBOT_NAME);
//    send_end_motion_to_ecps(lib::irp6ot_m::ROBOT_NAME);
//    sr_ecp_msg->message("--------Zaciskanie do 0.0565 DONE--------");

//    wait_ms(500);

//    set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_RUBIK_FACE_ROTATE, (int) ecp_mp::generator::RCSC_CCL_90, "", lib::irp6ot_m::ROBOT_NAME);
//    wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
//    sr_ecp_msg->message("--------Obrot DONE--------");

////    wait_ms(10000);

//    set_next_ecp_state(ecp_mp::generator::ECP_GEN_CONSTANT_VELOCITY, (int) lib::RELATIVE, 0.022, lib::irp6ot_tfg::ROBOT_NAME);
//    wait_for_task_termination(false, lib::irp6ot_tfg::ROBOT_NAME);
//    sr_ecp_msg->message("--------Rozluznienie chwytu--------");

//    //wait_ms(2000);

//    set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, (int) lib::RELATIVE, "../../src/application/hswitals/trj/irp6ot_cube_departure.trj", lib::irp6ot_m::ROBOT_NAME);
//    wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
//    sr_ecp_msg->message("--------Odjazd od kostki DONE--------");

}

// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void rubik_cube::create_robots()
{
	ACTIVATE_MP_ROBOT(irp6ot_tfg);
    ACTIVATE_MP_ROBOT(irp6ot_m);
//    ACTIVATE_MP_ROBOT(irp6p_tfg);
//    ACTIVATE_MP_ROBOT(irp6p_m);
}

task* return_created_mp_task(lib::configurator &_config)
{
    return new rubik_cube(_config);
}

} // namespace task
} // namespace mp
} // namespace mrrocpp

