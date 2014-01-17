/*!
 * @file
 * @brief File contains hswitals_rubik_rotate generator definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup generators
 */

#include <cstdio>
#include <fstream>
#include <iostream>
#include <ctime>
#include <unistd.h>
#include <cmath>
#include <iostream>

#include "base/lib/typedefs.h"

#include "base/lib/sr/sr_ecp.h"
#include "base/ecp/ecp_robot.h"
#include "ecp_g_hswitals_rubik_rotate.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

hswitals_rubik_rotate::hswitals_rubik_rotate(common::task::task& _ecp_task, int step) :
		common::generator::generator(_ecp_task), step_no(step)
{
    generator_name = ecp_mp::generator::ECP_GEN_HSWITALS_RUBIK_ROTATE;
}

void hswitals_rubik_rotate::configure(double l_turn_angle)
{
	turn_angle = l_turn_angle;
}

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool hswitals_rubik_rotate::first_step()
{
//    for (int i = 0; i < 6; i++) {
//        divisor[i] = 1;
//    }

    std::cout << std::endl << "hswitals_rubik_rotate" << node_counter << std::endl << std::endl;

	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i oreintacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// cout << "first_step" << endl;

	td.interpolation_node_no = 1;
	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 2;

	lib::Homog_matrix tool_frame(0.0, 0.0, 0.25);
	the_robot->ecp_command.robot_model.tool_frame_def.tool_frame = tool_frame;

	the_robot->ecp_command.instruction_type = lib::GET;
	the_robot->ecp_command.get_type = ARM_DEFINITION; // arm - ORYGINAL
	the_robot->ecp_command.set_type = ARM_DEFINITION | ROBOT_MODEL_DEFINITION;
	//	the_robot->ecp_command.set_type = ARM_DEFINITION;
	the_robot->ecp_command.robot_model.type = lib::TOOL_FRAME;
	the_robot->ecp_command.get_robot_model_type = lib::TOOL_FRAME;
	the_robot->ecp_command.set_arm_type = lib::PF_VELOCITY;
	//the_robot->ecp_command.get_arm_type = lib::FRAME;
	the_robot->ecp_command.motion_type = lib::RELATIVE;
	the_robot->ecp_command.interpolation_type = lib::TCIM;
	the_robot->ecp_command.motion_steps = td.internode_step_no;
	the_robot->ecp_command.value_in_step_no = td.value_in_step_no;

	for (int i = 0; i < 6; i++) {
		the_robot->ecp_command.arm.pf_def.force_xyz_torque_xyz[i] = 0;
		the_robot->ecp_command.arm.pf_def.arm_coordinates[i] = 0;
	}

	for (int i = 0; i < 3; i++) {
		the_robot->ecp_command.arm.pf_def.inertia[i] = lib::FORCE_INERTIA;
		the_robot->ecp_command.arm.pf_def.inertia[i + 3] = lib::TORQUE_INERTIA;
	}

    the_robot->ecp_command.arm.pf_def.inertia[0] = lib::FORCE_INERTIA * MULTI;
    the_robot->ecp_command.arm.pf_def.inertia[1] = lib::FORCE_INERTIA * MULTI;
    //the_robot->ecp_command.arm.pf_def.inertia[5] = lib::TORQUE_INERTIA * MULTI;
    //the_robot->ecp_command.arm.pf_def.reciprocal_damping[5] = (lib::TORQUE_RECIPROCAL_DAMPING * MULTI_RD) / 4 ;

    the_robot->ecp_command.arm.pf_def.reciprocal_damping[5] = lib::TORQUE_RECIPROCAL_DAMPING / 4;
	the_robot->ecp_command.arm.pf_def.behaviour[5] = lib::CONTACT;

	if (-0.1 < turn_angle && turn_angle < 0.1) {
		for (int i = 0; i < 6; i++)
			the_robot->ecp_command.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
	} else {
		for (int i = 0; i < 3; i++) {
            the_robot->ecp_command.arm.pf_def.reciprocal_damping[i] = lib::FORCE_RECIPROCAL_DAMPING;
            //the_robot->ecp_command.arm.pf_def.reciprocal_damping[i] = lib::FORCE_RECIPROCAL_DAMPING / DIVISOR;
			the_robot->ecp_command.arm.pf_def.behaviour[i] = lib::CONTACT;
		}
		for (int i = 3; i < 5; i++)
			the_robot->ecp_command.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;

        the_robot->ecp_command.arm.pf_def.reciprocal_damping[0] = lib::FORCE_RECIPROCAL_DAMPING * MULTI_RD;
        the_robot->ecp_command.arm.pf_def.reciprocal_damping[1] = lib::FORCE_RECIPROCAL_DAMPING * MULTI_RD;
        the_robot->ecp_command.arm.pf_def.reciprocal_damping[5] = lib::TORQUE_RECIPROCAL_DAMPING;
        //the_robot->ecp_command.arm.pf_def.reciprocal_damping[5] = lib::TORQUE_RECIPROCAL_DAMPING * MULTI_RD;
		the_robot->ecp_command.arm.pf_def.behaviour[5] = lib::CONTACT;
		the_robot->ecp_command.arm.pf_def.force_xyz_torque_xyz[5] = copysign(2.5, turn_angle);
	}

	return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool hswitals_rubik_rotate::next_step()
{
    //std::cout<<"next step"<<std::endl;

//    bool start_changing_divisor[6];

//    double current_irp6ot_force_x;

    double current_irp6ot_inertia[6];
    double current_irp6ot_reciprocal_damping[6];

    for (int i = 0; i < 6; i++) {
        current_irp6ot_inertia[i] = the_robot->ecp_command.arm.pf_def.inertia[i];
        current_irp6ot_reciprocal_damping[i] = the_robot->ecp_command.arm.pf_def.reciprocal_damping[i];
        //current_irp6ot_force[i] = the_robot->ecp_command.arm.pf_def.force_xyz_torque_xyz[i];
        //start_changing_divisor[i] = false;
    }

	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
    // UWAGA: dzialamy na 2jednoelementowej liscie robotow
	// cout << "next_step" << endl;

	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
	the_robot->ecp_command.instruction_type = lib::SET_GET;

	// Obliczenie zadanej pozycji posredniej w tym kroku ruchu

	if (node_counter == 1) {

		if (turn_angle < -0.1 || 0.1 < turn_angle) {
			lib::Homog_matrix frame(the_robot->reply_package.arm.pf_def.arm_frame);
			lib::Xyz_Euler_Zyz_vector xyz_eul_zyz;
			frame.get_xyz_euler_zyz(xyz_eul_zyz);
			double angle_to_move = (turn_angle / 180.0) * M_PI;
            if (xyz_eul_zyz[5] + angle_to_move < -M_PI) {
				stored_gamma = 2 * M_PI + xyz_eul_zyz[5] + angle_to_move;
				range_change = true;
			} else if (xyz_eul_zyz[5] + angle_to_move > M_PI) {
				stored_gamma = -2 * M_PI + xyz_eul_zyz[5] + angle_to_move;
				range_change = true;
			} else {
				stored_gamma = xyz_eul_zyz[5] + angle_to_move;
				range_change = false;
			}
		}
	} else {
		if (turn_angle < -0.1 || 0.1 < turn_angle) {
            lib::Homog_matrix current_frame(the_robot->reply_package.arm.pf_def.arm_frame);
			lib::Xyz_Euler_Zyz_vector xyz_eul_zyz;
			current_frame.get_xyz_euler_zyz(xyz_eul_zyz);
			double current_gamma = xyz_eul_zyz[5];
			if (!range_change) {
				if ((turn_angle < 0.0 && stored_gamma > current_gamma)
						|| (turn_angle > 0.0 && stored_gamma < current_gamma)) {
					return false;
				}
			} else {
				if ((turn_angle < 0.0 && stored_gamma < current_gamma)
						|| (turn_angle > 0.0 && stored_gamma > current_gamma)) {
					range_change = false;
				}
			}
		}

	}

    std::cout << "counter: " << node_counter << std::endl;

//    if (node_counter % 1000 == 0) {
//        divisor[i] *= 2;
//        std::stringstream ss(std::stringstream::in | std::stringstream::out);
//        ss << "divisor " << i << ": " << divisor[i];
//        sr_ecp_msg.message(ss.str().c_str());
//    }
if (node_counter % 100 == 0) {
 //   the_robot->ecp_command.arm.pf_def.reciprocal_damping[5] = current_irp6ot_reciprocal_damping[5] / 2;
//    the_robot->ecp_command.arm.pf_def.reciprocal_damping[1] = 1.01 * current_irp6ot_reciprocal_damping[1];
//    the_robot->ecp_command.arm.pf_def.reciprocal_damping[5] = 1.01 * current_irp6ot_reciprocal_damping[5];
 //   the_robot->ecp_command.arm.pf_def.inertia[5] = current_irp6ot_inertia[5] / 2;
//    the_robot->ecp_command.arm.pf_def.inertia[1] = 1.01 * current_irp6ot_inertia[1];
//    the_robot->ecp_command.arm.pf_def.inertia[5] = 1.01 * current_irp6ot_inertia[5];

    std::cout << " inertia x: " << current_irp6ot_inertia[0] << ", damping x: " << current_irp6ot_reciprocal_damping[0]
              << " inertia y: " << current_irp6ot_inertia[1] << ", damping y: " << current_irp6ot_reciprocal_damping[1]
              << " inertia az: " << current_irp6ot_inertia[5] << ", damping az: " << current_irp6ot_reciprocal_damping[5] << std::endl;
}
    return true;
}

void hswitals_rubik_rotate::conditional_execution()
{

    switch ((ecp_mp::generator::HSWITALS_TURN_ANGLES) ecp_t.mp_command.ecp_next_state.variant)
	{
        case ecp_mp::generator::HSWITALS_CCL_90:
			configure(-90.0);
			break;
        case ecp_mp::generator::HSWITALS_CL_0:
			configure(0.0);
			break;
        case ecp_mp::generator::HSWITALS_CL_90:
			configure(90.0);
			break;
        case ecp_mp::generator::HSWITALS_CL_180:
			configure(180.0);
			break;
		default:
			break;
	}

	Move();
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
