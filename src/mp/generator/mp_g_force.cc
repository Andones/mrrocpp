// -------------------------------------------------------------------------
//
// MP Master Process - methods�for force generators
//
// -------------------------------------------------------------------------
// Funkcje do konstruowania procesow MP

#include <stdio.h>
#include <iostream>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "mp/mp.h"
#include "mp/generator/mp_g_force.h"
#include "lib/mathtr.h"
#include "mp/generator/mp_g_common.h"

namespace mrrocpp {
namespace mp {
namespace generator {

// konstruktor
tff_single_robot_nose_run::tff_single_robot_nose_run(task::task& _mp_task, int step) :
	generator(_mp_task)
{
	step_no = step;
}
;

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool tff_single_robot_nose_run::first_step()
{
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i oreintacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	//  cout << "first_step" << endl;
	// 	irp6 = robot_list->E_ptr;
	irp6 = robot_m.begin()->second;
	irp6->communicate = true;

	vsp_force = sensor_m.begin()->second;

	idle_step_counter = 2;
	vsp_force->base_period=1;
	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 2;
	irp6->mp_command.command = lib::NEXT_POSE;
	irp6->mp_command.instruction.instruction_type = lib::GET;
	irp6->mp_command.instruction.get_type = ARM_DV;
	irp6->mp_command.instruction.set_type = ARM_DV | RMODEL_DV;
	irp6->mp_command.instruction.set_rmodel_type = lib::TOOL_FRAME;
	irp6->mp_command.instruction.get_rmodel_type = lib::TOOL_FRAME;
	irp6->mp_command.instruction.set_arm_type = lib::PF_VELOCITY;
	irp6->mp_command.instruction.get_arm_type = lib::FRAME;
	irp6->mp_command.instruction.motion_type = lib::RELATIVE;
	irp6->mp_command.instruction.interpolation_type = lib::TCIM;
	irp6->mp_command.instruction.motion_steps = td.internode_step_no;
	irp6->mp_command.instruction.value_in_step_no = td.value_in_step_no;

	lib::Homog_matrix tool_frame(0.0, 0.0, 0.25);
	tool_frame.get_frame_tab(irp6->mp_command.instruction.rmodel.tool_frame_def.tool_frame);

	for (int i=0; i<6; i++) {
		irp6->mp_command.instruction.arm.pf_def.arm_coordinates[i] = 0;
		irp6->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i] = 0;
		//	irp6->ecp_td.MPselection_vector[i] = FORCE_SV_AX;
	}

	for (int i=0; i<3; i++) {
		irp6->mp_command.instruction.arm.pf_def.inertia[i] = FORCE_INERTIA;
		irp6->mp_command.instruction.arm.pf_def.inertia[i+3] = TORQUE_INERTIA;
		irp6->mp_command.instruction.arm.pf_def.reciprocal_damping[i] = FORCE_RECIPROCAL_DAMPING;
		irp6->mp_command.instruction.arm.pf_def.reciprocal_damping[i+3] = TORQUE_RECIPROCAL_DAMPING;
		irp6->mp_command.instruction.arm.pf_def.behaviour[i] = lib::CONTACT;
		irp6->mp_command.instruction.arm.pf_def.behaviour[i+3] = lib::CONTACT;
	}

	//	  cout << "first_step 3" << endl;
	return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool tff_single_robot_nose_run::next_step()
{
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// UWAGA: dzialamy na jednoelementowej liscie robotow

	if (idle_step_counter) { // Oczekiwanie na odczyt aktualnego polozenia koncowki

		for (ecp_mp::sensors_t::iterator sensor_m_iterator = sensor_m.begin(); sensor_m_iterator
				!= sensor_m.end(); sensor_m_iterator++) {
			sensor_m_iterator->second->base_period=0;
			sensor_m_iterator->second->current_period=0;
		}

		idle_step_counter--;
		return true;
	}

	vsp_force->base_period=1;

	if (check_and_null_trigger()) {
		// printf("trigger\n");
		return false;
	}

	irp6->mp_command.instruction.arm.pf_def.gripper_coordinate = irp6->ecp_reply_package.reply_package.arm.pf_def.gripper_coordinate;

	irp6->mp_command.instruction.instruction_type = lib::SET_GET;

	// UWAGA: dzialamy na jednoelementowej liscie robotow
	if (irp6->ecp_reply_package.reply == lib::TASK_TERMINATED) {
		sr_ecp_msg.message("w mp task terminated");
		return false;
	} else
		return true;
}

tff_nose_run::tff_nose_run(task::task& _mp_task, int step) :
	generator(_mp_task), irp6ot_con(1), irp6p_con(1)
{
	step_no = step;
}

void tff_nose_run::configure(unsigned short l_irp6ot_con, unsigned short l_irp6p_con)
{
	irp6ot_con = l_irp6ot_con;
	irp6p_con = l_irp6p_con;
}

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool tff_nose_run::first_step()
{
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i oreintacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// cout << "first_step" << endl;
	irp6ot = robot_m[lib::ROBOT_IRP6_ON_TRACK];
	irp6p = robot_m[lib::ROBOT_IRP6_POSTUMENT];

	irp6ot->communicate = true;
	irp6p->communicate = true;

	vsp_force_irp6ot = sensor_m[lib::SENSOR_FORCE_ON_TRACK];
	vsp_force_irp6p = sensor_m[lib::SENSOR_FORCE_POSTUMENT];

	idle_step_counter = 2;
	vsp_force_irp6ot->base_period=1;
	vsp_force_irp6p->base_period=1;
	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 2;
	irp6ot->mp_command.command = lib::NEXT_POSE;
	irp6ot->mp_command.instruction.instruction_type = lib::GET;
	irp6ot->mp_command.instruction.get_type = ARM_DV;
	irp6ot->mp_command.instruction.set_type = ARM_DV | RMODEL_DV;
	irp6ot->mp_command.instruction.set_rmodel_type = lib::TOOL_FRAME;
	irp6ot->mp_command.instruction.get_rmodel_type = lib::TOOL_FRAME;
	irp6ot->mp_command.instruction.set_arm_type = lib::PF_VELOCITY;
	irp6ot->mp_command.instruction.get_arm_type = lib::FRAME;
	irp6ot->mp_command.instruction.motion_type = lib::RELATIVE;
	irp6ot->mp_command.instruction.interpolation_type = lib::TCIM;
	irp6ot->mp_command.instruction.motion_steps = td.internode_step_no;
	irp6ot->mp_command.instruction.value_in_step_no = td.value_in_step_no;

	for (int i=0; i<3; i++) {
		irp6ot->mp_command.instruction.arm.pf_def.arm_coordinates[i] = 0;
		irp6ot->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i] = 0;
		irp6ot->mp_command.instruction.arm.pf_def.arm_coordinates[i+3] = 0;
		irp6ot->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i+3] = 0;
		if (irp6ot_con) {
			irp6ot->mp_command.instruction.arm.pf_def.reciprocal_damping[i] = FORCE_RECIPROCAL_DAMPING;
			irp6ot->mp_command.instruction.arm.pf_def.behaviour[i] = lib::CONTACT;
		} else
			irp6ot->mp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
		if (irp6ot_con) {
			irp6ot->mp_command.instruction.arm.pf_def.reciprocal_damping[i+3] = TORQUE_RECIPROCAL_DAMPING;
			irp6ot->mp_command.instruction.arm.pf_def.behaviour[i+3] = lib::CONTACT;
		} else
			irp6ot->mp_command.instruction.arm.pf_def.behaviour[i+3] = lib::UNGUARDED_MOTION;

		irp6ot->mp_command.instruction.arm.pf_def.inertia[i] = FORCE_INERTIA;
		irp6ot->mp_command.instruction.arm.pf_def.inertia[i+3] = TORQUE_INERTIA;
	}

	lib::Homog_matrix tool_frame(0.0, 0.0, 0.25);
	tool_frame.get_frame_tab(irp6ot->mp_command.instruction.rmodel.tool_frame_def.tool_frame);

	irp6p->mp_command.command = lib::NEXT_POSE;
	irp6p->mp_command.instruction.instruction_type = lib::GET;
	irp6p->mp_command.instruction.get_type = ARM_DV;
	irp6p->mp_command.instruction.set_type = ARM_DV | RMODEL_DV;
	irp6p->mp_command.instruction.set_rmodel_type = lib::TOOL_FRAME;
	irp6p->mp_command.instruction.get_rmodel_type = lib::TOOL_FRAME;
	irp6p->mp_command.instruction.set_arm_type = lib::PF_VELOCITY;
	irp6p->mp_command.instruction.get_arm_type = lib::FRAME;
	irp6p->mp_command.instruction.motion_type = lib::RELATIVE;
	irp6p->mp_command.instruction.interpolation_type = lib::TCIM;
	irp6p->mp_command.instruction.motion_steps = td.internode_step_no;
	irp6p->mp_command.instruction.value_in_step_no = td.value_in_step_no;

	tool_frame.get_frame_tab(irp6p->mp_command.instruction.rmodel.tool_frame_def.tool_frame);

	for (int i=0; i<3; i++) {
		irp6p->mp_command.instruction.arm.pf_def.arm_coordinates[i] = 0;
		irp6p->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i] = 0;
		irp6p->mp_command.instruction.arm.pf_def.arm_coordinates[i+3] = 0;
		irp6p->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i+3] = 0;
		if (irp6p_con) {
			irp6p->mp_command.instruction.arm.pf_def.reciprocal_damping[i] = FORCE_RECIPROCAL_DAMPING;
			irp6p->mp_command.instruction.arm.pf_def.behaviour[i] = lib::CONTACT;
		} else
			irp6p->mp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
		if (irp6p_con) {
			irp6p->mp_command.instruction.arm.pf_def.reciprocal_damping[i+3] = TORQUE_RECIPROCAL_DAMPING;
			irp6p->mp_command.instruction.arm.pf_def.behaviour[i+3] = lib::CONTACT;
		} else
			irp6p->mp_command.instruction.arm.pf_def.behaviour[i+3] = lib::UNGUARDED_MOTION;
		irp6p->mp_command.instruction.arm.pf_def.inertia[i] = FORCE_INERTIA;
		irp6p->mp_command.instruction.arm.pf_def.inertia[i+3] = TORQUE_INERTIA;
	}

	//	  cout << "first_step 3" << endl;
	return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool tff_nose_run::next_step()
{
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// UWAGA: dzialamy na jednoelementowej liscie robotow
	//	cout << "next_step" << endl;

	if (idle_step_counter) { // Oczekiwanie na odczyt aktualnego polozenia koncowki
		for (ecp_mp::sensors_t::iterator sensor_m_iterator = sensor_m.begin(); sensor_m_iterator
				!= sensor_m.end(); sensor_m_iterator++) {
			sensor_m_iterator->second->base_period=0;
			sensor_m_iterator->second->current_period=0;
		}
		idle_step_counter--;
		return true;
	}

	vsp_force_irp6ot->base_period=1;
	vsp_force_irp6p->base_period=1;

	if (check_and_null_trigger()) {

		// printf("trigger\n");
		return false;
	}

	if (node_counter==3) {
		irp6ot->mp_command.instruction.arm.pf_def.gripper_coordinate = irp6ot->ecp_reply_package.reply_package.arm.pf_def.gripper_coordinate;
		irp6p->mp_command.instruction.arm.pf_def.gripper_coordinate = irp6p->ecp_reply_package.reply_package.arm.pf_def.gripper_coordinate;
	}

	irp6ot->mp_command.instruction.instruction_type = lib::SET_GET;
	irp6p->mp_command.instruction.instruction_type = lib::SET_GET;

	// UWAGA: dzialamy na jednoelementowej liscie robotow
	if ((irp6ot->ecp_reply_package.reply == lib::TASK_TERMINATED ) || (irp6ot->ecp_reply_package.reply == lib::TASK_TERMINATED )) {
		sr_ecp_msg.message("w mp task terminated");
		return false;
	} else
		return true;
}


tff_rubik_grab::tff_rubik_grab(task::task& _mp_task, int step) :
	generator(_mp_task), irp6ot_con(0), irp6p_con(0)
{
	step_no = step;
}

void tff_rubik_grab::configure(unsigned short l_irp6ot_con, unsigned short l_irp6p_con, double l_goal_position, double l_position_increment, int l_min_node_counter, bool l_irp6p_both_axes_running, bool l_irp6ot_both_axes_running)
{
	irp6ot_con = l_irp6ot_con;
	irp6p_con = l_irp6p_con;
	goal_position = l_goal_position;
	position_increment = l_position_increment;
	min_node_counter = l_min_node_counter;
	irp6p_both_axes_running = l_irp6p_both_axes_running;
	irp6ot_both_axes_running = l_irp6ot_both_axes_running;

}

bool tff_rubik_grab::first_step()
{
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	irp6ot = robot_m[lib::ROBOT_IRP6_ON_TRACK];
	irp6p = robot_m[lib::ROBOT_IRP6_POSTUMENT];

	if (irp6ot_con)
		irp6ot->communicate=true;
	else {
		irp6ot->communicate=false;
		irp6ot->new_pulse_checked = true;
	}
	if (irp6p_con)
		irp6p->communicate=true;
	else {
		irp6p->communicate=false;
		irp6p->new_pulse_checked = true;
	}

	vsp_force_irp6ot = sensor_m[lib::SENSOR_FORCE_ON_TRACK];
	vsp_force_irp6p = sensor_m[lib::SENSOR_FORCE_POSTUMENT];

	idle_step_counter = 1;
	vsp_force_irp6ot->base_period=1;
	vsp_force_irp6p->base_period=1;
	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 2;
	irp6ot->mp_command.command = lib::NEXT_POSE;
	irp6ot->mp_command.instruction.instruction_type = lib::GET;
	irp6ot->mp_command.instruction.get_type = ARM_DV;
	irp6ot->mp_command.instruction.set_type = ARM_DV | RMODEL_DV;
	irp6ot->mp_command.instruction.set_rmodel_type = lib::TOOL_FRAME;
	irp6ot->mp_command.instruction.get_rmodel_type = lib::TOOL_FRAME;
	irp6ot->mp_command.instruction.set_arm_type = lib::PF_VELOCITY;
	irp6ot->mp_command.instruction.get_arm_type = lib::FRAME;
	irp6ot->mp_command.instruction.motion_type = lib::RELATIVE;
	irp6ot->mp_command.instruction.interpolation_type = lib::TCIM;
	irp6ot->mp_command.instruction.motion_steps = td.internode_step_no;
	irp6ot->mp_command.instruction.value_in_step_no = td.value_in_step_no;

	lib::Homog_matrix tool_frame(0.0, 0.0, 0.25);
	tool_frame.get_frame_tab(irp6ot->mp_command.instruction.rmodel.tool_frame_def.tool_frame);

	for (int i=0; i<3; i++) {
		irp6ot->mp_command.instruction.arm.pf_def.arm_coordinates[i] = 0;
		irp6ot->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i] = 0;
		irp6ot->mp_command.instruction.arm.pf_def.arm_coordinates[i+3] = 0;
		irp6ot->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i+3] = 0;
		if (irp6ot_con) {
			irp6ot->mp_command.instruction.arm.pf_def.reciprocal_damping[i] = FORCE_RECIPROCAL_DAMPING;
			irp6ot->mp_command.instruction.arm.pf_def.behaviour[i] = lib::CONTACT;
		} else {
			irp6ot->mp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;

		}
		if (irp6ot_con) {
			irp6ot->mp_command.instruction.arm.pf_def.reciprocal_damping[i+3] = TORQUE_RECIPROCAL_DAMPING;
			irp6ot->mp_command.instruction.arm.pf_def.behaviour[i+3] = lib::CONTACT;
		} else
			irp6ot->mp_command.instruction.arm.pf_def.behaviour[i+3] = lib::UNGUARDED_MOTION;

		irp6ot->mp_command.instruction.arm.pf_def.inertia[i] = FORCE_INERTIA;
		irp6ot->mp_command.instruction.arm.pf_def.inertia[i+3] = TORQUE_INERTIA;
	}

	if (irp6ot_con) {
		if (irp6ot_both_axes_running)
			for (int i=0; i<2; i++) {
				irp6ot->mp_command.instruction.arm.pf_def.reciprocal_damping[i] = FORCE_RECIPROCAL_DAMPING;
				irp6ot->mp_command.instruction.arm.pf_def.behaviour[i] = lib::CONTACT;
			}
		else {
			irp6ot->mp_command.instruction.arm.pf_def.reciprocal_damping[1] = FORCE_RECIPROCAL_DAMPING;
			irp6ot->mp_command.instruction.arm.pf_def.behaviour[0] = lib::UNGUARDED_MOTION;
			irp6ot->mp_command.instruction.arm.pf_def.behaviour[1] = lib::CONTACT;
		}
		for (int i=2; i<6; i++)
			irp6ot->mp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
	} else
		for (int i=0; i<6; i++)
			irp6ot->mp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;

	irp6p->mp_command.command = lib::NEXT_POSE;
	irp6p->mp_command.instruction.instruction_type = lib::GET;
	irp6p->mp_command.instruction.get_type = ARM_DV;
	irp6p->mp_command.instruction.set_type = ARM_DV | RMODEL_DV;
	irp6p->mp_command.instruction.set_rmodel_type = lib::TOOL_FRAME;
	irp6p->mp_command.instruction.get_rmodel_type = lib::TOOL_FRAME;
	irp6p->mp_command.instruction.set_arm_type = lib::PF_VELOCITY;
	irp6p->mp_command.instruction.get_arm_type = lib::FRAME;
	irp6p->mp_command.instruction.motion_type = lib::RELATIVE;
	irp6p->mp_command.instruction.interpolation_type = lib::TCIM;
	irp6p->mp_command.instruction.motion_steps = td.internode_step_no;
	irp6p->mp_command.instruction.value_in_step_no = td.value_in_step_no;

	tool_frame.get_frame_tab(irp6p->mp_command.instruction.rmodel.tool_frame_def.tool_frame);

	for (int i=0; i<3; i++) {
		irp6p->mp_command.instruction.arm.pf_def.arm_coordinates[i] = 0;
		irp6p->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i] = 0;
		irp6p->mp_command.instruction.arm.pf_def.arm_coordinates[i+3] = 0;
		irp6p->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i+3] = 0;
		if (irp6p_con) {
			irp6p->mp_command.instruction.arm.pf_def.reciprocal_damping[i] = FORCE_RECIPROCAL_DAMPING;
			irp6p->mp_command.instruction.arm.pf_def.behaviour[i] = lib::CONTACT;
		} else
			irp6p->mp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
		if (irp6p_con) {
			irp6p->mp_command.instruction.arm.pf_def.reciprocal_damping[i+3] = TORQUE_RECIPROCAL_DAMPING;
			irp6p->mp_command.instruction.arm.pf_def.behaviour[i+3] = lib::CONTACT;
		} else
			irp6p->mp_command.instruction.arm.pf_def.behaviour[i+3] = lib::UNGUARDED_MOTION;
		irp6p->mp_command.instruction.arm.pf_def.inertia[i] = FORCE_INERTIA;
		irp6p->mp_command.instruction.arm.pf_def.inertia[i+3] = TORQUE_INERTIA;
	}

	if (irp6p_con) {
		if (irp6p_both_axes_running)
			for (int i=0; i<2; i++) {
				irp6p->mp_command.instruction.arm.pf_def.reciprocal_damping[i] = FORCE_RECIPROCAL_DAMPING;
				irp6p->mp_command.instruction.arm.pf_def.behaviour[i] = lib::CONTACT;
			}
		else {
			irp6p->mp_command.instruction.arm.pf_def.reciprocal_damping[1] = FORCE_RECIPROCAL_DAMPING;
			irp6p->mp_command.instruction.arm.pf_def.behaviour[1] = lib::CONTACT;
			irp6p->mp_command.instruction.arm.pf_def.behaviour[0] = lib::UNGUARDED_MOTION;
		}
		for (int i=2; i<6; i++)
			irp6p->mp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
	} else
		for (int i=0; i<6; i++)
			irp6p->mp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;

	return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool tff_rubik_grab::next_step()
{
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// UWAGA: dzialamy na jednoelementowej liscie robotow


	if (idle_step_counter) { // Oczekiwanie na odczyt aktualnego polozenia koncowki
		for (ecp_mp::sensors_t::iterator sensor_m_iterator = sensor_m.begin(); sensor_m_iterator
				!= sensor_m.end(); sensor_m_iterator++) {
			sensor_m_iterator->second->base_period=0;
			sensor_m_iterator->second->current_period=0;
		}

		idle_step_counter--;
		return true;
	}

	vsp_force_irp6ot->base_period=1;
	vsp_force_irp6p->base_period=1;

	if (check_and_null_trigger()) {

		// printf("trigger\n");
		return false;
	}

	// if (irp6ot->new_pulse) printf("irp6ot: \n");
	// if (irp6p->new_pulse) printf("irp6p: \n");

	if (node_counter==2) {
		irp6ot->mp_command.instruction.arm.pf_def.gripper_coordinate = irp6ot->ecp_reply_package.reply_package.arm.pf_def.gripper_coordinate;
		irp6p->mp_command.instruction.arm.pf_def.gripper_coordinate = irp6p->ecp_reply_package.reply_package.arm.pf_def.gripper_coordinate;
	}

	if (irp6ot_con) {
		if ((irp6ot->mp_command.instruction.arm.pf_def.gripper_coordinate > goal_position) || (node_counter < min_node_counter))
			irp6ot->mp_command.instruction.arm.pf_def.gripper_coordinate -= position_increment;
		else {
			return false;
		}
	} else
		irp6ot->mp_command.instruction.arm.pf_def.gripper_coordinate = irp6ot->ecp_reply_package.reply_package.arm.pf_def.gripper_coordinate;

	if (irp6p_con) {
		if ((irp6p->mp_command.instruction.arm.pf_def.gripper_coordinate > goal_position) || (node_counter < min_node_counter))
			irp6p->mp_command.instruction.arm.pf_def.gripper_coordinate -= position_increment;
		else {
			return false;
		}
	} else
		irp6p->mp_command.instruction.arm.pf_def.gripper_coordinate = irp6p->ecp_reply_package.reply_package.arm.pf_def.gripper_coordinate;

	irp6ot->mp_command.instruction.instruction_type = lib::SET_GET;
	irp6p->mp_command.instruction.instruction_type = lib::SET_GET;

	// UWAGA: dzialamy na jednoelementowej liscie robotow
	if (irp6ot->ecp_reply_package.reply == lib::TASK_TERMINATED) {
		sr_ecp_msg.message("w mp task terminated");
		return false;
	} else
		return true;
}

tff_rubik_face_rotate::tff_rubik_face_rotate(task::task& _mp_task, int step) :
	generator(_mp_task), irp6ot_con(1), irp6p_con(1)
{
	step_no = step;
}

void tff_rubik_face_rotate::configure(double l_irp6ot_con, double l_irp6p_con)
{
	irp6ot_con = l_irp6ot_con;
	irp6p_con = l_irp6p_con;
}

bool tff_rubik_face_rotate::first_step()
{
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i oreintacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	irp6ot = robot_m[lib::ROBOT_IRP6_ON_TRACK];
	irp6p = robot_m[lib::ROBOT_IRP6_POSTUMENT];
	irp6ot->communicate = true;
	irp6p->communicate = true;

	vsp_force_irp6ot = sensor_m[lib::SENSOR_FORCE_ON_TRACK];
	vsp_force_irp6p = sensor_m[lib::SENSOR_FORCE_POSTUMENT];

	idle_step_counter = 1;
	vsp_force_irp6ot->base_period=1;
	vsp_force_irp6p->base_period=1;
	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 2;
	irp6ot->mp_command.command = lib::NEXT_POSE;
	irp6ot->mp_command.instruction.instruction_type = lib::GET;
	irp6ot->mp_command.instruction.get_type = ARM_DV;
	irp6ot->mp_command.instruction.set_type = ARM_DV | RMODEL_DV;
	irp6ot->mp_command.instruction.set_rmodel_type = lib::TOOL_FRAME;
	irp6ot->mp_command.instruction.get_rmodel_type = lib::TOOL_FRAME;
	irp6ot->mp_command.instruction.set_arm_type = lib::PF_VELOCITY;
	irp6ot->mp_command.instruction.get_arm_type = lib::FRAME;
	irp6ot->mp_command.instruction.motion_type = lib::RELATIVE;
	irp6ot->mp_command.instruction.interpolation_type = lib::TCIM;
	irp6ot->mp_command.instruction.motion_steps = td.internode_step_no;
	irp6ot->mp_command.instruction.value_in_step_no = td.value_in_step_no;

	lib::Homog_matrix tool_frame(0.0, 0.0, 0.25);
	tool_frame.get_frame_tab(irp6ot->mp_command.instruction.rmodel.tool_frame_def.tool_frame);

	for (int i=0; i<3; i++) {
		irp6ot->mp_command.instruction.arm.pf_def.arm_coordinates[i] = 0;
		irp6ot->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i] = 0;
		irp6ot->mp_command.instruction.arm.pf_def.arm_coordinates[i+3] = 0;
		irp6ot->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i+3] = 0;
		if (irp6ot_con) {
			irp6ot->mp_command.instruction.arm.pf_def.reciprocal_damping[i] = FORCE_RECIPROCAL_DAMPING;
			irp6ot->mp_command.instruction.arm.pf_def.behaviour[i] = lib::CONTACT;
		} else
			irp6ot->mp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
		if (irp6ot_con) {
			irp6ot->mp_command.instruction.arm.pf_def.reciprocal_damping[i+3] = TORQUE_RECIPROCAL_DAMPING;
			irp6ot->mp_command.instruction.arm.pf_def.behaviour[i+3] = lib::CONTACT;
		} else
			irp6ot->mp_command.instruction.arm.pf_def.behaviour[i+3] = lib::UNGUARDED_MOTION;

		irp6ot->mp_command.instruction.arm.pf_def.inertia[i] = FORCE_INERTIA;
		irp6ot->mp_command.instruction.arm.pf_def.inertia[i+3] = TORQUE_INERTIA;
	}

	if (-0.1 < irp6ot_con && irp6ot_con < 0.1) {
		for (int i=0; i<6; i++)
			irp6ot->mp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
	} else {
		for (int i=0; i<3; i++) {
			irp6ot->mp_command.instruction.arm.pf_def.reciprocal_damping[i] = FORCE_RECIPROCAL_DAMPING;
			irp6ot->mp_command.instruction.arm.pf_def.behaviour[i] = lib::CONTACT;
		}
		for (int i=3; i<5; i++) {
			irp6ot->mp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;

		}
		irp6ot->mp_command.instruction.arm.pf_def.reciprocal_damping[5] = TORQUE_RECIPROCAL_DAMPING;
		irp6ot->mp_command.instruction.arm.pf_def.behaviour[5] = lib::CONTACT;

		if (irp6ot_con > 0.0)
			irp6ot->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[5] = -5;
		if (irp6ot_con < 0.0)
			irp6ot->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[5] = 5;
	}
	irp6p->mp_command.command = lib::NEXT_POSE;
	irp6p->mp_command.instruction.instruction_type = lib::GET;
	irp6p->mp_command.instruction.get_type = ARM_DV;
	irp6p->mp_command.instruction.set_type = ARM_DV | RMODEL_DV;
	irp6p->mp_command.instruction.set_rmodel_type = lib::TOOL_FRAME;
	irp6p->mp_command.instruction.get_rmodel_type = lib::TOOL_FRAME;
	irp6p->mp_command.instruction.set_arm_type = lib::PF_VELOCITY;
	irp6p->mp_command.instruction.get_arm_type = lib::FRAME;
	irp6p->mp_command.instruction.motion_type = lib::RELATIVE;
	irp6p->mp_command.instruction.interpolation_type = lib::TCIM;
	irp6p->mp_command.instruction.motion_steps = td.internode_step_no;
	irp6p->mp_command.instruction.value_in_step_no = td.value_in_step_no;

	tool_frame.get_frame_tab(irp6p->mp_command.instruction.rmodel.tool_frame_def.tool_frame);

	for (int i=0; i<3; i++) {
		irp6p->mp_command.instruction.arm.pf_def.arm_coordinates[i] = 0;
		irp6p->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i] = 0;
		irp6p->mp_command.instruction.arm.pf_def.arm_coordinates[i+3] = 0;
		irp6p->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i+3] = 0;
		irp6p->mp_command.instruction.arm.pf_def.reciprocal_damping[i] = FORCE_RECIPROCAL_DAMPING;
		irp6p->mp_command.instruction.arm.pf_def.reciprocal_damping[i+3] = TORQUE_RECIPROCAL_DAMPING;
		irp6p->mp_command.instruction.arm.pf_def.inertia[i] = FORCE_INERTIA;
		irp6p->mp_command.instruction.arm.pf_def.inertia[i+3] = TORQUE_INERTIA;
		irp6p->mp_command.instruction.arm.pf_def.behaviour[i] = lib::CONTACT;
		irp6p->mp_command.instruction.arm.pf_def.behaviour[i+3] = lib::CONTACT;
	}

	if (-0.1 < irp6p_con && irp6p_con < 0.1) {
		for (int i=0; i<6; i++) {
			irp6p->mp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
		}
	} else {
		for (int i=0; i<3; i++) {
			irp6p->mp_command.instruction.arm.pf_def.reciprocal_damping[i] = FORCE_RECIPROCAL_DAMPING;
			irp6p->mp_command.instruction.arm.pf_def.behaviour[i] = lib::CONTACT;
		}
		for (int i=3; i<5; i++) {
			irp6p->mp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
		}

		irp6p->mp_command.instruction.arm.pf_def.reciprocal_damping[5] = TORQUE_RECIPROCAL_DAMPING;
		irp6p->mp_command.instruction.arm.pf_def.behaviour[5] = lib::CONTACT;

		if (irp6p_con > 0.0)
			irp6p->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[5] = -5;
		if (irp6p_con < 0.0)
			irp6p->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[5] = 5;
	}

	return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool tff_rubik_face_rotate::next_step()
{
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// UWAGA: dzialamy na jednoelementowej liscie robotow

	if (idle_step_counter) { // Oczekiwanie na odczyt aktualnego polozenia koncowki
		for (ecp_mp::sensors_t::iterator sensor_m_iterator = sensor_m.begin(); sensor_m_iterator
				!= sensor_m.end(); sensor_m_iterator++) {
			sensor_m_iterator->second->base_period=0;
			sensor_m_iterator->second->current_period=0;
		}
		idle_step_counter--;
		return true;
	}
	if (node_counter==3) {
		irp6ot->mp_command.instruction.instruction_type = lib::SET_GET;
		irp6p->mp_command.instruction.instruction_type = lib::SET_GET;
		irp6p->mp_command.instruction.arm.pf_def.gripper_coordinate = irp6p->ecp_reply_package.reply_package.arm.pf_def.gripper_coordinate;
		irp6ot->mp_command.instruction.arm.pf_def.gripper_coordinate = irp6ot->ecp_reply_package.reply_package.arm.pf_def.gripper_coordinate;
		if (irp6ot_con < -0.1 || 0.1 < irp6ot_con) {
			lib::Homog_matrix frame(irp6ot->ecp_reply_package.reply_package.arm.pf_def.arm_frame);
			double xyz_eul_zyz[6];
			frame.get_xyz_euler_zyz(xyz_eul_zyz);
			double angle_to_move = (irp6ot_con / 180.0) * M_PI;
			if (xyz_eul_zyz[5] + angle_to_move < -M_PI) {
				irp6ot_stored_gamma = 2 * M_PI + xyz_eul_zyz[5] + angle_to_move;
				irp6ot_range_change = true;
			} else if (xyz_eul_zyz[5] + angle_to_move > M_PI) {
				irp6ot_stored_gamma = -2 * M_PI + xyz_eul_zyz[5] + angle_to_move;
				irp6ot_range_change = true;
			} else {
				irp6ot_stored_gamma = xyz_eul_zyz[5] + angle_to_move;
				irp6ot_range_change = false;
			}
		}
		if (irp6p_con < -0.1 || 0.1 < irp6p_con) {
			lib::Homog_matrix frame(irp6p->ecp_reply_package.reply_package.arm.pf_def.arm_frame);
			double xyz_eul_zyz[6];
			frame.get_xyz_euler_zyz(xyz_eul_zyz);
			double angle_to_move = (irp6p_con / 180.0) * M_PI;
			if (xyz_eul_zyz[5] + angle_to_move < -M_PI) {
				irp6p_stored_gamma = 2 * M_PI + xyz_eul_zyz[5] + angle_to_move;
				irp6p_range_change = true;
			} else if (xyz_eul_zyz[5] + angle_to_move > M_PI) {
				irp6p_stored_gamma = -2 * M_PI + xyz_eul_zyz[5] + angle_to_move;
				irp6p_range_change = true;
			} else {
				irp6p_stored_gamma = xyz_eul_zyz[5] + angle_to_move;
				irp6p_range_change = false;
			}
		}
	}
	if (node_counter >= 3) {
		if (irp6p_con < -0.1 || 0.1 < irp6p_con) {
			lib::Homog_matrix current_frame(irp6p->ecp_reply_package.reply_package.arm.pf_def.arm_frame);
			double xyz_eul_zyz[6];
			current_frame.get_xyz_euler_zyz(xyz_eul_zyz);
			double current_gamma = xyz_eul_zyz[5];
			if (!irp6p_range_change) {
				if(( irp6p_con < 0.0 && irp6p_stored_gamma> current_gamma)
						|| (
								irp6p_con> 0.0 && irp6p_stored_gamma < current_gamma))
						return false;
					} else {
				if ((irp6p_con < 0.0 && irp6p_stored_gamma < current_gamma) || (irp6p_con > 0.0 && irp6p_stored_gamma
						> current_gamma))
					irp6p_range_change = false;
			}
		}
		if (irp6ot_con < -0.1 || 0.1 < irp6ot_con) {
			lib::Homog_matrix current_frame(irp6ot->ecp_reply_package.reply_package.arm.pf_def.arm_frame);
			double xyz_eul_zyz[6];
			current_frame.get_xyz_euler_zyz(xyz_eul_zyz);
			double current_gamma = xyz_eul_zyz[5];
			if (!irp6ot_range_change) {
				if(( irp6ot_con < 0.0 && irp6ot_stored_gamma> current_gamma)
						|| (
								irp6ot_con> 0.0 && irp6ot_stored_gamma < current_gamma)) {
							return false;
						}
					} else {
				if ((irp6ot_con < 0.0 && irp6p_stored_gamma < current_gamma) || (irp6ot_con > 0.0 && irp6p_stored_gamma
						> current_gamma)) {
					irp6ot_range_change = false;
				}
			}
		}
	}
	vsp_force_irp6ot->base_period=1;
	vsp_force_irp6p->base_period=1;
	if (check_and_null_trigger()) {

		// printf("trigger\n");
		return false;
	}

	irp6ot->mp_command.instruction.arm.pf_def.gripper_coordinate = irp6ot->ecp_reply_package.reply_package.arm.pf_def.gripper_coordinate;
	irp6p->mp_command.instruction.arm.pf_def.gripper_coordinate = irp6p->ecp_reply_package.reply_package.arm.pf_def.gripper_coordinate;

	// UWAGA: dzialamy na jednoelementowej liscie robotow
	if (irp6ot->ecp_reply_package.reply == lib::TASK_TERMINATED) {
		sr_ecp_msg.message("w mp task terminated");
		return false;
	} else
		return true;
}

tff_gripper_approach::tff_gripper_approach(task::task& _mp_task, int step) :
	generator(_mp_task), irp6ot_speed(1.0), irp6p_speed(1.0)
{
	step_no = step;
}

void tff_gripper_approach::configure(double l_irp6ot_speed, double l_irp6p_speed, int l_motion_time)
{
	irp6ot_speed = l_irp6ot_speed;
	irp6p_speed = l_irp6p_speed;
	motion_time = l_motion_time;
}

bool tff_gripper_approach::first_step()
{
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i oreintacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	irp6ot = robot_m[lib::ROBOT_IRP6_ON_TRACK];
	irp6p = robot_m[lib::ROBOT_IRP6_POSTUMENT];

	irp6ot->communicate = true;
	irp6p->communicate = true;

	vsp_force_irp6ot = sensor_m[lib::SENSOR_FORCE_ON_TRACK];
	vsp_force_irp6p = sensor_m[lib::SENSOR_FORCE_POSTUMENT];

	idle_step_counter = 1;
	vsp_force_irp6ot->base_period=1;
	vsp_force_irp6p->base_period=1;
	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 2;
	irp6ot->mp_command.command = lib::NEXT_POSE;
	irp6ot->mp_command.instruction.instruction_type = lib::GET;
	irp6ot->mp_command.instruction.get_type = ARM_DV;
	irp6ot->mp_command.instruction.set_type = ARM_DV | RMODEL_DV;
	irp6ot->mp_command.instruction.set_rmodel_type = lib::TOOL_FRAME;
	irp6ot->mp_command.instruction.get_rmodel_type = lib::TOOL_FRAME;
	irp6ot->mp_command.instruction.set_arm_type = lib::PF_VELOCITY;
	irp6ot->mp_command.instruction.get_arm_type = lib::FRAME;
	irp6ot->mp_command.instruction.motion_type = lib::RELATIVE;
	irp6ot->mp_command.instruction.interpolation_type = lib::TCIM;
	irp6ot->mp_command.instruction.motion_steps = td.internode_step_no;
	irp6ot->mp_command.instruction.value_in_step_no = td.value_in_step_no;

	lib::Homog_matrix tool_frame(0.0, 0.0, 0.25);
	tool_frame.get_frame_tab(irp6ot->mp_command.instruction.rmodel.tool_frame_def.tool_frame);

	for (int i=0; i<3; i++) {
		irp6ot->mp_command.instruction.arm.pf_def.inertia[i] = FORCE_INERTIA;
		irp6ot->mp_command.instruction.arm.pf_def.inertia[i+3] = TORQUE_INERTIA;
	}

	for (int i=0; i<6; i++) {
		irp6ot->mp_command.instruction.arm.pf_def.arm_coordinates[i] = 0;
		irp6ot->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i] = 0;
	}

	irp6ot->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[2] = -irp6ot_speed;

	for (int i=0; i<6; i++)
		irp6ot->mp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
	irp6ot->mp_command.instruction.arm.pf_def.reciprocal_damping[2] = FORCE_RECIPROCAL_DAMPING;
	irp6ot->mp_command.instruction.arm.pf_def.behaviour[2] = lib::CONTACT;

	irp6p->mp_command.command = lib::NEXT_POSE;
	irp6p->mp_command.instruction.instruction_type = lib::GET;
	irp6p->mp_command.instruction.get_type = ARM_DV;
	irp6p->mp_command.instruction.set_type = ARM_DV | RMODEL_DV;
	irp6p->mp_command.instruction.set_rmodel_type = lib::TOOL_FRAME;
	irp6p->mp_command.instruction.get_rmodel_type = lib::TOOL_FRAME;
	irp6p->mp_command.instruction.set_arm_type = lib::PF_VELOCITY;
	irp6p->mp_command.instruction.get_arm_type = lib::FRAME;
	irp6p->mp_command.instruction.motion_type = lib::RELATIVE;
	irp6p->mp_command.instruction.interpolation_type = lib::TCIM;
	irp6p->mp_command.instruction.motion_steps = td.internode_step_no;
	irp6p->mp_command.instruction.value_in_step_no = td.value_in_step_no;

	tool_frame.get_frame_tab(irp6p->mp_command.instruction.rmodel.tool_frame_def.tool_frame);

	for (int i=0; i<3; i++) {
		irp6p->mp_command.instruction.arm.pf_def.inertia[i] = FORCE_INERTIA;
		irp6p->mp_command.instruction.arm.pf_def.inertia[i+3] = TORQUE_INERTIA;
	}

	for (int i=0; i<6; i++) {
		irp6p->mp_command.instruction.arm.pf_def.arm_coordinates[i] = 0;
		irp6p->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i] = 0;
	}

	for (int i=0; i<6; i++)
		irp6p->mp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;

	return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool tff_gripper_approach::next_step()
{
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// UWAGA: dzialamy na jednoelementowej liscie robotow

	if (idle_step_counter) { // Oczekiwanie na odczyt aktualnego polozenia koncowki
		for (ecp_mp::sensors_t::iterator sensor_m_iterator = sensor_m.begin(); sensor_m_iterator
				!= sensor_m.end(); sensor_m_iterator++) {
			sensor_m_iterator->second->base_period=0;
			sensor_m_iterator->second->current_period=0;
		}
		idle_step_counter--;
		return true;
	}
	if (node_counter==3) {
		irp6ot->mp_command.instruction.instruction_type = lib::SET_GET;
		irp6p->mp_command.instruction.instruction_type = lib::SET_GET;
		irp6p->mp_command.instruction.arm.pf_def.gripper_coordinate = irp6p->ecp_reply_package.reply_package.arm.pf_def.gripper_coordinate;
		irp6ot->mp_command.instruction.arm.pf_def.gripper_coordinate = irp6ot->ecp_reply_package.reply_package.arm.pf_def.gripper_coordinate;

	}

	if (node_counter >= 3) {
		if (node_counter > motion_time)
			return false;
	}
	vsp_force_irp6ot->base_period=1;
	vsp_force_irp6p->base_period=1;
	if (check_and_null_trigger()) {

		// printf("trigger\n");
		return false;
	}

	// UWAGA: dzialamy na jednoelementowej liscie robotow
	if (irp6ot->ecp_reply_package.reply == lib::TASK_TERMINATED) {
		sr_ecp_msg.message("w mp task terminated");
		return false;
	} else
		return true;
}

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

nose_run_force::nose_run_force(task::task& _mp_task, int step) :
	generator(_mp_task)
{
	step_no = step;
}

bool nose_run_force::first_step()
{
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i oreintacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana


	irp6ot = robot_m[lib::ROBOT_IRP6_ON_TRACK];
	irp6p = robot_m[lib::ROBOT_IRP6_POSTUMENT];
	conv = robot_m[lib::ROBOT_CONVEYOR];

	irp6ot->communicate = true;
	irp6p->communicate = true;
	conv->communicate = true;

	vsp_force_irp6ot = sensor_m[lib::SENSOR_FORCE_ON_TRACK];
	vsp_force_irp6p = sensor_m[lib::SENSOR_FORCE_POSTUMENT];

	idle_step_counter = 1;

	for (int i=0; i<6; i++)
		delta[i]=0.0;
	// wylaczenie jednego pomiaru sily
	vsp_force_irp6ot->base_period=1;
	vsp_force_irp6p->base_period=1;

	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 2;

	// track
	irp6ot->mp_command.command = lib::NEXT_POSE;
	irp6ot->mp_command.instruction.instruction_type = lib::GET;
	irp6ot->mp_command.instruction.get_type = ARM_DV;
	irp6ot->mp_command.instruction.set_type = ARM_DV;
	/*
	 irp6ot->ecp_td.force_move_mode=2; // z regulacja silowa po query
	 irp6ot->ecp_td.position_set_mode=1; // przyrostowo

	 irp6ot->ecp_td.force_axis_quantity=3; // DOBRZE

	 irp6ot->ecp_td.ECPtoEDP_force_coordinates[0]=0.0;
	 irp6ot->ecp_td.ECPtoEDP_force_coordinates[1]=0.0;
	 irp6ot->ecp_td.ECPtoEDP_force_coordinates[2]=0.0;

	 for (int j=0; j<6 ; j++)	{
	 irp6ot->ecp_td.position_increment[j]=0.0;
	 }

	 irp6ot->ecp_td.dyslocation_matrix[0][0]=1;
	 irp6ot->ecp_td.dyslocation_matrix[1][1]=1;
	 irp6ot->ecp_td.dyslocation_matrix[2][2]=1;
	 irp6ot->ecp_td.dyslocation_matrix[3][3]=0;
	 irp6ot->ecp_td.dyslocation_matrix[4][4]=0;
	 irp6ot->ecp_td.dyslocation_matrix[5][5]=0;

	 irp6ot->mp_command.instruction.set_arm_type = POSE_FORCE_TORQUE_AT_FRAME;
	 irp6ot->mp_command.instruction.get_arm_type = POSE_FORCE_TORQUE_AT_FRAME;
	 irp6ot->mp_command.instruction.motion_type = PF_MOVING_FRAME_WITH_DESIRED_FORCE_OR_SPEED;
	 irp6ot->mp_command.instruction.motion_steps = td.internode_step_no;
	 irp6ot->mp_command.instruction.value_in_step_no = td.value_in_step_no;


	 // postument
	 irp6p->mp_command.command = lib::NEXT_POSE;
	 irp6p->mp_command.instruction.instruction_type = lib::GET;
	 irp6p->mp_command.instruction.get_type = ARM_DV;
	 irp6p->mp_command.instruction.set_type = ARM_DV;

	 irp6p->ecp_td.force_move_mode=2; // z regulacja silowa po query
	 irp6p->ecp_td.position_set_mode=1; // przyrostowo

	 irp6p->ecp_td.force_axis_quantity=2; // DOBRZE

	 irp6p->ecp_td.relative_force_vector[0]=0.0;
	 irp6p->ecp_td.relative_force_vector[1]=1.0;
	 irp6p->ecp_td.relative_force_vector[2]=0.0;

	 irp6p->ecp_td.ECPtoEDP_force_coordinates[0]=0.0;
	 irp6p->ecp_td.ECPtoEDP_force_coordinates[1]=0.0;
	 irp6p->ecp_td.ECPtoEDP_force_coordinates[2]=0.0;

	 for (int j=0; j<6 ; j++)	{
	 irp6p->ecp_td.position_increment[j]=0.0;
	 }

	 irp6p->ecp_td.dyslocation_matrix[0][0]=1;
	 irp6p->ecp_td.dyslocation_matrix[1][1]=1;
	 irp6p->ecp_td.dyslocation_matrix[2][2]=1;
	 irp6p->ecp_td.dyslocation_matrix[3][3]=0;
	 irp6p->ecp_td.dyslocation_matrix[4][4]=0;
	 irp6p->ecp_td.dyslocation_matrix[5][5]=0;

	 irp6p->mp_command.instruction.set_arm_type = POSE_FORCE_LINEAR;
	 irp6p->mp_command.instruction.get_arm_type = POSE_FORCE_LINEAR;
	 irp6p->mp_command.instruction.motion_type = lib::ABSOLUTE;
	 irp6p->mp_command.instruction.motion_steps = td.internode_step_no;
	 irp6p->mp_command.instruction.value_in_step_no = td.value_in_step_no;

	 */

	// conveyor
	conv->mp_command.command = lib::NEXT_POSE;
	conv->mp_command.instruction.instruction_type = lib::GET;
	conv->mp_command.instruction.get_type = ARM_DV;
	conv->mp_command.instruction.set_type = ARM_DV;

	conv->mp_command.instruction.set_arm_type = lib::JOINT;
	conv->mp_command.instruction.get_arm_type = lib::JOINT;
	conv->mp_command.instruction.motion_type = lib::ABSOLUTE;
	conv->mp_command.instruction.interpolation_type = lib::MIM;
	conv->mp_command.instruction.motion_steps = td.internode_step_no;
	conv->mp_command.instruction.value_in_step_no = td.value_in_step_no;

	return true;

}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool nose_run_force::next_step()
{
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// UWAGA: dzialamy na jednoelementowej liscie robotow

	int i; // licznik kolejnych wspolrzednych wektora [0..6]


	if (idle_step_counter) { // Oczekiwanie na odczyt aktualnego polozenia koncowki
		// wylaczenie pomiaru sily
		for (ecp_mp::sensors_t::iterator sensor_m_iterator = sensor_m.begin(); sensor_m_iterator
				!= sensor_m.end(); sensor_m_iterator++) {
			sensor_m_iterator->second->base_period=0;
			sensor_m_iterator->second->current_period=0;
		}
		idle_step_counter--;
		return true;
	}
	// wlaczenie pomiaru sily
	vsp_force_irp6ot->base_period=1;
	vsp_force_irp6p->base_period=1;
	// Koniec wodzenia za nos
	if (check_and_null_trigger()) {

		return false;
	}

	irp6ot->mp_command.instruction.instruction_type = lib::SET_GET;

	// irp6_postument
	irp6p->mp_command.instruction.instruction_type = lib::SET_GET;

	// conveyor
	conv->mp_command.instruction.instruction_type = lib::SET_GET;
	conv->mp_command.instruction.get_type = NOTHING_DV;
	conv->mp_command.instruction.get_arm_type = lib::INVALID_END_EFFECTOR;

	for (i=0; i<6; i++) {
		conv->mp_command.instruction.arm.pf_def.arm_coordinates[i]=conv->ecp_reply_package.reply_package.arm.pf_def.arm_coordinates[i];
	}

	// UWAGA: dzialamy na jednoelementowej liscie robotow
	if (irp6ot->ecp_reply_package.reply == lib::TASK_TERMINATED) {
		sr_ecp_msg.message("w mp task terminated");
		return false;
	} else {
		return true;
	}

}

// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////
//
// 			drawing_teach_in_force
//
// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////


drawing_teach_in_force::drawing_teach_in_force(task::task& _mp_task, int step) :
	teach_in(_mp_task)
{
	step_no = step;
}

bool drawing_teach_in_force::first_step()
{

	idle_step_counter = 1;

	conv_summar_inc=0.0;

	irp6ot = robot_m[lib::ROBOT_IRP6_ON_TRACK];
	irp6p = robot_m[lib::ROBOT_IRP6_POSTUMENT];
	conv = robot_m[lib::ROBOT_CONVEYOR];

	irp6ot->communicate = true;
	irp6p->communicate = true;
	conv->communicate = true;

	vsp_force_irp6ot = sensor_m[lib::SENSOR_FORCE_ON_TRACK];
	vsp_force_irp6p = sensor_m[lib::SENSOR_FORCE_POSTUMENT];

	if (teach_or_move == YG_MOVE) {

		// 	the_robot.set_ecp_reply (lib::ECP_ACKNOWLEDGE);

		for (int i=0; i<6; i++)
			delta[i]=0.0;

		initiate_pose_list();

		gen_state = next_gen_state = 4; // jazda w powietrzu
		prev_gen_state = 0;

		// 	the_robot.mp_buffer_receive_and_send ();


		td.interpolation_node_no = 1;
		td.internode_step_no = step_no;
		td.value_in_step_no = td.internode_step_no - 2;

		// on_track

		irp6ot->mp_command.instruction.instruction_type = lib::GET;
		irp6ot->mp_command.instruction.get_type = ARM_DV;
		irp6ot->mp_command.instruction.set_type = ARM_DV;
		/*
		 irp6ot->ecp_td.force_move_mode=2;
		 irp6ot->ecp_td.position_set_mode=1; // przyrostowo

		 irp6ot->ecp_td.force_axis_quantity=1;

		 irp6ot->ecp_td.relative_force_vector[0]=0.0;
		 irp6ot->ecp_td.relative_force_vector[1]=0.0;
		 irp6ot->ecp_td.relative_force_vector[2]=1.0;

		 normalize_vector(irp6ot->ecp_td.relative_force_vector, irp6ot->ecp_td.relative_force_vector, 3);

		 irp6ot->mp_command.instruction.set_arm_type = POSE_FORCE_LINEAR;
		 irp6ot->mp_command.instruction.get_arm_type = POSE_FORCE_LINEAR;
		 irp6ot->mp_command.instruction.motion_type = lib::ABSOLUTE;
		 irp6ot->mp_command.instruction.motion_steps = td.internode_step_no;
		 irp6ot->mp_command.instruction.value_in_step_no = td.value_in_step_no;

		 irp6ot->ecp_td.ECPtoEDP_force_coordinates[2]=SILA_DOCISKUEDP;

		 irp6ot->ecp_td.dyslocation_matrix[0][0]=1;
		 irp6ot->ecp_td.dyslocation_matrix[1][1]=1;
		 irp6ot->ecp_td.dyslocation_matrix[2][2]=1;
		 irp6ot->ecp_td.dyslocation_matrix[3][3]=0;
		 irp6ot->ecp_td.dyslocation_matrix[4][4]=0;
		 irp6ot->ecp_td.dyslocation_matrix[5][5]=0;

		 // postument

		 irp6p->mp_command.instruction.instruction_type = lib::GET;
		 irp6p->mp_command.instruction.get_type = ARM_DV;
		 irp6p->mp_command.instruction.set_type = ARM_DV;
		 irp6p->ecp_td.force_move_mode=2;
		 irp6p->ecp_td.position_set_mode=1; // przyrostowo

		 irp6p->ecp_td.force_axis_quantity=1;

		 irp6p->ecp_td.relative_force_vector[0]=0.0;
		 irp6p->ecp_td.relative_force_vector[1]=0.0;
		 irp6p->ecp_td.relative_force_vector[2]=1.0;

		 normalize_vector(irp6p->ecp_td.relative_force_vector, irp6p->ecp_td.relative_force_vector, 3);

		 irp6p->mp_command.instruction.set_arm_type = POSE_FORCE_LINEAR;
		 irp6p->mp_command.instruction.get_arm_type = POSE_FORCE_LINEAR;
		 irp6p->mp_command.instruction.motion_type = lib::ABSOLUTE;
		 irp6p->mp_command.instruction.motion_steps = td.internode_step_no;
		 irp6p->mp_command.instruction.value_in_step_no = td.value_in_step_no;

		 irp6p->ecp_td.ECPtoEDP_force_coordinates[2]=SILA_DOCISKUEDP;

		 irp6p->ecp_td.dyslocation_matrix[0][0]=1;
		 irp6p->ecp_td.dyslocation_matrix[1][1]=1;
		 irp6p->ecp_td.dyslocation_matrix[2][2]=1;
		 irp6p->ecp_td.dyslocation_matrix[3][3]=0;
		 irp6p->ecp_td.dyslocation_matrix[4][4]=0;
		 irp6p->ecp_td.dyslocation_matrix[5][5]=0;
		 */

		// conveyor
		conv->mp_command.command = lib::NEXT_POSE;
		conv->mp_command.instruction.instruction_type = lib::GET;
		conv->mp_command.instruction.get_type = ARM_DV;
		conv->mp_command.instruction.set_type = ARM_DV;

		conv->mp_command.instruction.set_arm_type = lib::JOINT;
		conv->mp_command.instruction.get_arm_type = lib::JOINT;
		conv->mp_command.instruction.motion_type = lib::ABSOLUTE;
		conv->mp_command.instruction.interpolation_type = lib::MIM;
		conv->mp_command.instruction.motion_steps = td.internode_step_no;
		conv->mp_command.instruction.value_in_step_no = td.value_in_step_no;

		return true;

		// UCZENIE

	} else if (teach_or_move == YG_TEACH) {

		// the_robot.set_ecp_reply (lib::ECP_ACKNOWLEDGE);

		// zerowanie odczytow
		vsp_force_irp6ot->to_vsp.parameters=1;
		vsp_force_irp6ot->configure_sensor();
		vsp_force_irp6ot->to_vsp.parameters=4;
		vsp_force_irp6ot->configure_sensor();

		// 	vsp_force_irp6p->to_vsp.parameters=1;
		// 	vsp_force_irp6p->configure_sensor();
		// 	vsp_force_irp6p->to_vsp.parameters=4;
		// 	vsp_force_irp6p->configure_sensor();


		gen_state = 0; // nie zapisuje trajektorii dopoki nie osiagnie chociaz raz podloza

		for (int i=0; i<6; i++)
			delta[i]=0.0;

		create_pose_list_head(emptyps, 0.0, 2, delta);

		// 	the_robot.mp_buffer_receive_and_send ();


		td.interpolation_node_no = 1;
		td.internode_step_no = step_no;
		td.value_in_step_no = td.internode_step_no - 2;

		irp6ot->mp_command.instruction.instruction_type = lib::GET;
		irp6ot->mp_command.instruction.get_type = ARM_DV;
		irp6ot->mp_command.instruction.set_type = ARM_DV;

		/*
		 irp6ot->ecp_td.force_axis_quantity=3;

		 irp6ot->mp_command.instruction.set_arm_type = POSE_FORCE_LINEAR;
		 irp6ot->mp_command.instruction.get_arm_type = POSE_FORCE_LINEAR;
		 irp6ot->mp_command.instruction.motion_type = lib::ABSOLUTE;
		 irp6ot->mp_command.instruction.motion_steps = td.internode_step_no;
		 irp6ot->mp_command.instruction.value_in_step_no = td.value_in_step_no;

		 irp6ot->ecp_td.ECPtoEDP_force_coordinates[0]=0;
		 irp6ot->ecp_td.ECPtoEDP_force_coordinates[1]=0;
		 irp6ot->ecp_td.ECPtoEDP_force_coordinates[2]=SILA_DOCISKUEDP;

		 for (int j=0; j<6 ; j++)	{
		 irp6ot->ecp_td.position_increment[j]=0.0;
		 }
		 // zerowy przyrost pozycji

		 irp6ot->ecp_td.dyslocation_matrix[0][0]=1;
		 irp6ot->ecp_td.dyslocation_matrix[1][1]=1;
		 irp6ot->ecp_td.dyslocation_matrix[2][2]=1;
		 irp6ot->ecp_td.dyslocation_matrix[3][3]=0;
		 irp6ot->ecp_td.dyslocation_matrix[4][4]=0;
		 irp6ot->ecp_td.dyslocation_matrix[5][5]=0;

		 // postument
		 irp6p->mp_command.command = lib::NEXT_POSE;
		 irp6p->mp_command.instruction.instruction_type = lib::GET;
		 irp6p->mp_command.instruction.get_type = ARM_DV;
		 irp6p->mp_command.instruction.set_type = ARM_DV;

		 irp6p->mp_command.instruction.set_arm_type = lib::JOINT;
		 irp6p->mp_command.instruction.get_arm_type = lib::JOINT;
		 irp6p->mp_command.instruction.motion_type = lib::ABSOLUTE;
		 irp6p->mp_command.instruction.motion_steps = td.internode_step_no;
		 irp6p->mp_command.instruction.value_in_step_no = td.value_in_step_no;
		 */

		// conveyor
		conv->mp_command.command = lib::NEXT_POSE;
		conv->mp_command.instruction.instruction_type = lib::GET;
		conv->mp_command.instruction.get_type = ARM_DV;
		conv->mp_command.instruction.set_type = ARM_DV;

		conv->mp_command.instruction.set_arm_type = lib::JOINT;
		conv->mp_command.instruction.get_arm_type = lib::JOINT;
		conv->mp_command.instruction.motion_type = lib::ABSOLUTE;
		conv->mp_command.instruction.interpolation_type = lib::MIM;
		conv->mp_command.instruction.motion_steps = td.internode_step_no;
		conv->mp_command.instruction.value_in_step_no = td.value_in_step_no;

		return true;

	}

	return true;
}
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
bool drawing_teach_in_force::next_step()
{
	if (idle_step_counter) { // Oczekiwanie na odczyt aktualnego polozenia koncowki
		// wylaczenie pomiaru sily
		for (ecp_mp::sensors_t::iterator sensor_m_iterator = sensor_m.begin(); sensor_m_iterator
				!= sensor_m.end(); sensor_m_iterator++) {
			sensor_m_iterator->second->base_period=0;
			sensor_m_iterator->second->current_period=0;
		}
		idle_step_counter--;
		return true;
	}

	// wlaczenie pomiaru sily
	vsp_force_irp6ot->base_period=1;
	vsp_force_irp6p->base_period=1;

	if (teach_or_move == YG_MOVE) {

		common::mp_taught_in_pose tip; // Nauczona pozycja

		if (!(is_pose_list_element())) { // Koniec odcinka
			vsp_force_irp6ot->to_vsp.parameters = 6;
			vsp_force_irp6ot->configure_sensor();

			vsp_force_irp6p->to_vsp.parameters = 6;
			vsp_force_irp6p->configure_sensor();
			// 	the_robot.mp_buffer_receive_and_send ();
			// flush_pose_list();
			return false;
		}

		// irp6_on_track
		irp6ot->mp_command.instruction.instruction_type = lib::SET_GET;

		// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
		irp6p->mp_command.instruction.instruction_type = lib::SET_GET;

		// wersja dwurobotowa
		conv->mp_command.instruction.instruction_type = lib::SET_GET;
		conv->mp_command.instruction.get_type = NOTHING_DV;
		conv->mp_command.instruction.get_arm_type = lib::INVALID_END_EFFECTOR;

		get_pose(tip);

		gen_state=next_gen_state;

		switch (gen_state) {
			case 0:
				break;
			case 1:
				break;
			case 2: // powierzchnia

				if (prev_gen_state != gen_state) {
					sr_ecp_msg.message("ECP Powierzchnia");
				}

				td.interpolation_node_no = 1;
				td.internode_step_no = step_no;
				td.value_in_step_no = td.internode_step_no - 2;
				/*
				 irp6p->ecp_td.force_axis_quantity=1;

				 irp6p->mp_command.instruction.motion_steps = td.internode_step_no;
				 irp6p->mp_command.instruction.value_in_step_no = td.value_in_step_no;

				 // inkrementacja numeru iteracji dla biezacego stanu
				 in_state_iteration++;

				 #define ST2_LOW_SEGMENT 20

				 if (in_state_iteration < ST2_LOW_SEGMENT)
				 {
				 irp6ot->ecp_td.ECPtoEDP_force_coordinates[2]= (short) (MIN_SILA_DOCISKUEDP +
				 (in_state_iteration)*(SILA_DOCISKUEDP - MIN_SILA_DOCISKUEDP) /	(ST2_LOW_SEGMENT));
				 irp6p->ecp_td.ECPtoEDP_force_coordinates[2]= (short) (MIN_SILA_DOCISKUEDP +
				 (in_state_iteration)*(SILA_DOCISKUEDP - MIN_SILA_DOCISKUEDP) /	(ST2_LOW_SEGMENT));
				 } else
				 {
				 irp6ot->ecp_td.ECPtoEDP_force_coordinates[2]=SILA_DOCISKUEDP;
				 irp6p->ecp_td.ECPtoEDP_force_coordinates[2]=SILA_DOCISKUEDP;
				 }


				 irp6ot->ecp_td.position_increment[0]=tip.coordinates[0];
				 // ponizej wersja jednorobotowa
				 // irp6ot->ecp_td.position_increment[1]=tip.coordinates[1];
				 // wariant z tasmociagiem na osi Y
				 irp6ot->ecp_td.position_increment[1]=0.0;
				 irp6ot->ecp_td.position_increment[2]=0.0;
				 irp6ot->ecp_td.position_increment[3]=0.0;
				 irp6ot->ecp_td.position_increment[4]=0.0;
				 irp6ot->ecp_td.position_increment[5]=0.0;


				 irp6p->ecp_td.position_increment[0]=tip.coordinates[0];
				 // ponizej wersja jednorobotowa
				 // irp6ot->ecp_td.position_increment[1]=tip.coordinates[1];
				 // wariant z tasmociagiem na osi Y
				 irp6p->ecp_td.position_increment[1]=0.0;
				 irp6p->ecp_td.position_increment[2]=0.0;
				 irp6p->ecp_td.position_increment[3]=0.0;
				 irp6p->ecp_td.position_increment[4]=0.0;
				 irp6p->ecp_td.position_increment[5]=0.0;

				 // drugi robot - usunac w wersji jednorobotowej
				 conv_summar_inc+=tip.coordinates[1];

				 if ((tip.extra_info == 3)||(tip.extra_info == 4)||(tip.extra_info == 5)) {
				 // nauczona trajektoria przeszla w unoszenie
				 next_gen_state = 3;
				 in_state_iteration = 0;
				 } else {
				 next_pose_list_ptr ();
				 }
				 */
				break;

			case 3: // unoszenie
				if (prev_gen_state != gen_state) {
					sr_ecp_msg.message("ECP Unoszenie");
				}

				td.interpolation_node_no = 1;
				td.internode_step_no = 500;
				td.value_in_step_no = td.internode_step_no - 2;

				/*
				 irp6ot->ecp_td.force_axis_quantity=0;

				 irp6ot->mp_command.instruction.motion_steps = td.internode_step_no;
				 irp6ot->mp_command.instruction.value_in_step_no = td.value_in_step_no;

				 irp6ot->ecp_td.position_increment[0]=0.0;
				 irp6ot->ecp_td.position_increment[1]=0.0;
				 irp6ot->ecp_td.position_increment[2]=0.01;
				 irp6ot->ecp_td.position_increment[3]=0.0;
				 irp6ot->ecp_td.position_increment[4]=0.0;
				 irp6ot->ecp_td.position_increment[5]=0.0;


				 irp6p->ecp_td.force_axis_quantity=0;

				 irp6p->mp_command.instruction.motion_steps = td.internode_step_no;
				 irp6p->mp_command.instruction.value_in_step_no = td.value_in_step_no;

				 irp6p->ecp_td.position_increment[0]=0.0;
				 irp6p->ecp_td.position_increment[1]=0.0;
				 irp6p->ecp_td.position_increment[2]=0.01;
				 irp6p->ecp_td.position_increment[3]=0.0;
				 irp6p->ecp_td.position_increment[4]=0.0;
				 irp6p->ecp_td.position_increment[5]=0.0;
				 */
				next_gen_state = 4;

				break;

			case 4: // uniesienie

				if (prev_gen_state != gen_state) {
					sr_ecp_msg.message("ECP Uniesienie");
				}

				td.interpolation_node_no = 1;
				td.internode_step_no = step_no;
				td.value_in_step_no = td.internode_step_no - 2;

				/*
				 irp6ot->ecp_td.force_axis_quantity = 0;

				 irp6ot->mp_command.instruction.motion_steps = td.internode_step_no;
				 irp6ot->mp_command.instruction.value_in_step_no = td.value_in_step_no;

				 irp6ot->ecp_td.position_increment[0]=tip.coordinates[0];
				 // ponziej wersja jednorobotowa
				 // irp6ot->ecp_td.position_increment[1]=tip.coordinates[1];
				 // wariant z tasmociagiem na osi Y
				 irp6ot->ecp_td.position_increment[1]=0.0;
				 irp6ot->ecp_td.position_increment[2]=0.0;
				 irp6ot->ecp_td.position_increment[3]=0.0;
				 irp6ot->ecp_td.position_increment[4]=0.0;
				 irp6ot->ecp_td.position_increment[5]=0.0;


				 irp6p->ecp_td.force_axis_quantity = 0;

				 irp6p->mp_command.instruction.motion_steps = td.internode_step_no;
				 irp6p->mp_command.instruction.value_in_step_no = td.value_in_step_no;

				 irp6p->ecp_td.position_increment[0]=tip.coordinates[0];
				 // ponziej wersja jednorobotowa
				 // irp6ot->ecp_td.position_increment[1]=tip.coordinates[1];
				 // wariant z tasmociagiem na osi Y
				 irp6p->ecp_td.position_increment[1]=0.0;
				 irp6p->ecp_td.position_increment[2]=0.0;
				 irp6p->ecp_td.position_increment[3]=0.0;
				 irp6p->ecp_td.position_increment[4]=0.0;
				 irp6p->ecp_td.position_increment[5]=0.0;
				 */
				// drugi robot - usunac w wersji jednorobotowej
				conv_summar_inc+=tip.coordinates[1];

				if (tip.extra_info == 2) { // nauczona trajektoria osiagnela powierzchnie
					next_gen_state = 5;
					vsp_force_irp6ot->to_vsp.parameters = 5;
					vsp_force_irp6ot->configure_sensor();
					vsp_force_irp6p->to_vsp.parameters = 5;
					vsp_force_irp6p->configure_sensor();
				} else {
					next_pose_list_ptr();
				}

				break;

			case 5: // opuszczanie

				if (prev_gen_state != gen_state) {
					sr_ecp_msg.message("ECP Opuszczanie");
				}

				// 		if (vsp_force_irp6ot->image.sensor_union.force.event_type == 2) {

				if ((vsp_force_irp6ot->image.sensor_union.force.event_type == 2)&&(vsp_force_irp6p->image.sensor_union.force.event_type == 2)) {
					// czujnik wyczul powierzchnie
					next_gen_state = 6;
					in_state_iteration=0;
				} else {

					td.interpolation_node_no = 1;
					td.internode_step_no = step_no;
					td.value_in_step_no = td.internode_step_no - 2;
					/*
					 irp6ot->ecp_td.force_axis_quantity=1;
					 irp6ot->mp_command.instruction.motion_steps = td.internode_step_no;
					 irp6ot->mp_command.instruction.value_in_step_no = td.value_in_step_no;
					 irp6ot->ecp_td.ECPtoEDP_force_coordinates[2]=SILA_DOCISKUEDP_OPADANIE;

					 irp6ot->ecp_td.position_increment[0]=0.0;
					 irp6ot->ecp_td.position_increment[1]=0.0;
					 irp6ot->ecp_td.position_increment[2]=0.0;
					 irp6ot->ecp_td.position_increment[3]=0.0;
					 irp6ot->ecp_td.position_increment[4]=0.0;
					 irp6ot->ecp_td.position_increment[5]=0.0;

					 irp6p->ecp_td.force_axis_quantity=1;
					 irp6p->mp_command.instruction.motion_steps = td.internode_step_no;
					 irp6p->mp_command.instruction.value_in_step_no = td.value_in_step_no;
					 irp6p->ecp_td.ECPtoEDP_force_coordinates[2]=SILA_DOCISKUEDP_OPADANIE;

					 irp6p->ecp_td.position_increment[0]=0.0;
					 irp6p->ecp_td.position_increment[1]=0.0;
					 irp6p->ecp_td.position_increment[2]=0.0;
					 irp6p->ecp_td.position_increment[3]=0.0;
					 irp6p->ecp_td.position_increment[4]=0.0;
					 irp6p->ecp_td.position_increment[5]=0.0;
					 */
				}

				break;
			case 6: // zetkniecie z kartka

				if (prev_gen_state != gen_state) {
					sr_ecp_msg.message("ECP Zetkniecie");
				}

				in_state_iteration++;

				td.interpolation_node_no = 1;
				td.internode_step_no = step_no;
				td.value_in_step_no = td.internode_step_no - 2;
				/*
				 irp6ot->ecp_td.force_axis_quantity=1;
				 irp6ot->mp_command.instruction.motion_steps = td.internode_step_no;
				 irp6ot->mp_command.instruction.value_in_step_no = td.value_in_step_no;

				 irp6ot->ecp_td.position_increment[0]=0.0;
				 irp6ot->ecp_td.position_increment[1]=0.0;
				 irp6ot->ecp_td.position_increment[2]=0.0;
				 irp6ot->ecp_td.position_increment[3]=0.0;
				 irp6ot->ecp_td.position_increment[4]=0.0;
				 irp6ot->ecp_td.position_increment[5]=0.0;

				 irp6p->ecp_td.force_axis_quantity=1;
				 irp6p->mp_command.instruction.motion_steps = td.internode_step_no;
				 irp6p->mp_command.instruction.value_in_step_no = td.value_in_step_no;

				 irp6p->ecp_td.position_increment[0]=0.0;
				 irp6p->ecp_td.position_increment[1]=0.0;
				 irp6p->ecp_td.position_increment[2]=0.0;
				 irp6p->ecp_td.position_increment[3]=0.0;
				 irp6p->ecp_td.position_increment[4]=0.0;
				 irp6p->ecp_td.position_increment[5]=0.0;

				 #define ST6_LOW_SEGMENT 100

				 // dobor sily docisku - chcemy ladnie uderzyc w powierzchnie
				 if (in_state_iteration<ST6_LOW_SEGMENT)
				 {
				 irp6ot->ecp_td.ECPtoEDP_force_coordinates[2]=MIN_SILA_DOCISKUEDP;
				 irp6p->ecp_td.ECPtoEDP_force_coordinates[2]=MIN_SILA_DOCISKUEDP;
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

		// DRUGI ROBOT - CONVEYOR
		conv->mp_command.instruction.arm.pf_def.arm_coordinates[0]= conv->ecp_reply_package.reply_package.arm.pf_def.arm_coordinates[0]+conv_summar_inc;

		return true;

		// UCZENIE

	} else if (teach_or_move == YG_TEACH) {

		int i; // licznik kolejnych wspolrzednych wektora [0..6]

		double inc_delta[6]= { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

		if (check_and_null_trigger()) { // Koniec odcinka
			// 	the_robot.mp_buffer_receive_and_send ();

			initiate_pose_list();
			return false;
		}

		for (i=0; i<6; i++) {
			inc_delta[i]=-irp6ot->ecp_reply_package.reply_package.arm.pf_def.arm_coordinates[i];
		}

		for (i=0; i<6; i++) {
			inc_delta[i]+=irp6ot->ecp_reply_package.reply_package.arm.pf_def.arm_coordinates[i];
		}

		// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
		irp6ot->mp_command.instruction.instruction_type = lib::SET_GET;

		// irp6_postument
		irp6p->mp_command.instruction.instruction_type = lib::SET_GET;
		irp6p->mp_command.instruction.get_type = NOTHING_DV;
		irp6p->mp_command.instruction.get_arm_type = lib::INVALID_END_EFFECTOR;

		for (i=0; i<6; i++) {
			irp6p->mp_command.instruction.arm.pf_def.arm_coordinates[i]=irp6p->ecp_reply_package.reply_package.arm.pf_def.arm_coordinates[i];
		}

		irp6p->mp_command.instruction.instruction_type = lib::SET_GET;
		irp6p->mp_command.instruction.get_type = NOTHING_DV;
		irp6p->mp_command.instruction.get_arm_type = lib::INVALID_END_EFFECTOR;

		// wersja dwurobotowa
		conv->mp_command.instruction.instruction_type = lib::SET_GET;
		conv->mp_command.instruction.get_type = NOTHING_DV;
		conv->mp_command.instruction.get_arm_type = lib::INVALID_END_EFFECTOR;

		conv->mp_command.instruction.arm.pf_def.arm_coordinates[0]= conv->ecp_reply_package.reply_package.arm.pf_def.arm_coordinates[0];

		// 	if (vsp_force_irp6ot->image.sensor_union.force.event_type==2) {
		if (vsp_force_irp6ot->image.sensor_union.force.event_type==2) {
			gen_state = 1;
		}

		if (gen_state == 1) {
			insert_pose_list_element(emptyps, 0.0, vsp_force_irp6ot->image.sensor_union.force.event_type, inc_delta);
		}

		return true;

	}

	return true;

}
// --------------------------------------------------------------------------

} // namespace generator
} // namespace mp
} // namespace mrrocpp

