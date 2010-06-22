/*
 * $Id$
 *
 *  Created on: Mar 3, 2010
 *      Author: mboryn
 */

#include "visual_servo_manager.h"

using namespace logger;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

visual_servo_manager::visual_servo_manager(mrrocpp::ecp::common::task::task & ecp_task, const std::string& section_name) :
	generator(ecp_task), current_position_saved(false), motion_steps(30), max_speed(0), max_angular_speed(0),
			max_acceleration(0), max_angular_acceleration(0)
{
	// 2 ms per one step
	dt = motion_steps * 0.002;

	max_speed = ecp_task.config.value <double> ("v_max", section_name);
	max_angular_speed = ecp_task.config.value <double> ("omega_max", section_name);
	max_acceleration = ecp_task.config.value <double> ("a_max", section_name);
	max_angular_acceleration = ecp_task.config.value <double> ("epsilon_max", section_name);

	//	log("a_max: %lg, v_max: %lg\n", a_max, v_max);
	//	log("v_max * dt: %lg\n", v_max * dt);
}

visual_servo_manager::~visual_servo_manager()
{
}

bool visual_servo_manager::first_step()
{
	log_dbg("ecp_g_ib_eih::first_step()\n");

	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
	the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.set_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
	the_robot->ecp_command.instruction.motion_steps = motion_steps;
	the_robot->ecp_command.instruction.value_in_step_no = motion_steps - 3;

	for (int i = 0; i < 6; i++) {
		the_robot->ecp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
	}

	current_position_saved = false;

	//	sigevent ev;
	//	SIGEV_NONE_INIT(&ev);
	//
	//	if (timer_create(CLOCK_REALTIME, &ev, &timerek) < 0) {
	//		log("timer_create(CLOCK_REALTIME, ev, timerek) < 0: %d\n", errno);
	//	}
	//
	//	setup_timer();

	prev_velocity.setZero();
	prev_angular_velocity.setZero();
	velocity.setZero();
	angular_velocity.setZero();
	acceleration.setZero();
	angular_acceleration.setZero();

	for (int i = 0; i < termination_conditions.size(); ++i) {
		termination_conditions[i]->reset();
	}

	return true;
}

//void visual_servo_manager::setup_timer()
//{
//	max_t.it_value.tv_sec = 1;
//	max_t.it_value.tv_nsec = 0;
//	timer_settime(timerek, 0, &max_t, NULL);
//}

bool visual_servo_manager::next_step()
{
	//	timer_gettime(timerek, &curr_t);
	//	log("timer_gettime(timerek, &curr_t): %d.%09d\n", curr_t.it_value.tv_sec, curr_t.it_value.tv_nsec);
	//	setup_timer();

	//	log_dbg("bool visual_servo_manager::next_step() begin\n");
	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;

	if (!current_position_saved) { // save first frame
		//log_dbg("ecp_g_ib_eih::next_step() 1\n");
		current_position.set_from_frame_tab(the_robot->reply_package.arm.pf_def.arm_frame);

		current_position_saved = true;
	}

	// get readings from all servos and aggregate
	lib::Homog_matrix position_change = get_aggregated_position_change();
	//	log_dbg("bool visual_servo_manager::next_step(): position_change = (%+07.3lg, %+07.3lg, %+07.3lg)\n", position_change(0, 3), position_change(1, 3), position_change(2, 3));

	lib::Homog_matrix next_position = current_position * position_change;

	//	log_dbg("bool visual_servo_manager::next_step(): next_position = (%+07.3lg, %+07.3lg, %+07.3lg)\n", next_position(0, 3), next_position(1, 3), next_position(2, 3));

	// apply weak position constraints
	bool constraints_kept = false;
	for (int i = 0; i < position_constraints.size(); ++i) {
		if (position_constraints[i]->is_position_ok(next_position)) {
			constraints_kept = true;
		}
	}
	if (!constraints_kept && position_constraints.size() > 0) {
		position_constraints[0]->apply_constraint(next_position);
	}

	position_change = (!current_position) * next_position;

	lib::Xyz_Angle_Axis_vector aa_vector;
	position_change.get_xyz_angle_axis(aa_vector);
	Eigen::Matrix <double, 3, 1> ds = aa_vector.block(0, 0, 3, 1);
	Eigen::Matrix <double, 3, 1> dalpha = aa_vector.block(3, 0, 3, 1);

	constrain_vector(ds, prev_velocity, velocity, acceleration, max_speed, max_acceleration);
	constrain_vector(dalpha, prev_angular_velocity, angular_velocity, angular_acceleration, max_angular_speed, max_angular_acceleration);

	aa_vector.block(0, 0, 3, 1) = ds;
	aa_vector.block(3, 0, 3, 1) = dalpha;

	position_change.set_from_xyz_angle_axis(aa_vector);

	next_position = current_position * position_change;

	// update speed and acceleration in termination conditions
	for (int i = 0; i < termination_conditions.size(); ++i) {
		termination_conditions[i]->update_end_effector_speed(velocity);
		termination_conditions[i]->update_end_effector_accel(acceleration);
	}

	//	log_dbg("bool visual_servo_manager::next_step(): next_position = (%+07.3lg, %+07.3lg, %+07.3lg)\n", next_position(0, 3), next_position(1, 3), next_position(2, 3));
	// send command to the robot
	next_position.get_frame_tab(the_robot->ecp_command.instruction.arm.pf_def.arm_frame);

	// save next position
	current_position = next_position;

	bool object_visible = false;
	for (std::vector <boost::shared_ptr <visual_servo> >::iterator it = servos.begin(); it != servos.end(); ++it) {
		object_visible = object_visible || (*it)->is_object_visible();
	}

	// update object visiblity in termination conditions
	for (int i = 0; i < termination_conditions.size(); ++i) {
		termination_conditions[i]->update_object_visibility(object_visible);
	}

	// check termination conditions
	for (int i = 0; i < termination_conditions.size(); ++i) {
		if (termination_conditions[i]->terminate_now()) {
			return false;
		}
	}

	return true;
} // next_step()


void visual_servo_manager::constrain_vector(Eigen::Matrix <double, 3, 1> &ds, Eigen::Matrix <double, 3, 1> &prev_v, Eigen::Matrix <
		double, 3, 1> &v, Eigen::Matrix <double, 3, 1> &a, double max_v, double max_a)
{
	// apply speed constraints
	double ds_norm = ds.norm();
	if (ds_norm > (max_v * dt)) {
		ds = ds * ((max_v * dt) / ds_norm);
	}

	v = ds / dt;

	// apply acceleration constraints
	Eigen::Matrix <double, 3, 1> dv = v - prev_v;
	double dv_norm = dv.norm();
	if (dv_norm > (max_a * dt)) {
		dv = dv * ((max_a * dt) / dv_norm);
		v = prev_v + dv;
		ds = v * dt;
	}

	a = dv / dt;

	prev_v = v;
}

const lib::Homog_matrix& visual_servo_manager::get_current_position() const
{
	return current_position;
}

void visual_servo_manager::configure(const std::string & sensor_prefix)
{
	//	log_dbg("void visual_servo_manager::configure() 1\n");
	configure_all_servos();
	//	log_dbg("void visual_servo_manager::configure() 2\n");
	int i = 0;
	for (std::vector <boost::shared_ptr <visual_servo> >::iterator it = servos.begin(); it != servos.end(); ++it, ++i) {
		(*it)->get_vsp_fradia()->configure_sensor();
		char sensor_suffix[64];
		sprintf(sensor_suffix, "%02d", i);
		lib::SENSOR_t sensor_id = sensor_prefix + sensor_suffix;
		sensor_m[sensor_id] = (*it)->get_vsp_fradia().get();
	}
	//	log_dbg("void visual_servo_manager::configure() 3\n");
}

void visual_servo_manager::add_position_constraint(boost::shared_ptr <position_constraint> new_constraint)
{
	position_constraints.push_back(new_constraint);
}

void visual_servo_manager::add_termination_condition(boost::shared_ptr <termination_condition> term_cond)
{
	termination_conditions.push_back(term_cond);
}

} // namespace generator

} // namespace common

} // namespace ecp

} // namespace mrrocpp
