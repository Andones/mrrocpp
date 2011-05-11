/*
 * g_rotate.cc
 *
 *  Created on: Apr 13, 2010
 *      Author: mmichnie
 */

#include "g_rotate.h"

#include <iostream>

using namespace std;

#define MOTION_STEPS 50

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

const char g_rotate::configSectionName[] = { "[g_rotate]" };

g_rotate::g_rotate(mrrocpp::ecp::common::task::task & _ecp_task) :
	common::generator::generator(_ecp_task)
//:generator(_ecp_task), logEnabled(true)
{
	index = 0;
	r = 0.05;
	k = 0.0;
	direction = -1;
	k_max = 0.0;
}

g_rotate::~g_rotate()
{
}
/**
 * direction to move:
 * 0 - -Y up (robot)
 * 1 -  X right
 * 2 -  Y down (computer)
 * 3 - -X left
*/
void g_rotate::configure(int new_direction, double new_k_max)
{
	index = 0;
	k = 0.0;
	direction = new_direction;
	k_max = new_k_max;
}

bool g_rotate::first_step()
{
	log("g_rotate::first_step()\n");

	the_robot->ecp_command.instruction_type = lib::GET;
	the_robot->ecp_command.get_type = ARM_DEFINITION; // arm - ORYGINAL
	the_robot->ecp_command.set_type = ARM_DEFINITION;
	the_robot->ecp_command.get_arm_type = lib::JOINT;
	the_robot->ecp_command.set_arm_type = lib::JOINT;

	the_robot->ecp_command.interpolation_type = lib::TCIM;
	the_robot->ecp_command.motion_steps = MOTION_STEPS;
	the_robot->ecp_command.value_in_step_no = MOTION_STEPS - 3;
	the_robot->ecp_command.motion_type = lib::ABSOLUTE;

	log("g_rotate::first_step() end\n");

	return true;
}

bool g_rotate::next_step()
{
	the_robot->ecp_command.instruction_type = lib::SET_GET;
	the_robot->ecp_command.get_type = ARM_DEFINITION; // arm - ORYGINAL
	the_robot->ecp_command.set_type = ARM_DEFINITION;
	the_robot->ecp_command.get_arm_type = lib::JOINT;//polozenie w xyz w macierzy 3na4
	the_robot->ecp_command.set_arm_type = lib::JOINT;

	the_robot->ecp_command.interpolation_type = lib::TCIM;
	the_robot->ecp_command.motion_steps = MOTION_STEPS;
	the_robot->ecp_command.value_in_step_no = MOTION_STEPS - 3;
	the_robot->ecp_command.motion_type = lib::ABSOLUTE;//polozenie od srodka postumenta


	if (index == 0) {
		//currentFrame = the_robot->reply_package.arm.pf_def.arm_frame;

		//currentFrame.get_translation_vector(first_trans_vect);
		//std::cout << currentFrame << std::endl;
		for (size_t i = 0; i < lib::MAX_SERVOS_NR; ++i)
		{
			first_arm_coordinates[i] = the_robot->reply_package.arm.pf_def.arm_coordinates[i];
		}
		index++;
	}

	//lib::Homog_matrix nextFrame;
	//nextFrame = currentFrame;

		//q1 = the_robot->reply_package.arm.pf_def.arm_coordinates[0];
		//the_robot->ecp_command.arm.pf_def.arm_coordinates[i] = the_robot->reply_package.arm.pf_def.arm_coordinates[i];

	//double trans_vect[3];

	/*modyfikuj nextFrame*/
	//nextFrame.get_translation_vector(trans_vect);

	//move direction
	/*
	if(direction==1)//up
	{
		trans_vect[0] = first_trans_vect[0] - r * k;
		trans_vect[1] = first_trans_vect[1] + r * k;
	}
	if(direction==2)//right
	{
		trans_vect[1] = first_trans_vect[1] + r * k;
		trans_vect[0] = first_trans_vect[0] + r * k;
	}
	if(direction==3)//down
	{
		trans_vect[0] = first_trans_vect[0] + r * k;
		trans_vect[1] = first_trans_vect[1] - r * k;
	}
	if(direction==0)//left
	{
		trans_vect[1] = first_trans_vect[1] - r * k;
		trans_vect[0] = first_trans_vect[0] - r * k;
	}

	k += 0.1;
	nextFrame.set_translation_vector(trans_vect);
	*/
	/*koniec modyfikacji*/

	//pobierz
	for (size_t i = 0; i < lib::MAX_SERVOS_NR; ++i)
	{
		current_arm_coordinates[i] = the_robot->reply_package.arm.pf_def.arm_coordinates[i];
	}

	//sprawdz czy ma ruszac sie jeszcze
	if (current_arm_coordinates[5] <= first_arm_coordinates[5] - 0.785)
			return false;

	current_arm_coordinates[5] -= 0.01;

	//przekaz rozkaz
	for (size_t i = 0; i < lib::MAX_SERVOS_NR; ++i)
	{
		the_robot->ecp_command.arm.pf_def.arm_coordinates[i] = current_arm_coordinates[i];
	}

	//the_robot->ecp_command.arm.pf_def.arm_frame = nextFrame;
	//currentFrame = nextFrame;

	//if (k > k_max)
	//	return false;


	return true;
} // next_step()

void g_rotate::log(const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);

	if (!logEnabled) {
		va_end(ap);
		return;
	}

	vfprintf(stdout, fmt, ap);
	fflush( stdout);
	va_end(ap);
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
