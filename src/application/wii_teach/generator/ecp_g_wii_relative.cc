#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "math.h"
#include "application/wii_teach/generator/ecp_g_wii_relative.h"
#include "ecp_g_wii.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace generator {

wii_relative::wii_relative (common::task::task& _ecp_task,ecp_mp::sensor::wiimote* _wiimote) : wii(_ecp_task,_wiimote)
{
    int i;
    for(i = 0;i<7;++i)
    {
        multipliers[i] = 0.003;
        maxChange[i] = 0.0001;
    }
}

void wii_relative::set_position(void)
{
    the_robot->ecp_command.instruction.instruction_type = lib::SET;
    the_robot->ecp_command.instruction.set_type = ARM_DV;
    the_robot->ecp_command.instruction.set_arm_type = lib::FRAME;
    the_robot->ecp_command.instruction.motion_type = lib::RELATIVE;
    the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
    the_robot->ecp_command.instruction.motion_steps = 8;
    the_robot->ecp_command.instruction.value_in_step_no = 8;

    homog_matrix.set_xyz_angle_axis(
        currentValue[0] + nextChange[0],
        currentValue[1] + nextChange[1],
        currentValue[2] + nextChange[2],
        currentValue[3] + nextChange[3],
        currentValue[4] + nextChange[4],
        currentValue[5] + nextChange[5]
    );
    homog_matrix.get_frame_tab(the_robot->ecp_command.instruction.arm.pf_def.arm_frame);

    the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = currentValue[6] + nextChange[6];
}


bool wii_relative::first_step()
{
    the_robot->ecp_command.instruction.instruction_type = lib::GET;
    the_robot->ecp_command.instruction.get_type = ARM_DV;
    the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
    the_robot->ecp_command.instruction.motion_type = lib::RELATIVE;
    the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
    the_robot->ecp_command.instruction.motion_steps = 8;
    the_robot->ecp_command.instruction.value_in_step_no = 8;

    releasedA = false;
    stop = false;

    return true;
}

void wii_relative::preset_position(void)
{
    homog_matrix.set_frame_tab(the_robot->ecp_command.instruction.arm.pf_def.arm_frame);
    homog_matrix.get_xyz_angle_axis(currentValue);

    currentGripperValue = the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate;
}


}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp
