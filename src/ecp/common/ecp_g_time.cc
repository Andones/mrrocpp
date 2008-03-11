// -------------------------------------------------------------------------
//                            ecp_gen_time.cc
//            Effector Control Process (ECP) - rysowanie
// 			Funkcje do tworzenia procesow ECP z rysowaniem
// 			Ostatnia modyfikacja: 01.06.2006r.
// -------------------------------------------------------------------------

#include <stdio.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp/common/ecp_g_time.h"

#include "lib/mathtr.h"

time_generator::time_generator(ecp_task& _ecp_task, int step):
        ecp_generator (_ecp_task)
{
    step_no = step;
}


bool time_generator::first_step ( )
{
    run_counter = 0;
    second_step = false;

    td.interpolation_node_no = 1;
    td.internode_step_no = step_no;
    td.value_in_step_no = td.internode_step_no - 2;


    the_robot->EDP_data.instruction_type = GET;
    the_robot->EDP_data.get_type = ARM_DV; // arm - ORYGINAL
    the_robot->EDP_data.set_type = ARM_DV;

    the_robot->EDP_data.set_arm_type = JOINT;
    the_robot->EDP_data.get_arm_type = JOINT;

    the_robot->EDP_data.motion_type = ABSOLUTE;
     the_robot->EDP_data.interpolation_type = MOTORS_INTERPOLATION;
    the_robot->EDP_data.motion_steps = td.internode_step_no;
    the_robot->EDP_data.value_in_step_no = td.value_in_step_no;




    return true;
}
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
bool time_generator::next_step ( )
{
    // zmienne wykorzystywane przy rysowaniu
    if (check_and_null_trigger())
    {
        return false;
    }

    // Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
    the_robot->EDP_data.instruction_type = SET;
    the_robot->EDP_data.set_type = ARM_DV;
    the_robot->EDP_data.get_type = NOTHING_DV;
    the_robot->EDP_data.get_arm_type = INVALID_END_EFFECTOR;
    

    //transmitter_m[TRANSMITTER_PLAYER]->t_read(0);

#if 1

    printf("%lu.%09lu\n",
           sensor_m[SENSOR_TIME]->image.time.ts.tv_sec,
           sensor_m[SENSOR_TIME]->image.time.ts.tv_nsec
          );
#endif

    if (node_counter <= 2)
    {
        for (int i = 0; i < 8; i++)
            start_joint_arm_coordinates[i] = the_robot->EDP_data.current_joint_arm_coordinates[i];
    }
    else
    {
        for (int i = 0; i < 8; i++)
        {
            //start_joint_arm_coordinates[i] +=
        }
    }

    for (int i = 0; i < 8; i++)
    {
        the_robot->EDP_data.next_joint_arm_coordinates[i] = start_joint_arm_coordinates[i];
    }

    //printf("\n");



    return true;
}
