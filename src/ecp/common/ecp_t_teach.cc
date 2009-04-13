// ------------------------------------------------------------------------
//
//                     EFFECTOR CONTROL PROCESS (ECP) - main()
//
// Ostatnia modyfikacja: 2006
// ------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/srlib.h"
#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_postument/ecp_local.h"
#include "ecp/irp6_mechatronika/ecp_local.h"

#include "ecp/common/ecp_t_teach.h"
#include "ecp/common/ecp_teach_in_generator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

// KONSTRUKTORY
teach::teach(lib::configurator &_config) : base(_config)
{
    tig = NULL;
}

// methods for ECP template to redefine in concrete classes
void teach::task_initialization(void)
{
    if (strcmp(config.section_name, "[ecp_irp6_on_track]") == 0)
    {
        ecp_m_robot = new irp6ot::ecp_irp6_on_track_robot (*this);
    }
    else if (strcmp(config.section_name, "[ecp_irp6_postument]") == 0)
    {
        ecp_m_robot = new irp6p::ecp_irp6_postument_robot (*this);
    }
    else if (strcmp(config.section_name, "[ecp_irp6_mechatronika]") == 0)
    {
        ecp_m_robot = new irp6m::ecp_irp6_mechatronika_robot (*this);
    }

    tig = new ecp_teach_in_generator (*this);

    sr_ecp_msg->message("ECP loaded");
};


void teach::main_task_algorithm(void)
{
    switch (ecp_m_robot->robot_name)
    {
    case ROBOT_IRP6_ON_TRACK:
        sr_ecp_msg->message("ECP teach irp6ot");
        break;
    case ROBOT_IRP6_POSTUMENT:
        sr_ecp_msg->message("ECP teach irp6p");
        break;
    default:
        fprintf(stderr, "%s:%d unknown robot type\n", __FILE__, __LINE__);
    }

    if ( operator_reaction ("Teach in? ") )
    {
        tig->flush_pose_list(); // Usuniecie listy pozycji, o ile istnieje
        tig->teach (MOTOR, "Teach-in the trajectory\n");
    }

    if ( operator_reaction ("Save trajectory? ") )
    {
        tig->save_file (MOTOR);
    }

    if ( operator_reaction ("Load trajectory? ") )
    {
        tig->load_file_from_ui ();
    }

    // Aktualnie petla wykonuje sie jednokrotnie, gdyby MP przejal sterowanie
    // to petle mozna przerwac przez STOP lub przez polecenie END_MOTION wydane
    // przez MP
    //  printf("w ecp for\n");
    tig->Move();
    // 	 printf("w ecp for za move\n");
    // Oczekiwanie na STOP
    ecp_termination_notice();
}

base* return_created_ecp_task (lib::configurator &_config)
{
	return new teach(_config);
}
} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

