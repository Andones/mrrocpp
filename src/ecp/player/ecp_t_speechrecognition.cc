#include <stdio.h>
#include <string.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "ecp/player/ecp_g_speechrecognition.h"
#include "ecp/player/ecp_t_speechrecognition.h"
#include "ecp_mp/ecp_mp_t_player.h"

namespace mrrocpp {
namespace ecp {
namespace player {
namespace task {

// KONSTRUKTORY
speechrecognition::speechrecognition(configurator &_config)
        : base(_config)
{
    srg = new generator::speechrecognition (*this);
}

speechrecognition::~speechrecognition()
{
    delete srg;
}

// methods for ECP template to redefine in concrete classes
void speechrecognition::task_initialization(void)
{
    sr_ecp_msg->message("ECP loaded");
}

void speechrecognition::main_task_algorithm(void)
{
    for(;;)
    {
        sr_ecp_msg->message("Waiting for MP order");

        get_next_state ();

        sr_ecp_msg->message("Order received");

        switch ( (ecp_mp::task::ECP_PLAYER_STATES) mp_command.ecp_next_state.mp_2_ecp_next_state)
        {
        	case ecp_mp::task::ECP_GEN_SPEECHRECOGNITION:
        		srg->Move();
        		break;
        	default:
        		fprintf(stderr, "invalid ecp_next_state.mp_2_ecp_next_state (%d)\n", mp_command.ecp_next_state.mp_2_ecp_next_state);
        		break;
        }

        ecp_termination_notice();
    }
}

}
} // namespace player

namespace common {
namespace task {

base* return_created_ecp_task (configurator &_config)
{
	return new player::task::speechrecognition(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


