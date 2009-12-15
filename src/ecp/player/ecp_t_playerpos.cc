#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "ecp/player/ecp_g_playerpos.h"
#include "ecp/player/ecp_t_playerpos.h"
#include "ecp_mp/task/ecp_mp_t_player.h"

namespace mrrocpp {
namespace ecp {
namespace player {
namespace task {

// KONSTRUKTORY
playerpos::playerpos(lib::configurator &_config) :
	task(_config)
{
	ppg = new generator::playerpos (*this);
}

playerpos::~playerpos()
{
	delete ppg;
}

void playerpos::main_task_algorithm(void)
{
	for (;;) {
		sr_ecp_msg->message("Waiting for MP order");

		get_next_state();

		sr_ecp_msg->message("Order received");

		switch ( (ecp_mp::task::ECP_PLAYER_STATES) mp_command.ecp_next_state.mp_2_ecp_next_state) {
			case ecp_mp::task::ECP_GEN_PLAYERPOS:
				ppg->set_goal(mp_command.ecp_next_state.playerpos_goal);
				ppg->Move();
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

task* return_created_ecp_task (lib::configurator &_config)
{
	return new player::task::playerpos(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


