// ------------------------------------------------------------------------
//   ecp_t_tran.cc - przezroczyste wersja dla dowolnego z robotow
//
//                     EFFECTOR CONTROL PROCESS (ECP) - main()
//
// Ostatnia modyfikacja: 2006
// ------------------------------------------------------------------------

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp/speaker/ecp_local.h"
#include "ecp/speaker/ecp_t_rcsc_speaker.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"

namespace mrrocpp {
namespace ecp {
namespace common {

// KONSTRUKTORY
ecp_task_rcsc_speaker::ecp_task_rcsc_speaker(configurator &_config) : ecp_task(_config)
{
    gt = NULL;
    speak = NULL;
}

// methods for ECP template to redefine in concrete classes
void ecp_task_rcsc_speaker::task_initialization(void)
{
    ecp_m_robot = new ecp_speaker_robot (*this);

    gt = new ecp_generator_t(*this);
    speak = new speaking_generator (*this, 8);

    sr_ecp_msg->message("ECP loaded");
}


void ecp_task_rcsc_speaker::main_task_algorithm(void)
{
	for(;;)
	{

		sr_ecp_msg->message("Waiting for MP order");

		get_next_state ();

		sr_ecp_msg->message("Order received");

		switch ( (ecp_mp::task::RCSC_ECP_STATES) mp_command.ecp_next_state.mp_2_ecp_next_state)
		{
			case ecp_mp::task::ECP_GEN_TRANSPARENT:
				gt->Move();
				break;
			case ecp_mp::task::ECP_GEN_SPEAK:
				speak->configure(mp_command.ecp_next_state.mp_2_ecp_next_state_string);
				speak->Move();
				break;
			default:
				break;
		}
	}
}

ecp_task* return_created_ecp_task (configurator &_config)
{
	return new ecp_task_rcsc_speaker(_config);
}

} // namespace common
} // namespace ecp
} // namespace mrrocpp

