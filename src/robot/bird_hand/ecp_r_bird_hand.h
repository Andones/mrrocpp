#if !defined(_ECP_R_BIRD_HAND_H)
#define _ECP_R_BIRD_HAND_H

/*!
 * @file
 * @brief File contains ecp robot class declaration for Bird Hand three finger gripper
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup bird_hand
 */

#include "base/ecp/ecp_robot.h"
#include "robot/bird_hand/const_bird_hand.h"
#include "base/kinematics/kinematics_manager.h"
#include "robot/bird_hand/kinematic_model_bird_hand.h"

namespace mrrocpp {
namespace ecp {
namespace bird_hand {

// ---------------------------------------------------------------
class robot : public common::robot::ecp_robot, public kinematics::common::kinematics_manager
{
protected:

	// zadawanie rozkazu
	lib::single_thread_port <lib::bird_hand_command> bird_hand_command_data_port;
	lib::bird_hand_command bird_hand_command_structure;

	// zadawanie parametrow konfiguracji
	lib::single_thread_port <lib::bird_hand_configuration> bird_hand_configuration_command_data_port;
	lib::bird_hand_configuration bird_hand_configuration_command_structure;

	// odbieranie statusu robota
	lib::single_thread_request_port <lib::bird_hand_status> bird_hand_status_reply_data_request_port;
	lib::bird_hand_status bird_hand_status_reply_structure;

	// odczytanie parametrow konfiguracji
	lib::single_thread_request_port <lib::bird_hand_configuration> bird_hand_configuration_reply_data_request_port;
	lib::bird_hand_configuration bird_hand_configuration_reply_structure;

	// bufory do edp
	lib::bird_hand_cbuffer ecp_edp_cbuffer;
	lib::bird_hand_rbuffer edp_ecp_rbuffer;

	void create_kinematic_models_for_given_robot(void);

public:
	robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp);
	robot(common::task::task& _ecp_object);

	void create_command();
	void get_reply();

};
// ---------------------------------------------------------------

} // namespace bird_hand
} // namespace ecp
} // namespace mrrocpp

#endif
