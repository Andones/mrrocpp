// -------------------------------------------------------------------------
//                            ecp_local.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP
// robot - irp6_mechatronika
//
// -------------------------------------------------------------------------

#if !defined(_ECP_R_BIRD_HAND_H)
#define _ECP_R_BIRD_HAND_H

#include "ecp/common/ecp_robot.h"
#include "lib/robot_consts/bird_hand_const.h"
#include "kinematics/common/kinematics_manager.h"
#include "kinematics/bird_hand/kinematic_model_bird_hand.h"
#include "lib/data_port_headers/epos.h"

namespace mrrocpp {
namespace ecp {
namespace bird_hand {

// ---------------------------------------------------------------
class robot: public common::ecp_robot,
		public kinematics::common::kinematics_manager {
	// Klasa dla robota irp6_postument (sztywnego)
protected:
	//bufory wejsciowe z generatora
	lib::single_thread_port<lib::epos_low_level_command>
			epos_low_level_command_data_port;
	lib::epos_low_level_command epos_low_level_command_structure;

	lib::single_thread_port<lib::epos_gen_parameters>
			epos_gen_parameters_data_port;
	lib::epos_gen_parameters epos_gen_parameters_structure;

	// bufor wyjsciowe do generatora
	lib::single_thread_request_port<lib::epos_reply>
			epos_reply_data_request_port;
	lib::epos_reply epos_reply_structure;

	// bufory do edp
	lib::bird_hand_cbuffer ecp_edp_cbuffer;
	lib::bird_hand_rbuffer edp_ecp_rbuffer;

	void create_kinematic_models_for_given_robot(void);
	void add_data_ports();

public:
	robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp);
	robot(common::task::task& _ecp_object);

	void create_command();
	void get_reply();
	void clear_data_ports();

}; // end: class ecp_irp6_mechatronika_robot
// ---------------------------------------------------------------

} // namespace bird_hand
} // namespace ecp
} // namespace mrrocpp

#endif
