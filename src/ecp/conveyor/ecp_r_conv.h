// -------------------------------------------------------------------------
//                            ecp.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP
// robot - conveyor
//
// -------------------------------------------------------------------------

#if !defined(_ECP_R_CONVEYOR_H)
#define _ECP_R_CONVEYOR_H

#include "ecp/common/ecp_robot.h"

namespace mrrocpp {
namespace ecp {
namespace conveyor {

// ---------------------------------------------------------------
class robot: public common::ecp_robot
{
	// Klasa dla robota conveyor (sztywnego)

	public:
	robot (lib::configurator &_config, lib::sr_ecp *_sr_ecp);
	robot (common::task::task& _ecp_object);

	virtual void create_command (void);
	// wypelnia bufor wysylkowy do EDP na podstawie danych zawartych w obrazie
	// robota wykorzystywanych przez generator
	// Ten bufor znajduje sie w robocie

	void get_reply (void);
	virtual void get_input_reply (void);
	virtual void get_arm_reply (void);
	virtual void get_rmodel_reply (void);

}; // end: class ecp_conveyor_robot
// ---------------------------------------------------------------

} // namespace conveyor
} // namespace ecp
} // namespace mrrocpp

#endif
