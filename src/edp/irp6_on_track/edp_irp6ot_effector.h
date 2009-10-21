// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			edp_irp6ot_effector.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		Robot IRp-6 na na torze jezdnym
//				- deklaracja klasy edp_irp6ot_effector
//
// Autor:		tkornuta
// Data:		17.01.2007
// ------------------------------------------------------------------------


#ifndef __EDP_IRP6_ON_TRACK_H
#define __EDP_IRP6_ON_TRACK_H

// Klasa edp_irp6s_robot.
#include "edp/common/edp_irp6s_postument_track.h"

#define IRP6OT_GRIPPER_CATCH_AXE 7
#define IRP6OT_GRIPPER_TURN_AXE 6

namespace mrrocpp {
namespace edp {
namespace irp6ot {

// Klasa reprezentujaca robota IRp-6 na torze jezdnym.
class effector : public common::irp6s_postument_track_effector
{
protected:
    // Metoda tworzy modele kinematyczne dla robota IRp-6 na postumencie.
    virtual void create_kinematic_models_for_given_robot(void);

public:
    effector (lib::configurator &_config);
};

} // namespace irp6ot
} // namespace edp
} // namespace mrrocpp



#endif
