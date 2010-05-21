// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			edp_irp6ot_effector.h
// Opis:		Robot IRp-6 na na torze jezdnym
//				- deklaracja klasy edp_irp6ot_effector
//
// Autor:		tkornuta
// Data:		17.01.2007
// ------------------------------------------------------------------------


#ifndef __EDP_IRP6_ON_TRACK_H
#define __EDP_IRP6_ON_TRACK_H

#include "edp/irp6_on_track/sg_irp6ot.h"
#include "edp/common/edp_irp6s_postument_track.h"
#include "lib/robot_consts/irp6ot_const.h"

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
    common::servo_buffer *return_created_servo_buffer ();
};

} // namespace irp6ot
} // namespace edp
} // namespace mrrocpp



#endif
