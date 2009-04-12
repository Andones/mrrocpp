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
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/common/ecp_g_jarosz.h"
#include "ecp/irp6_on_track/ecp_t_fr_irp6ot.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {


// KONSTRUKTORY
ecp_task_fr_irp6ot::ecp_task_fr_irp6ot(configurator &_config) : ecp_task(_config)
{
    adg1 = NULL;
    adg2 = NULL;
    el = NULL;
}

// methods for ECP template to redefine in concrete classes
void ecp_task_fr_irp6ot::task_initialization(void)
{
    ecp_m_robot = new ecp_irp6_on_track_robot (*this);

    tdes_joint.arm_type = JOINT;
    tdes_joint.interpolation_node_no =200;
    tdes_joint.internode_step_no = 10;
    tdes_joint.value_in_step_no = tdes_joint.internode_step_no -2;
    tdes_joint.coordinate_delta[0] = 1.28;
    tdes_joint.coordinate_delta[1] = 2.880;
    tdes_joint.coordinate_delta[2] = -1.365;
    tdes_joint.coordinate_delta[3] = 0.38;
    tdes_joint.coordinate_delta[4] = 0.0;
    tdes_joint.coordinate_delta[5] = 0.0;
    tdes_joint.coordinate_delta[6] = 0.0;
    tdes_joint.coordinate_delta[7] = 0.0;


    trajectory_description tdes_ext;
    tdes_ext.arm_type = XYZ_EULER_ZYZ;
    tdes_ext.interpolation_node_no =200;
    tdes_ext.internode_step_no = 10;
    tdes_ext.value_in_step_no = tdes_ext.internode_step_no -2;
    tdes_ext.coordinate_delta[0] = -0.69825;
    tdes_ext.coordinate_delta[1] = 1.47171;
    tdes_ext.coordinate_delta[2] = 1;
    tdes_ext.coordinate_delta[3] = 2.87979;
    tdes_ext.coordinate_delta[4] = 1.57080;
    tdes_ext.coordinate_delta[5] = -3.14159;
    tdes_ext.coordinate_delta[6] = 0.074;

    for (int i = 0; i < MAX_SERVOS_NR; i++)
    {
        ta[i] = 0.3;
        tb[i] = 0.9;
    }

    // parabolic_generator adg2(XYZ_EULER_ZYZ, 10., ext_pp);  // generator dla trajektorii dojscia we wsp. zew.
    adg1 = new common::ecp_linear_parabolic_generator (*this, tdes_joint, ta, tb);
    // parabolic_generator adg1(JOINT, 20., joint_pp);   // generator dla trajektorii dojscia we wsp. wew
    // generator dla trajektorii dojscia we wsp. zew.
    adg2 = new common::ecp_linear_parabolic_generator (*this, tdes_ext, ta, tb);
    el = new common::ecp_elipsoid_generator (*this);


    sr_ecp_msg->message("ECP loaded");
}


void ecp_task_fr_irp6ot::main_task_algorithm(void)
{
	if (operator_reaction("Start motion? ")) {
		// Odtwarzanie nauczonej lub wczytanej trajektorii zaczynamy od jej poczatku
		// Trajektoria podejscia
		adg1->Move(); // przejscie do nastepnej pozycji
		if (operator_reaction("Start external motion? "))
			adg2->Move();

		if (operator_reaction("Start elipsoid trajectory? ")) // Oczekiwanie na zezwolnie na frezowanie
			el->Move();
		if (operator_reaction("Save trajectory?"))
			ecp_save_trajectory(*el, *this);
		while (!operator_reaction("End motion? ")); // Oczekiwanie na zezwolnie na frezowanie
	}

	// Informacja dla MP o zakonczeniu zadania uzytkownika
	ecp_termination_notice();
}

} // namespace irp6ot

namespace common {
namespace task {

ecp_task* return_created_ecp_task (configurator &_config)
{
	return new irp6ot::ecp_task_fr_irp6ot(_config);
}
}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

