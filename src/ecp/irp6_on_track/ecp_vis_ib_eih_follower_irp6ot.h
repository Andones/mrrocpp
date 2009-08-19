/*
 * ecp_vis_ib_eih_follower_irp6ot.h
 *
 *  Created on: Dec 9, 2008
 *      Author: rtulwin
 */

#ifndef ECP_VIS_IB_EIH_FOLLOWER_IRP6OT_H_
#define ECP_VIS_IB_EIH_FOLLOWER_IRP6OT_H_

#include "ecp/common/ecp_visual_servo.h"

#include <string.h>
#include <math.h>

#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "ecp_mp/ecp_mp_s_cvfradia.h"
#include "ecp/common/ecp_t_cvfradia.h"

#include <iostream>

namespace mrrocpp {
namespace ecp {
namespace irp6ot {

#define MOTION_STEPS 10


class ecp_vis_ib_eih_follower_irp6ot: public common::ecp_visual_servo {
	//bool check_if_followed();
public:
	//Wirtualny sensor
	lib::sensor *vsp_fradia;
	//Pozycja w nastepnym kroku.
    double next_position[8];
    //Obliczone sterowanie dla osi x;
    /*double x;
    //Obliczone sterowanie dla osi y;
    double y;
    //Przyrost polozenia w makrokroku
    double s;*/

    //Orientacja koncokwi wzgledem ukladu bazowego.
    double alpha;

	/*//Maksymalna wartosc  predkosci.
	double v_max;
	//Wartosc przyspieszenia z jakim osiagana jest maksymalna predkosc.
	double  a;
	//Minimalna  wartosc predkosci do jakiej schodzimy przy hamowaniu.
	double v_min;
	//Predkosc chwilowa.
	double v;
	//Dystans wyrazony w pikselach, przy ktorym nastepuje hamowanie.
	double breaking_dist;
	//Czas makrokroku.
	double t_m;
	//Droga do przejechania w nastepnym makrokroku.

	double old_frame_no;*/
	bool first_move;
	/*bool breaking;
	int holes;
	bool above_object;*/

	ecp_vis_ib_eih_follower_irp6ot(common::task::task& _ecp_task);
	virtual ~ecp_vis_ib_eih_follower_irp6ot();
	virtual bool first_step(void);
	virtual bool next_step_without_constraints();
	virtual void entertain_constraints();
	//virtual void retrieve_parameters();
};

} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif /* ECP_VIS_IB_EIH_FOLLOWER_IRP6OT_H_ */


