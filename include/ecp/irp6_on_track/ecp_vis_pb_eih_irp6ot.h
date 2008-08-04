///////////////////////////////////////////////////////////
//  ecp_vis_pb_eih_irp6ot.h
//  Implementation of the Class ecp_vis_pb_eih_irp6ot
//  Created on:      04-sie-2008 14:26:04
//  Original author: tkornuta
///////////////////////////////////////////////////////////

#if !defined(EA_F2A497C9_34AF_4480_B49D_A41B7D6E362F__INCLUDED_)
#define EA_F2A497C9_34AF_4480_B49D_A41B7D6E362F__INCLUDED_

#include "ecp_visual_servo.h"

class ecp_vis_pb_eih_irp6ot : public ecp_visual_servo
{

public:
	ecp_vis_pb_eih_irp6ot();
	virtual ~ecp_vis_pb_eih_irp6ot();

	virtual void next_step_without_constraints()();
	virtual void entertain_constraints();
	virtual bool first_step(void);

};
#endif // !defined(EA_F2A497C9_34AF_4480_B49D_A41B7D6E362F__INCLUDED_)
