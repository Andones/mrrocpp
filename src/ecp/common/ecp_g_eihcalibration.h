/*
 * ecp_g_eihcalibration.h
 *
 *  Created on: July 28, 2009
 *      Author: jkosiore
 */

#ifndef ECP_G_EIHCALIBRATION_H_
#define ECP_G_EIHCALIBRATION_H_

//fradia
#include "ecp_mp/ecp_mp_s_cvfradia.h"
#include "ecp/common/ecp_t_cvfradia.h"

#include "ecp/common/ecp_generator.h"

#include <cstring>
#include <iostream>
#include <unistd.h>

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

class eihgenerator : public common::generator::generator
{
	ecp_mp::sensor::cvfradia * sensor;

  public:
	int count;
	double tab[12];
	eihgenerator(common::task::task& _ecp_task);
	~eihgenerator();
	bool first_step();
	bool next_step();

	void get_frame();
};

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* ECP_G_EIHCALIBRATION_H_ */

