/*
 * kinematic_parameters_spkm2.h
 *
 *  Created on: 23-12-2011
 *      Author: tkornuta
 */

#ifndef KINEMATIC_PARAMETERS_SPKM1_H_
#define KINEMATIC_PARAMETERS_SPKM1_H_

#include "kinematic_parameters_spkm.h"

namespace mrrocpp {
namespace kinematics {
namespace spkm2 {

class kinematic_parameters_spkm2 : public mrrocpp::kinematics::spkm::kinematic_parameters_spkm
{
public:
	//! Constructor - sets the values of the SPKM geometric parameters.
	kinematic_parameters_spkm2();
};

}
}
}

#endif /* KINEMATIC_PARAMETERS_SPKM1_H_ */
