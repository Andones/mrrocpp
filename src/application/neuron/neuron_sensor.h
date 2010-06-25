/*
 * neuron_sensor.h
 *
 *  Created on: Jun 23, 2010
 *      Author: tbem
 */

#ifndef NEURON_SENSOR_H_
#define NEURON_SENSOR_H_


#include "base/ecp_mp/ecp_mp_sensor.h"

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

/*
 * Class responsible for representing neuron vsp in mrrocpp that in fact handles communication between
 * mrrocpp and external vsp
 */
class neuron_sensor : public ecp_mp::sensor::sensor_interface {
private:
	int a;

public:
	neuron_sensor();
	virtual ~neuron_sensor();
	void get_reading();
	void configure_sensor();
	void initiate_reading();
};

} //sensor
} //ecp_mp
} //mrrocpp
#endif /* NEURON_SENSOR_H_ */
