/*
 * $Id$
 *
 *  Created on: Mar 3, 2010
 *      Author: mboryn
 */

#include "visual_servo.h"

using namespace logger;

namespace mrrocpp {
namespace ecp {
namespace servovision {

visual_servo::visual_servo(boost::shared_ptr <visual_servo_regulator> regulator) :
	regulator(regulator), object_visible(false), max_steps_without_reading(5), steps_without_reading(0)
{

}

visual_servo::~visual_servo()
{
}

lib::Homog_matrix visual_servo::get_position_change(const lib::Homog_matrix& current_position, double dt)
{
	lib::Homog_matrix delta_position;

	if (get_sensor_report() == lib::VSP_SENSOR_NOT_CONFIGURED) { // sensor not yet ready
		return delta_position;
	} else if (get_sensor_report() == lib::VSP_READING_NOT_READY) { // maybe there was a reading
		if (steps_without_reading > max_steps_without_reading) { // but if it was too long ago
			object_visible = false; // we have to consider object not longer visible
			log_dbg("pb_eih_visual_servo::get_position_change(): object considered no longer visible\n");
			return delta_position;
		} else {
			steps_without_reading++;
		}
	} else if (get_sensor_report() == lib::VSP_REPLY_OK) { // we have a reading
		steps_without_reading = 0; // reset counter
	}

	if (is_object_visible()) {
		return compute_position_change(current_position, dt);
	}

	return delta_position;
}

} // namespace servovision
} // namespace ecp
} // namespace mrrocpp
