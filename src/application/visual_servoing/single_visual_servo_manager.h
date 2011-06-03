/*
 * single_visual_servo_manager.h
 *
 *  Created on: Apr 21, 2010
 *      Author: mboryn
 */

#ifndef SINGLE_VISUAL_SERVO_MANAGER_H_
#define SINGLE_VISUAL_SERVO_MANAGER_H_

#include "visual_servo_manager.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

/** @addtogroup servovision
 *  @{
 */

/**
 *
 */
class single_visual_servo_manager : public visual_servo_manager
{
public:
			single_visual_servo_manager(mrrocpp::ecp::common::task::task & ecp_task, const char * section_name, boost::shared_ptr <
					mrrocpp::ecp::servovision::visual_servo> vs);
	virtual ~single_visual_servo_manager();
protected:
	virtual lib::Homog_matrix get_aggregated_position_change();
	virtual void configure_all_servos();

private:
	void update_motion_steps(double discode_processing_time, double discode_synchronization_delay, double discode_total_time, double image_mrroc_delay);

	double image_sampling_period;
	static const double image_sampling_period_default;
	int motion_steps_base;


	std::string txtbuf;
	int txtiter;
};

/** @} */

}//namespace generator

}

}

}

#endif /* SINGLE_VISUAL_SERVO_MANAGER_H_ */
