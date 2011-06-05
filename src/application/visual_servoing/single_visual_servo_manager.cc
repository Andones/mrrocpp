/*
 * single_visual_servo_manager.cc
 *
 *  Created on: Apr 21, 2010
 *      Author: mboryn
 */

#include <stdexcept>

#include "single_visual_servo_manager.h"

#include "base/lib/logger.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

using namespace logger;
using namespace std;

const double single_visual_servo_manager::image_sampling_period_default = 0.040;

single_visual_servo_manager::single_visual_servo_manager(
		mrrocpp::ecp::common::task::task & ecp_task, const char * section_name, boost::shared_ptr<
				mrrocpp::ecp::servovision::visual_servo> vs) :
	visual_servo_manager(ecp_task, section_name)
{
	servos.push_back(vs);

	image_sampling_period
			= ecp_task.config.exists("image_sampling_period", section_name) ? ecp_task.config.value<
					double> ("image_sampling_period", section_name)
					: image_sampling_period_default;

	txtiter = 0;
	txtbuf.reserve(50000);
}

single_visual_servo_manager::~single_visual_servo_manager()
{
}

lib::Homog_matrix single_visual_servo_manager::get_aggregated_position_change()
{
	bool reading_received = false;
	if (servos[0]->get_sensor()->get_state()
			== ecp_mp::sensor::discode::discode_sensor::DSS_READING_RECEIVED) {
		reading_received = true;
	}

	lib::Homog_matrix pc = servos[0]->get_position_change(get_current_position(), get_dt());

	if(reading_received){
		update_motion_steps();
	}

	return pc;
}

void single_visual_servo_manager::configure_all_servos()
{
	motion_steps_base = get_motion_steps();
	log_dbg("single_visual_servo_manager::configure_all_servos(): motion_steps_base = %d\n",
			motion_steps_base);
}

void single_visual_servo_manager::update_motion_steps()
{
	ecp_mp::sensor::discode::reading_message_header rmh = servos[0]->get_sensor()->get_rmh();
	Types::Mrrocpp_Proxy::Reading* reading = servos[0]->get_reading();

	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);

	int seconds = ts.tv_sec - reading->processingStartSeconds;
	int nanoseconds = ts.tv_nsec - reading->processingStartNanoseconds;
	double image_mrroc_delay = seconds + 1e-9 * nanoseconds;
	image_mrroc_delay -= servos[0]->get_sensor()->get_mrroc_discode_time_offset();

	char txt[1000];

//	sprintf(txt, "mrroc_discode_time_offset = %g\n", mrroc_discode_time_offset);
//	txtbuf += txt;
//
//	sprintf(txt, "rmh.sendTimeSeconds = %d    rmh.sendTimeNanoseconds = %d    \n",
//			rmh.sendTimeSeconds, rmh.sendTimeNanoseconds);
//	txtbuf += txt;
//
//	sprintf(
//			txt,
//			"reading->processingStartSeconds = %d    reading->processingStartNanoseconds = %d    \n",
//			reading->processingStartSeconds, reading->processingStartNanoseconds);
//	txtbuf += txt;
//
//	sprintf(txt,
//			"reading->processingEndSeconds = %d    reading->processingEndNanoseconds = %d    \n",
//			reading->processingEndSeconds, reading->processingEndNanoseconds);
//	txtbuf += txt;

	sprintf(txt, "ts.tv_sec = %ld    ts.tv_nsec = %ld    \n", ts.tv_sec, ts.tv_nsec);
	txtbuf += txt;

	double offset = fmod(image_mrroc_delay, image_sampling_period);

	if (offset > image_sampling_period / 2) {
		offset -= image_sampling_period;
	}

	double offset_threshold = image_sampling_period / 20;

	int ms;

	if (offset > offset_threshold) {
		ms = motion_steps_base - 1;
	} else if (offset < -offset_threshold) {
		ms = motion_steps_base + 1;
	} else {
		ms = motion_steps_base;
	}

	set_new_motion_steps(ms);

	sprintf(txt, "e = %g          image_mrroc_delay = %g     ms = %d\n", offset, image_mrroc_delay, ms);
	txtbuf += txt;

	txtiter++;
	if (txtiter > 100) {
		log("\n\n\nHEHEHEHE:\n%s\n\n", txtbuf.c_str());
		throw runtime_error("HEHEHEEHEH");
	}
}

}//namespace generator

}

}

}
