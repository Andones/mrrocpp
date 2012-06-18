#include <iostream>
#include <exception>

#include "edp_typedefs.h"
#include "edp_e_manip.h"
#include "base/lib/mis_fun.h"
#include "reader.h"
#include "base/kinematics/kinematic_model_with_tool.h"
#include "edp_imu_sensor.h"

namespace mrrocpp {
namespace edp {
namespace sensor {

void imu::operator()()
{
	if (!master.robot_test_mode) {
		lib::set_thread_priority(lib::PTHREAD_MAX_PRIORITY - 1);
	}

	try {
		if (!imu_sensor_test_mode) {
			connect_to_hardware();
		}

		thread_started.command();

		configure_sensor();
	}

	catch (lib::exception::se_sensor & error) {
		std::cerr << "sensor_error w force thread EDP" << std::endl;

		uint64_t error0 = 0;

		if (uint64_t const * tmp = boost::get_error_info <mrrocpp_error0>(error)) {
			error0 = *tmp;
		}

		switch (error0)
		{
			case SENSOR_NOT_CONFIGURED:
				//		from_vsp.vsp_report = lib::sensor::VSP_SENSOR_NOT_CONFIGURED;
				break;
			case READING_NOT_READY:
				//		from_vsp.vsp_report = lib::sensor::VSP_READING_NOT_READY;
				break;
		}
		sr_msg->message(lib::FATAL_ERROR, error0);

	}

	catch (std::exception & e) {
		printf("force sensor exception: %s\n", e.what());
		sr_msg->message(lib::FATAL_ERROR, e.what());
		exit(EXIT_SUCCESS);
	}

	catch (...) {
		std::cerr << "unidentified error force thread w EDP" << std::endl;
	}

	while (!boost::this_thread::interruption_requested()) {
		wait_for_event();
		get_reading();
		//	sr_msg->message("imu operator() in while");
	}
	sr_msg->message("imu operator() interruption_requested");
}

imu::imu(common::manip_effector &_master) :
		imu_sensor_test_mode(true), master(_master)
{
	sr_msg =
			boost::shared_ptr <lib::sr_vsp>(new lib::sr_vsp(lib::EDP, "i_" + master.config.robot_name, master.config.get_sr_attach_point()));

	sr_msg->message("imu constructor");

	if (master.config.exists(lib::IMU_SENSOR_TEST_MODE)) {
		imu_sensor_test_mode = master.config.exists_and_true(lib::IMU_SENSOR_TEST_MODE);
	}

	if (imu_sensor_test_mode) {
		sr_msg->message("IMU sensor test mode activated");
	}

}

imu::~imu()
{
	sr_msg->message("~imu destructor");
}

void imu::wait_for_event()
{
	if (!imu_sensor_test_mode) {

		wait_for_particular_event();

	} else {
		usleep(1000);
	}
}

/***************************** odczyt z czujnika *****************************/
void imu::get_reading(void)
{
	if (!imu_sensor_test_mode) {
		get_particular_reading();

	} else {

	}

}

} // namespace sensor
} // namespace edp
} // namespace mrrocpp
