/*
 * pb_eih_visual_servo.cc
 *
 *  Created on: May 11, 2010
 *      Author: mboryn
 */

#include "pb_eih_visual_servo.h"

using ecp_mp::sensor::fradia_sensor;
using namespace logger;
using namespace visual_servo_types;
using namespace std;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

pb_eih_visual_servo::pb_eih_visual_servo(/*boost::shared_ptr <visual_servo_regulator> regulator,*/const char* section_name, mrrocpp::lib::configurator& configurator) :
	visual_servo(regulator), object_visible(false)
{
	try {
		vsp_fradia
				= boost::shared_ptr <fradia_sensor <position_based_reading, position_based_configuration> >(new fradia_sensor <
						position_based_reading, position_based_configuration> (configurator, section_name));

		position_based_configuration pb_config;

		Eigen::Matrix <double, 3, 3> intrinsics = configurator.value <3, 3> ("fradia_camera_intrinsics", section_name);

		Eigen::Matrix <double, 1, 5> distortion = configurator.value <1, 5> ("fradia_camera_distortion", section_name);

		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				pb_config.dcp.intrinsics[i][j] = intrinsics(i, j);
			}
		}
		for (int i = 0; i < 5; ++i) {
			pb_config.dcp.distortion[i] = distortion(0, i);
		}

		vsp_fradia->configure_fradia_task(pb_config);

		Eigen::Matrix <double, 3, 3> e_T_c;
		e_T_c = configurator.value <3, 3> ("e_t_c_rotation", section_name);

		double rot[3][3];

		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				rot[i][j] = e_T_c(i, j);
			}
		}

		e_T_c_position.set_rotation_matrix(rot);

		desiredTranslation = configurator.value <3, 1> ("desired_translation", section_name);
	} catch (const exception& e) {
		printf("pb_eih_visual_servo::pb_eih_visual_servo(): %s\n", e.what());
		throw e;
	}

}

pb_eih_visual_servo::~pb_eih_visual_servo()
{
}

lib::Homog_matrix pb_eih_visual_servo::get_position_change(const lib::Homog_matrix& current_position)
{
	lib::Homog_matrix delta_position;

	object_visible = vsp_fradia->received_object.tracking;
	if (vsp_fradia->received_object.tracking) {
		delta_position.set_translation_vector(vsp_fradia->received_object.position.translation);
		for (int i = 0; i < 3; ++i) {
			logDbg("error: %8lg, %8lg, %8lg\n", delta_position[0][3], delta_position[1][3], delta_position[2][3]);
			delta_position[i][3] -= desiredTranslation(i, 0);
		}
		delta_position = e_T_c_position * delta_position;
	}

	return delta_position;
}

boost::shared_ptr <mrrocpp::lib::sensor> pb_eih_visual_servo::get_vsp_fradia()
{
	return boost::dynamic_pointer_cast <mrrocpp::lib::sensor>(vsp_fradia);
}

bool pb_eih_visual_servo::is_object_visible()
{
	return object_visible;
}

} // namespace generator

}

}

}
