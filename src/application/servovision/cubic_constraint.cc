/*
 * cubic_constraint.cc
 *
 *  Created on: Apr 26, 2010
 *      Author: mboryn
 */

#include "cubic_constraint.h"
#include "lib/logger.h"

using namespace logger;
using namespace std;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

cubic_constraint::cubic_constraint(const Eigen::Matrix <double, 3, 1>& translation_min, const Eigen::Matrix <double, 3,
		1>& translation_max, const Eigen::Matrix <double, 3, 1>& rotation_min, const Eigen::Matrix <double, 3, 1>& rotation_max) :
	translation_min(translation_min), translation_max(translation_max), rotation_min(rotation_min),
			rotation_max(rotation_max)
{
	for (int i = 0; i < 3; ++i) {
		rotation_division(i, 0) = rotation_min(i, 0) + rotation_max(i, 0);
		if (rotation_min(i, 0) <= rotation_max(i, 0)) {
			if (rotation_division(i, 0) >= M_PI) {
				rotation_division(i, 0) -= M_PI;
			} else {
				rotation_division(i, 0) += M_PI;
			}
		}
	}

}

cubic_constraint::~cubic_constraint()
{
}

void cubic_constraint::apply_constraint()
{
	lib::Xyz_Angle_Axis_vector position_aa;
	new_position.get_xyz_angle_axis(position_aa);

	for (int i = 0; i < 3; ++i) {
		position_aa(i, 0) = min(translation_max(i, 0), position_aa(i, 0));
		position_aa(i, 0) = max(translation_min(i, 0), position_aa(i, 0));

		if (position_aa(i + 3, 0) < 0) {
			log("cubic_constraint::apply_constraint(): position_aa(%d, 0) = %g\n", i + 3, position_aa(i + 3, 0));
		}

		if (rotation_min(i, 0) < rotation_division(i, 0)) {
			if (position_aa(i + 3, 0) < rotation_min(i, 0) || position_aa(i + 3, 0) >= rotation_division(i, 0)) {
				position_aa(i + 3, 0) = rotation_min(i, 0);
			}
		} else {
			if (position_aa(i + 3, 0) < rotation_min(i, 0) && position_aa(i + 3, 0) >= rotation_division(i, 0)) {
				position_aa(i + 3, 0) = rotation_min(i, 0);
			}
		}

		if (rotation_max(i, 0) > rotation_division(i, 0)) {
			if (position_aa(i + 3, 0) > rotation_max(i, 0) || position_aa(i + 3, 0) < rotation_division(i, 0)) {
				position_aa(i + 3, 0) = rotation_max(i, 0);
			}
		} else {
			if (position_aa(i + 3, 0) > rotation_max(i, 0) && position_aa(i + 3, 0) < rotation_division(i, 0)) {
				position_aa(i + 3, 0) = rotation_max(i, 0);
			}
		}
	}
	new_position.set_from_xyz_angle_axis(position_aa);
}

bool cubic_constraint::is_translation_ok()
{
	double t[3];
	new_position.get_translation_vector(t);
	for (int i = 0; i < 3; ++i) {
		if (t[i] < translation_min(i, 0) || translation_max(i, 0) < t[i]) {
			return false;
		}
	}
	return true;
}

bool cubic_constraint::is_rotation_ok()
{
	lib::Xyz_Angle_Axis_vector position_aa;
	new_position.get_xyz_angle_axis(position_aa);

	for (int i = 0; i < 3; ++i) {
		if (position_aa(i + 3, 0) < 0) {
			log("cubic_constraint::is_rotation_ok(): position_aa(%d, 0) = %g\n", i + 3, position_aa(i + 3, 0));
		}

		if (rotation_min(i, 0) < rotation_division(i, 0)) {
			if (position_aa(i + 3, 0) < rotation_min(i, 0) || position_aa(i + 3, 0) >= rotation_division(i, 0)) {
				return false;
			}
		} else {
			if (position_aa(i + 3, 0) < rotation_min(i, 0) && position_aa(i + 3, 0) >= rotation_division(i, 0)) {
				return false;
			}
		}

		if (rotation_max(i, 0) > rotation_division(i, 0)) {
			if (position_aa(i + 3, 0) > rotation_max(i, 0) || position_aa(i + 3, 0) < rotation_division(i, 0)) {
				return false;
			}
		} else {
			if (position_aa(i + 3, 0) > rotation_max(i, 0) && position_aa(i + 3, 0) < rotation_division(i, 0)) {
				return false;
			}
		}
	}
	return true;
}

double cubic_constraint::get_distance_from_allowed_area()
{
	return 1.0;
}

} // namespace generator

}

}

}
