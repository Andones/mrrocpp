#include "ecp_mp_g_hswitals_generatore.h"

namespace mrrocpp {
namespace ecp_mp {
namespace generator {
namespace hswitals_generatore {

void specification_data_type::set_compliance(bool x, bool y, bool z, bool ax, bool ay, bool az)
{
	if (x) {
		behaviour[0] = lib::CONTACT;
	} else {
		behaviour[0] = lib::UNGUARDED_MOTION;
	}

	if (y) {
		behaviour[1] = lib::CONTACT;
	} else {
		behaviour[1] = lib::UNGUARDED_MOTION;
	}

	if (z) {
		behaviour[2] = lib::CONTACT;
	} else {
		behaviour[2] = lib::UNGUARDED_MOTION;
	}

	if (ax) {
		behaviour[3] = lib::CONTACT;
	} else {
		behaviour[3] = lib::UNGUARDED_MOTION;
	}

	if (ay) {
		behaviour[4] = lib::CONTACT;
	} else {
		behaviour[4] = lib::UNGUARDED_MOTION;
	}

	if (az) {
		behaviour[5] = lib::CONTACT;
	} else {
		behaviour[5] = lib::UNGUARDED_MOTION;
	}

}

void specification_data_type::set_inertia(double x, double y, double z, double ax, double ay, double az)
{
    inertia[0] = x;
    inertia[1] = y;
    inertia[2] = z;
    inertia[3] = ax;
    inertia[4] = ay;
    inertia[5] = az;
}

void specification_data_type::set_reciprocal_damping(double x, double y, double z, double ax, double ay, double az)
{
    reciprocal_damping[0] = x;
    reciprocal_damping[1] = y;
    reciprocal_damping[2] = z;
    reciprocal_damping[3] = ax;
    reciprocal_damping[4] = ay;
    reciprocal_damping[5] = az;
}

specification_data_type::specification_data_type(bool x, bool y, bool z, bool ax, bool ay, bool az,
                                                                double force_inertia_x, double force_inertia_y, double force_inertia_z,
                                                                double torque_inertia_ax, double torque_inertia_ay, double torque_inertia_az,
                                                                double force_reciprocal_damping_x, double force_reciprocal_damping_y, double force_reciprocal_damping_z,
                                                                double torque_reciprocal_damping_x, double torque_reciprocal_damping_y, double torque_reciprocal_damping_z)
{
	set_compliance(x, y, z, ax, ay, az);
    set_inertia(force_inertia_x, force_inertia_y, force_inertia_z, torque_inertia_ax, torque_inertia_ay, torque_inertia_az);
    set_reciprocal_damping(force_reciprocal_damping_x, force_reciprocal_damping_y, force_reciprocal_damping_z, torque_reciprocal_damping_x, torque_reciprocal_damping_y, torque_reciprocal_damping_z);
}

specification_data_type::specification_data_type()
{
	set_compliance(true, true, true, true, true, true);
    set_inertia(lib::FORCE_INERTIA, lib::FORCE_INERTIA, lib::FORCE_INERTIA, lib::TORQUE_INERTIA, lib::TORQUE_INERTIA, lib::TORQUE_INERTIA);
    set_reciprocal_damping(lib::FORCE_RECIPROCAL_DAMPING, lib::FORCE_RECIPROCAL_DAMPING, lib::FORCE_RECIPROCAL_DAMPING, lib::TORQUE_RECIPROCAL_DAMPING, lib::TORQUE_RECIPROCAL_DAMPING, lib::TORQUE_RECIPROCAL_DAMPING);
}

}
} // namespace generator
} // namespace ecp_mp
} // namespace mrrocpp
