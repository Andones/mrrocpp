#if !defined(_ECP_MP_G_HSWITALS_GENERATORE_H)
#define _ECP_MP_G_HSWITALS_GENERATORE_H

/*!
 * @file
 * @brief File contains tff nose run label definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup generators
 */

#include "../../base/lib/com_buf.h"
#include "../../base/lib/impconst.h"

namespace mrrocpp {
namespace ecp_mp {
namespace generator {
namespace hswitals_generatore {
enum communication_type
{
    no_data = 0, specification = 1
};

/**
 * @brief Enum to define mp to ecp communication variants
 */

class specification_data_type
{
public:
    lib::BEHAVIOUR_SPECIFICATION behaviour[6];
    double inertia[6];
    double reciprocal_damping[6];

    void set_compliance(bool x, bool y, bool z, bool ax, bool ay, bool az);
    void set_inertia(double x, double y, double z, double ax, double ay, double az);
    void set_reciprocal_damping(double x, double y, double z, double ax, double ay, double az);

    specification_data_type();
    specification_data_type(bool x, bool y, bool z, bool ax, bool ay, bool az,
                            double force_inertia_x, double force_inertia_y, double force_inertia_z,
                            double torque_inertia_ax, double torque_inertia_ay, double torque_inertia_az,
                            double force_reciprocal_damping_x, double force_reciprocal_damping_y, double force_reciprocal_damping_z,
                            double torque_reciprocal_damping_x, double torque_reciprocal_damping_y, double torque_reciprocal_damping_z);

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		// serialize base class information
		ar & behaviour;
        ar & inertia;
        ar & reciprocal_damping;
	}

};

}
/*!
 * @brief hswitals_generatore label
 */
const std::string ECP_GEN_HSWITALS_GENERATORE = "ECP_GEN_HSWITALS_GENERATORE";

} // namespace generator
} // namespace ecp_mp
} // namespace mrrocpp

#endif
