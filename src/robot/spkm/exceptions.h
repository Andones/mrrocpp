/*!
 * @file exceptions.h
 * @brief File containing consts, types and classes related to exceptions specific for SPKM.
 *
 * @author Tomasz Kornuta
 * @date 08-02-2011
 *
 * @ingroup spkm
 */

#ifndef SPKM_EXCEPTION_H_
#define SPKM_EXCEPTION_H_

#include <string>

#include "base/edp/edp_exceptions.h"
#include "dp_spkm.h"

namespace mrrocpp {
namespace edp {
namespace spkm {

//! Pose specification type.
typedef boost::error_info <struct pose_specification_, mrrocpp::lib::spkm::POSE_SPECIFICATION> pose_specification;

//! Convert exception's to human-readable string
inline std::string to_string(pose_specification const & e)
{
	switch (e.value())
	{
		case lib::spkm::WRIST_XYZ_EULER_ZYZ:
			return "WRIST_XYZ_EULER_ZYZ";
		case lib::spkm::TOOL_XYZ_EULER_ZYZ:
			return "TOOL_XYZ_EULER_ZYX";
		case lib::spkm::JOINT:
			return "JOINT";
		case lib::spkm::MOTOR:
			return "MOTOR";
		default:
			return "UNKNOWN";
	}
}

//! Number of angle that caused the exception.
typedef boost::error_info <struct angle_number_, int> angle_number;

//! Number of angle that caused the exception.
typedef boost::error_info <struct device_name_, std::string> device_name;

/*!
 * \brief Exception thrown when cartesian pose is required, but unknown.
 * \author Tomasz Kornuta
 */
REGISTER_NON_FATAL_ERROR(nfe_current_cartesian_pose_unknown, "Required current cartesian pose is unknown")

/*!
 * \brief Exception thrown when new motion command while robot motion was in progress.
 * \author Tomasz Kornuta
 */
REGISTER_NON_FATAL_ERROR(nfe_motion_in_progress, "Command cannot be performed because robot motion still is in progress")

/*!
 * \brief Exception thrown when thyk alpha limit is exceeded.
 * \author Tomasz Kornuta
 */
REGISTER_NON_FATAL_ERROR(nfe_thyk_alpha_limit_exceeded, "Thyk alpha limit is exceeded")

/*!
 * \brief Exception thrown when thyk beta limit is exceeded.
 * \author Tomasz Kornuta
 */
REGISTER_NON_FATAL_ERROR(nfe_thyk_beta_limit_exceeded, "Thyk beta limit is exceeded")

/*!
 * \brief Exception thrown when the motor jam is detected.
 * \author Tomasz Kornuta
 */
REGISTER_FATAL_ERROR(fe_motor_jam_detected, "Motor jam detected")



} // namespace spkm
} // namespace edp
} // namespace mrrocpp


#endif /* SPKM_EXCEPTION_H_ */
