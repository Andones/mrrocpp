/*!
 * @brief File containing consts, types and classes related to exceptions specific to ECP
 *
 * @author yoyek

 * @ingroup ecp
 */

#ifndef ECP_EXCEPTION_H_
#define ECP_EXCEPTION_H_

#include "base/lib/exception.h"

namespace mrrocpp {
namespace ecp {
namespace exception {

/*!
 * \brief MP non fatal error
 * \author yoyek
 */
// REGISTER_NON_FATAL_ERROR(nfe, "MP non_fatal_error")
/*!
 * \brief MP generator non fatal error
 * \author yoyek
 */
// REGISTER_NON_FATAL_ERROR(nfe_g, "MP generator non_fatal_error")
/*!
 * \brief MP robot non fatal error
 * \author yoyek
 */
// REGISTER_NON_FATAL_ERROR(nfe_r, "MP generator non_fatal_error")
/*!
 * \brief ECP System error
 * \author yoyek
 */
REGISTER_SYSTEM_ERROR(se, "ECP system_error")

} // namespace exception
} // namespace ecp
} // namespace mrrocpp

#endif
