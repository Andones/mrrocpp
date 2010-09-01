#if !defined(__IRP6_TFG_DATA_PORT_H)
#define __IRP6_TFG_DATA_PORT_H

/*!
 * @file
 * @brief File contains data port communication structures for Irp6 two finger grippers
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup irp6ot_tfg irp6p_tfg
 */

namespace mrrocpp {
namespace lib {
namespace irp6_tfg {

struct command
{
	double desired_position;
};

}
}
}

#endif
