/**
 * @file ecp_g_stats_generator.h
 * @brief Header file for stats_generator class
 * @author Tomasz Bem (mebmot@wp.pl)
 * @author Rafal Tulwin (rtulwin@stud.elka.pw.edu.pl)
 * @ingroup stats
 * @date 02.07.2010
 */

#ifndef ECP_G_STATS_GENERATOR_H_
#define ECP_G_STATS_GENERATOR_H_

#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/mrmath/mrmath.h"

#include "generator/ecp/ecp_g_teach_in.h"
#include <vector>

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/**
 * @brief Generator for executing trajectory and generating appropriate statistics.
 */
class stats_generator : public common::generator::generator
{
private:

public:
	stats_generator(common::task::task& _ecp_task);
	virtual ~stats_generator();
	virtual bool first_step();
	virtual bool next_step();

	void reset();
};

}//generator
}//common
}//ecp
}//mrrocpp

#endif /* ECP_G_STATS_GENERATOR_H_ */
