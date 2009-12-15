/*!
 * \file ecp_t_spotsrecognition.h
 * \brief Declaration of a class responsible
 * for robot moving, used by Piotr Sakowicz.
 * - class declaration
 * \author vented.baffle
 * \date 21.08.2008
 */

#ifndef _ECP_T_SpotsRecognition_H_
#define _ECP_T_SpotsRecognition_H_

#include "ecp/common/ecp_task.h"
#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/task/ecp_mp_t_rcsc.h"
#include "ecp_mp/sensor/ecp_mp_s_schunk.h"

//fradia
#include "ecp_mp/sensor/ecp_mp_s_cvfradia.h"
#include "ecp/common/ecp_t_cvfradia.h"

#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/irp6_on_track/ecp_g_spots_recognition.h"

#include "ecp/common/ecp_g_smooth.h"
#include "ecp/common/ecp_g_force.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

class spots_recognition: public common::task::task
{
	const char * trajektoria_poczatkowa;
	const char * trajektoria_koncowa;

	protected:

		generator::spots* generator;
		common::generator::smooth* smooth;
		common::generator::sr_nose_run* nose;
		common::generator::bias_edp_force* befg;


	public:
		spots_recognition(lib::configurator &_config);

		void main_task_algorithm(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif

