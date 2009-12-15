// ------------------------------------------------------------------------
//   ecp_t_nalewanie.cc
//
//                     EFFECTOR CONTROL PROCESS (lib::ECP) - main()
//
// Ostatnia modyfikacja: 2007
// ------------------------------------------------------------------------

#include <string.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include <fstream>

#include "lib/srlib.h"
#include "ecp_mp/task/ecp_mp_t_rcsc.h"
#include "ecp_mp/sensor/ecp_mp_s_schunk.h"

#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/irp6_postument/ecp_r_irp6p.h"
#include "ecp/common/task/ecp_t_nalewanie.h"
#include "lib/mathtr.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

// KONSTRUKTORY
nalewanie::nalewanie(lib::configurator &_config) : task(_config)
{
    if (config.section_name == ECP_IRP6_ON_TRACK_SECTION)
    {
        ecp_m_robot = new irp6ot::robot (*this);
    }
    else if (config.section_name == ECP_IRP6_POSTUMENT_SECTION)
    {
        ecp_m_robot = new irp6p::robot (*this);
    }

    sg = new generator::smooth (*this, true, true);

    sg->load_file_with_path ("../trj/rcsc/irp6ot_sm_ap_2.trj");

    sr_ecp_msg->message("ECP loaded");
}


void nalewanie::main_task_algorithm(void)
{
	/*
	lib::Homog_matrix *mat=new lib::Homog_matrix();
    double qq[7];

	mat->set_xyz_quaternion(1, 2, 3, 0.5, 0.5, 0.5, 0.5);
    mat->get_xyz_quaternion(qq);

    printf("%f, %f, %f, %f, %f, %f, %f\n", qq[0],qq[1],qq[2],qq[3],qq[4],qq[5],qq[6]);
    */

    sg->Move();
}

task* return_created_ecp_task (lib::configurator &_config)
{
	return new nalewanie(_config);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp
