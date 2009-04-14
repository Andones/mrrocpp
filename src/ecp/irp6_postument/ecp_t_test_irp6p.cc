// ------------------------------------------------------------------------
//   ecp_t_tran.cc - przezroczyste wersja dla dowolnego z robotow
//
//                     EFFECTOR CONTROL PROCESS (lib::ECP) - main()
//
// Ostatnia modyfikacja: 2006
// ------------------------------------------------------------------------


#include <stdio.h>
#include <unistd.h>
#include <map>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"

#include "ecp/irp6_postument/ecp_local.h"
#include "ecp/common/ecp_g_jarosz.h"
#include "ecp/irp6_postument/ecp_t_test_irp6p.h"
#include "ecp_mp/ecp_mp_s_schunk.h"
#include "ecp/irp6_postument/ecp_g_test.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p {
namespace task {


// KONSTRUKTORY
test::test(lib::configurator &_config) : base(_config)
{
}

// methods for ECP template to redefine in concrete classes
void test::task_initialization(void)
{
	ecp_m_robot = new ecp_irp6_postument_robot (*this);

		// Powolanie czujnikow
	sensor_m[lib::SENSOR_FORCE_POSTUMENT] =
		new ecp_mp::sensor::schunk (lib::SENSOR_FORCE_POSTUMENT, "[vsp_force_irp6p]", *this);

	// Konfiguracja wszystkich czujnikow
	for (std::map <lib::SENSOR_ENUM, lib::sensor*>::iterator sensor_m_iterator = sensor_m.begin();
		 sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
	{
		sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
		sensor_m_iterator->second->configure_sensor();
	}


	usleep(1000*100);

	sr_ecp_msg->message("ECP loaded");
}


void test::main_task_algorithm(void)
{
	generator::y_simple ysg(*this, 8);
	ysg.sensor_m = sensor_m;

	for(;;) {
		sr_ecp_msg->message("NOWA SERIA");
		sr_ecp_msg->message("Ruch");
		sr_ecp_msg->message("Zakocz - nacisnij PULSE ECP trigger");
		ysg.Move();
	}
}

}
} // namespace irp6p

namespace common {
namespace task {

base* return_created_ecp_task (lib::configurator &_config)
{
	return new irp6p::task::test(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

