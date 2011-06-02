/*!
 * @file
 * @brief File contains mp empty generator definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup mp
 */

#include <cstring>

#include <boost/foreach.hpp>

#include "base/mp/MP_main_error.h"
#include "base/mp/mp_robot.h"

#include "robot/player/ecp_mp_t_player.h"
#include "base/mp/generator/mp_g_sporadicaly_coordinated.h"

namespace mrrocpp {
namespace mp {
namespace generator {

// ###############################################################
// Generator pusty. Faktyczna generacja trajektorii odbywa sie w ECP
// ###############################################################

continously_coordinated::continously_coordinated(task::task& _mp_task) :
	generator(_mp_task), cycle_counter(0)
{
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step --------------------------------------
// ----------------------------------------------------------------------------------------------

bool continously_coordinated::next_step()
{
	// cykl oznacza odebranie danych od wszystkich robotow

	// sprawdza czy wszystkie roboty


	cycle_counter++;

	return next_step_inside();
}

} // namespace generator
} // namespace mp
} // namespace mrrocpp

