// ------------------------------------------------------------------------
// Proces: 	EFFECTOR CONTROL PROCESS (lib::ECP)
// Plik:			ecp_fs.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Testowy obraz testowego czujnika VSP
// Autor:		tkornuta
// Data:		04.11.2005
// ------------------------------------------------------------------------

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

// Zawiera klase bazowa ecp_mp_sensor.
#include "ecp_mp/sensor/ecp_mp_sensor.h"
// Zawiera klase ecp_mp_time_sensor.
#include "ecp_mp/sensor/ecp_mp_s_time.h"

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

/***************************** CONSTRUCTOR ********************************/
time::time (lib::SENSOR_ENUM _sensor_name, const char* _section_name, task::task& _ecp_mp_object):
	sensor (_sensor_name, _section_name, _ecp_mp_object) {
  union_size = sizeof(image.sensor_union.time);
}//: ecp_mp_time_sensor

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp
