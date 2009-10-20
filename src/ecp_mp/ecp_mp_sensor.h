#if !defined(_ECP_MP_SENSOR_H)
#define _ECP_MP_SENSOR_H

#include "lib/sensor.h"
#include "lib/messip/messip.h"

#include <map>

namespace mrrocpp {
namespace ecp_mp {
namespace task {
// TODO: Forward declaration
class task;
}
}
}

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

// Klasa bazowa czujnikow po stronie procesu ECP.
class sensor: public lib::sensor {
protected:
	//! Sensor manger descriptor
#if !defined(USE_MESSIP_SRR)
	int sd;
#else
	//! Sensor descriptor
	messip_channel_t *sd;
#endif /* USE_MESSIP_SRR */
	// Nazwa czujnika.
	std::string VSP_NAME;

	// Wskaznik na obiekt do komunikacji z SR
	lib::sr_ecp &sr_ecp_msg;

public:

	const lib::SENSOR_ENUM sensor_name; // nazwa czujnika z define w impconst.h

	// Wlasciwy konstruktor czujnika wirtualnego.
	sensor(lib::SENSOR_ENUM _sensor_name, const char* _section_name, task::task& _ecp_mp_object);

	// TODO: Destruktor czujnika wirtualnego
//	virtual ~sensor();

	virtual void configure_sensor(void);
	virtual void initiate_reading(void);
	virtual void terminate(void);
	virtual void get_reading(void);
	void get_reading(lib::SENSOR_IMAGE & sensor_image);
};

} // namespace sensor

// Kontener zawierajacy wykorzystywane czyjniki
typedef std::map<lib::SENSOR_ENUM, lib::sensor *> sensors_t;

} // namespace ecp_mp
} // namespace mrrocpp

#endif /* _ECP_MP_SENSOR_H */
