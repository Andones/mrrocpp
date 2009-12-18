// -------------------------------------------------------------------------
// Proces: 	VIRTUAL SENSOR PROCESS (lib::VSP)
// Plik:			vsp_force_sensor.h
// System:	QNX/MRROCPP  v. 6.3
// Opis:		Deklaracja klasy vsp_schunk - czujnik sily.
// Autor:		tkornuta
// Data:		09.11.2005
// -------------------------------------------------------------------------

#if !defined(_VSP_SCHUNK_H)
#define _VSP_SCHUNK_H

#include "vsp/common/vsp_sensor.h"
#include "vsp/vsp_force_sensor.h"

namespace mrrocpp {
namespace vsp {
namespace sensor {

#define VSP_MSG_SEND_TIMEOUT_HIGH 10000000

/********** klasa czujnikow po stronie VSP **************/
class schunk : public sensor{

private:
	short ap_state, prev_ap_state, next_ap_state; // stan aplikacji z punktu widzenia wykrywanych zdarzen
		// 1- po biasie, 2 - przyleganie do powierzchni, 3 - unoszenie, 4 - uniesienie (jazda w powietrzu),
		// 5- opuszczanie, (dalej 2 przyleganie do powierzchni), 6 - nie wykrywa zdarzen

		int edp_vsp_fd; // do polaczenia z EDP


		struct sigevent event;
		lib::VSP_EDP_message vsp_edp_command;// by Y do komuniacji z EDP
		lib::EDP_VSP_reply edp_vsp_reply;

		unsigned int ms_nr; // numer odczytu z czujnika

		struct timespec start[9];

public:
	schunk(lib::configurator &_config);
	~schunk(void);

	void configure_sensor (void);	// konfiguracja czujnika
	void wait_for_event(void);		// oczekiwanie na zdarzenie
	void initiate_reading (void);		// zadanie odczytu od VSP
	void get_reading (void);			// odebranie odczytu od VSP		// zwraca blad
}; // end: class vsp_sensor

} // namespace sensor
} // namespace vsp
} // namespace mrrocpp

#endif
