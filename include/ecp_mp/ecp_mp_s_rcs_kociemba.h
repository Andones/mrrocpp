// -------------------------------------------------------------------------
// Proces: 	EFFECTOR CONTROL PROCESS (ECP) 
// Plik:	ecp_mp_s_rcs_kociemba.h
// System:	QNX/MRROCPP  v. 6.3
// Opis:	czujnik znajdujacy rozwiazanie kostki Rubika algorytmem Kociemby
// Autor:	jsalacka
// Data:	25.03.2007
// -------------------------------------------------------------------------

#ifndef __ECP_RCS_KOCIEMBA_H
#define __ECP_RCS_KOCIEMBA_H

#include "ecp_mp/ecp_mp_sensor.h"				// klasa bazowa sensor

namespace mrrocpp {
namespace ecp_mp {

// ####################################################################
// ## KLASA czujnika - rozwiazywanie kostki Rubika algorytmem Kociemby  ##
// ####################################################################
class ecp_mp_rcs_kociemba : public ecp_mp_sensor{

  public:
	// Konstruktor czujnika wirtualnego - wywolanie konstruktora klasy bazowej.
 	ecp_mp_rcs_kociemba (SENSOR_ENUM _sensor_name, const char* _section_name, ecp_mp_task& _ecp_mp_object);
	// Konfiguracja czujnika.
	void configure_sensor (void);
	// Odebranie odczytu od VSP.
	void get_reading (void);
	// Inicjalizacja czujnika
	void initiate_reading();

}; 

} // namespace ecp_mp
} // namespace mrrocpp

#endif
