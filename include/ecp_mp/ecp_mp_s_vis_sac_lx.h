// -------------------------------------------------------------------------
// Proces: 	EFFECTOR CONTROL PROCESS (ECP) 
// Plik:			ecp_mp_s_mic.h
// System:	QNX/MRROCPP  v. 6.3
// Opis:		Ogolna struktura obrazow czujnika
// Autor:		tkornuta
// Data:		29.11.2006
// -------------------------------------------------------------------------

#ifndef __ECP_MP_S_VIS_SAC_LX_H
#define __ECP_MP_S_VIS_SAC_LX_H

#include "ecp_mp/ecp_mp_sensor.h"				// klasa bazowa sensor

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

/***************** Klasa czujnikow ********************/
class vis_sac_lx: public base{
  private:									// pola do komunikacji

  public:
	// Konstruktor czujnika wirtualnego - wywolanie konstruktora klasy bazowej.
 	vis_sac_lx (SENSOR_ENUM _sensor_name, const char* _section_name, task::base& _ecp_mp_object);
											// konstruktor czujnika virtualnego
//	void get_reading (void);			// odebranie odczytu od VSP
}; 

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp

#endif
