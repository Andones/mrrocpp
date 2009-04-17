// -------------------------------------------------------------------------
// Proces: 	EFFECTOR CONTROL PROCESS (lib::ECP) 
// Plik:			ecp_mp_s_mic.h
// System:	QNX/MRROCPP  v. 6.3
// Opis:		Ogolna struktura obrazow czujnika
// Autor:		tkornuta
// Data:		29.11.2006
// -------------------------------------------------------------------------

#ifndef __ECP_MP_S_MIC_H
#define __ECP_MP_S_MIC_H

#include "ecp_mp/ecp_mp_sensor.h"				// klasa bazowa sensor

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {


/***************** Klasa czujnikow ********************/
class mic: public base{
  public:
	// Konstruktor czujnika wirtualnego - wywolanie konstruktora klasy bazowej.
 	mic (lib::SENSOR_ENUM _sensor_name, const char* _section_name, task::task& _ecp_mp_object);
											// konstruktor czujnika virtualnego

}; 


} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp

#endif
