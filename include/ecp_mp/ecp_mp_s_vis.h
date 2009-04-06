// -------------------------------------------------------------------------
// Proces: 	EFFECTOR CONTROL PROCESS (ECP) 
// Plik:			ecp_mp_s_vis.h
// System:	QNX/MRROCPP  v. 6.3
// Opis:		Ogolna struktura obrazow czujnika
// Autor:		tkornuta
// Data:		29.11.2006
// -------------------------------------------------------------------------

#ifndef __ECP_MP_S_VIS_H
#define __ECP_MP_S_VIS_H

#include "ecp_mp/ecp_mp_sensor.h"				// klasa bazowa sensor

namespace mrrocpp {
namespace ecp_mp {

/***************** Klasa czujnikow ********************/
class ecp_mp_vis_sensor: public ecp_mp_sensor{
  public:
	// Konstruktor czujnika wirtualnego - wywolanie konstruktora klasy bazowej.
 	ecp_mp_vis_sensor (SENSOR_ENUM _sensor_name, const char* _section_name, ecp_mp_task& _ecp_mp_object);
											// konstruktor czujnika virtualnego
}; 

} // namespace ecp_mp
} // namespace mrrocpp

#endif
