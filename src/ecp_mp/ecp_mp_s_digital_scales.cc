// -------------------------------------------------------------------------
// Proces: 	EFFECTOR CONTROL PROCESS (ECP)
// Plik:			ecp_m_em.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		metody klasy ecp_mp_sensor dla czujnika z linialami
// Autor:		tkornuta
// Data:		30.11.2006
// -------------------------------------------------------------------------

/********************************* INCLUDES *********************************/

#include "ecp_mp/ecp_mp_s_digital_scales.h"

/***************************** CONSTRUCTOR ********************************/
ecp_mp_digital_scales_sensor::ecp_mp_digital_scales_sensor(SENSOR_ENUM _sensor_name, char* _section_name, ecp_mp_task& _ecp_mp_object) :
	ecp_mp_sensor(_sensor_name, _section_name, _ecp_mp_object)
{
	// Ustawienie wielkosci przesylanej unii.
	union_size = sizeof(image.ds);
	// Wyzerowanie odczytow.
	for (int i =0; i<6; i++)
		image.ds.readings[i] = 0;
}
