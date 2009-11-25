// -------------------------------------------------------------------------
//                            sg_local.h
// Definicje struktur danych i metod dla procesu EDP postument
//
// Ostatnia modyfikacja: 2006
// -------------------------------------------------------------------------


#ifndef __SG_IRP6P_H
#define __SG_IRP6P_H

#include "edp/common/edp.h"
#include "edp/common/sg_irp6p_and_conv.h"

// os od ktorej startuje synchronizacja - numeracja od 0
#define IRP6P_SYN_INIT_AXE 1

namespace mrrocpp {
namespace edp {
namespace irp6p {
class effector;

class servo_buffer: public common::servo_buffer
{
		// Bufor polecen przysylanych z EDP_MASTER dla SERVO
		// Obiekt z algorytmem regulacji

		uint8_t Move_a_step(void); // wykonac ruch o krok nie reagujac na SYNCHRO_SWITCH i SYNCHRO_T

	public:
		// output_buffer
		void get_all_positions(void);
		effector &master;

		servo_buffer(effector &_master); // konstruktor
		~servo_buffer(void); // destruktor

		void synchronise(void); // synchronizacja
		uint64_t compute_all_set_values(void);
		// obliczenie nastepnej wartosci zadanej dla wszystkich napedow
};

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
