// -------------------------------------------------------------------------
//                            hi_rydz.h
// Definicje struktur danych i metod dla interfejsu sprzetowego dla robota irp6 postument
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __HI_LOCAL_IRP6P_H
#define __HI_LOCAL_IRP6P_H

#include <signal.h>
#include <time.h>

#include "edp/common/hi_rydz.h"

namespace mrrocpp {
namespace edp {
namespace irp6p {

// Struktury danych wykorzystywane w hardware_interface
const int IRQ_REAL = 10; // Numer przerwania sprzetowego
const unsigned short int INT_FREC_DIVIDER = 8; // mnoznik czestotliwosci przerwan (odpowiada 2ms)

#define HI_RYDZ_INTR_TIMEOUT_HIGH 10000000 // by Y - timeout przerwania z szafy badz zegara

#define FIRST_SERVO_PTR           0xC1
#define INTERRUPT_GENERATOR_SERVO_PTR	 0xC0


#define ISA_CARD_OFFSET 0x20 // w zaleznosci od ustawienia na karcie isa

#define IRP6_POSTUMENT_AXIS_1_MAX_CURRENT           0x24FF // ustawienie pradu maksymalnego dla przedostatniej osi - obrot chwytaka
#define IRP6_POSTUMENT_AXIS_2_MAX_CURRENT           0x24FF // ustawienie pradu maksymalnego dla przedostatniej osi - obrot chwytaka
#define IRP6_POSTUMENT_AXIS_3_MAX_CURRENT           0x24FF // ustawienie pradu maksymalnego dla przedostatniej osi - obrot chwytaka
#define IRP6_POSTUMENT_AXIS_4_MAX_CURRENT           0x24FF // ustawienie pradu maksymalnego dla przedostatniej osi - obrot chwytaka
#define IRP6_POSTUMENT_AXIS_5_MAX_CURRENT           0x24FF // ustawienie pradu maksymalnego dla przedostatniej osi - obrot chwytaka
#define IRP6_POSTUMENT_AXIS_6_MAX_CURRENT           0x24FF // ustawienie pradu maksymalnego dla przedostatniej osi - obrot chwytaka
// 13,7 j na amper

#define IRP6_POSTUMENT_AXIS_7_MAX_CURRENT           0x2420 // ustawienie pradu maksymalnego dla zacisku chwytaka
// 34,7 j na 100ma, streafa nieczulosci 40ma

class effector;

// ------------------------------------------------------------------------
//                HARDWARE_INTERFACE class
// ------------------------------------------------------------------------

class hardware_interface: public common::hardware_interface {
public:
	hardware_interface(effector &_master); // Konstruktor
	~hardware_interface(void); // Destruktor
	bool is_hardware_error(void); // Sprawdzenie czy wystapil blad sprzetowy

	uint64_t read_write_hardware(void); // Obsluga sprzetu
	void reset_counters(void); // Zerowanie licznikow polozenia

	void start_synchro(int drive_number);

	// oczekiwanie na przerwanie - tryb obslugi i delay(lag) po odebraniu przerwania
	int hi_int_wait(int inter_mode, int lag);

	void finish_synchro(int drive_number);

private:
	edp::common::irq_data_t irq_data;

	//! periodic timer
	timer_t timerid;

	//! periodic timer signal mask
	sigset_t mask;
}; // koniec: class hardware_interface

#ifdef __cplusplus
extern "C"
{
#endif
    const struct sigevent *
                int_handler (void *arg, int id);
#ifdef __cplusplus
}
#endif


} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif // __HI_RYDZ_H
