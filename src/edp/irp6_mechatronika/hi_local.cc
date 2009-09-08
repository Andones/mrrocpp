// ------------------------------------------------------------------------
//                            hi_rydz.cc
//
// Funkcje do obslugi sprzetu (serwomechanizmow cyfrowych) dla robota irp6 mechatronika
//
// Ostatnia modyfikacja: styczen 2005
// cala komunikacja ze sprzetem przerzucona do oblsugi przerwania ze wzgledou na drugi proces korzsytajacy z tego samego
// przerwania - tasmociag
// ------------------------------------------------------------------------

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <sys/types.h>
#include <unistd.h>
#include <semaphore.h>
#ifdef __QNXNTO__
#include <process.h>
#include <sys/neutrino.h>
#include <sys/sched.h>
#include <hw/inout.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#endif

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

// Klasa edp_irp6m_effector.
#include "edp/irp6_mechatronika/edp_irp6m_effector.h"
// Klasa hardware_interface.
#include "edp/irp6_mechatronika/hi_local.h"

namespace mrrocpp {
namespace edp {
namespace irp6m {

/*
#define CLOCKID CLOCK_REALTIME
#define SIG SIGRTMIN
*/

// ------------------------------------------------------------------------
hardware_interface::hardware_interface ( effector &_master )   : common::hardware_interface(_master)
{
    // tablica pradow maksymalnych d;a poszczegolnych osi
    int max_current [IRP6_MECHATRONIKA_NUM_OF_SERVOS] = {
    		IRP6_MECHATRONIKA_AXIS_1_MAX_CURRENT,
            IRP6_MECHATRONIKA_AXIS_2_MAX_CURRENT,
            IRP6_MECHATRONIKA_AXIS_3_MAX_CURRENT,
            IRP6_MECHATRONIKA_AXIS_4_MAX_CURRENT,
            IRP6_MECHATRONIKA_AXIS_5_MAX_CURRENT
    };

    // Sledzenie zera rezolwera - wylaczane
    trace_resolver_zero = false;

    irq_data.md.is_power_on = true;
    irq_data.md.is_robot_blocked = false;

    // nadanie odpowiednich uprawnien watkowi
    // w celu umozliwienia komunikacji z magistral isa i obslugi przerwania
#ifdef __QNXNTO__
    ThreadCtl (_NTO_TCTL_IO, NULL);

    memset(&irq_data.event, 0, sizeof(irq_data.event));
    irq_data.event.sigev_notify = SIGEV_INTR;

    int irq_no;    // Numer przerwania sprzetowego
#endif

    if(master.test_mode)
    {
#ifdef __QNXNTO__
        irq_no = 0;   // Przerwanie od zegara o okresie 1ms
#endif
        // domyslnie robot jest zsynchronizowany
        irq_data.md.is_synchronised = true;
    }
    else
    {
#ifdef __QNXNTO__
        irq_no = IRQ_REAL;   // Numer przerwania sprzetowego od karty ISA
#endif
        // domyslnie robot nie jest zsynchronizowany
        irq_data.md.is_synchronised = false;
    }
#ifdef __QNXNTO__
    // inicjacja wystawiania przerwan
	if (master.test_mode == 0) {
		// konieczne dla skasowania przyczyny przerwania
		out8(ADR_OF_SERVO_PTR, INTERRUPT_GENERATOR_SERVO_PTR);
		in16(SERVO_REPLY_STATUS_ADR); // Odczyt stanu wylacznikow
		in16(SERVO_REPLY_INT_ADR);
	}

    if ( (int_id = InterruptAttach (irq_no, int_handler, (void *) &irq_data , sizeof(irq_data), 0)) == -1)
    {
        // Obsluga bledu
        perror( "Unable to attach interrupt handler: ");
    }

    // oczekiwanie na przerwanie
    if (hi_int_wait(INT_EMPTY, 0)==-1) // jesli sie nie przyjdzie na czas
    {
        // inicjacja wystawiania przerwan
        if(master.test_mode==0)
        {
            // Ustawienie czestotliwosci przerwan
        	lib::WORD int_freq = SET_INT_FREQUENCY | INT_FREC_DIVIDER;
            out8(ADR_OF_SERVO_PTR, INTERRUPT_GENERATOR_SERVO_PTR);
            out16(SERVO_COMMAND1_ADR, int_freq);
            delay(10);
            out16(SERVO_COMMAND1_ADR, START_CLOCK_INTERRUPTS);
        }
    }
#else
    fprintf(stderr, "Blocking signal %d\n", SIGRTMIN);
    if (sigemptyset (&mask) == -1) {
    	perror("sigemptyset()");
    }
    if (sigaddset (&mask, SIGRTMIN) == -1) {
    	perror("sigaddset()");
    }

    /* Create the timer */
    struct sigevent sev;
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = SIGRTMIN;
    sev.sigev_value.sival_ptr = &timerid;
    if (timer_create (CLOCK_REALTIME, &sev, &timerid) == -1) {
    	perror("timer_create()");
    }

    /* Start the timer */
    struct itimerspec its;
    its.it_value.tv_sec = 0;
    its.it_value.tv_nsec = 1000000; // 1kHz
    its.it_interval.tv_sec = its.it_value.tv_sec;
    its.it_interval.tv_nsec = its.it_value.tv_nsec;

    if (timer_settime (timerid, 0, &its, NULL) == -1) {
    	perror("timer_settime()");
    }
#endif

    master.controller_state_edp_buf.is_synchronised = irq_data.md.is_synchronised;

    // Zakaz pracy recznej we wszystkich osiach

    for (int i = 0; i < IRP6_MECHATRONIKA_NUM_OF_SERVOS; i++ )
    {
        robot_status[i].adr_offset_plus_0 = 0;
        robot_status[i].adr_offset_plus_2 = 0;
        robot_status[i].adr_offset_plus_4 = 0;
        robot_status[i].adr_offset_plus_6 = 0;
        robot_status[i].adr_offset_plus_8 = 0;
        robot_status[i].adr_offset_plus_a = 0;
        meassured_current[i] = 0;

        if(master.test_mode==0)
        {
            /*
            out8(ADR_OF_SERVO_PTR, FIRST_SERVO_PTR + (lib::BYTE)i);
            out16(SERVO_COMMAND1_ADR,RESET_MANUAL_MODE); // Zerowanie ruchow recznych
            out16(SERVO_COMMAND1_ADR, PROHIBIT_MANUAL_MODE); // Zabrania ruchow za pomoca przyciskow w szafie
            */
            irq_data.md.card_adress=FIRST_SERVO_PTR + (lib::BYTE)i;
            irq_data.md.register_adress=SERVO_COMMAND1_ADR;
            irq_data.md.value=RESET_MANUAL_MODE;
            hi_int_wait(INT_SINGLE_COMMAND, 2);
            irq_data.md.value=PROHIBIT_MANUAL_MODE;
            hi_int_wait(INT_SINGLE_COMMAND, 2);
            irq_data.md.value=max_current[i];
            hi_int_wait(INT_SINGLE_COMMAND, 2);
        }
    }

    if(master.test_mode==0)
    {
        // Zerowanie licznikow polozenia wszystkich osi
        reset_counters();
        is_hardware_error();
    }

    first = true; // Pierwszy krok

    fprintf(stderr, "!!! OK !!!\n");
}
// ------------------------------------------------------------------------


// ------------------------------------------------------------------------
hardware_interface::~hardware_interface ( void )   // destruktor
{
#ifdef __QNXNTO__
    if(master.test_mode==0)
    {
        reset_counters();
        // Zezwolenie na prace reczna

        for (int i = 0; i < IRP6_MECHATRONIKA_NUM_OF_SERVOS; i++ )
        {
            irq_data.md.card_adress=FIRST_SERVO_PTR + (lib::BYTE)i;
            irq_data.md.register_adress=SERVO_COMMAND1_ADR;
            irq_data.md.value=ALLOW_MANUAL_MODE;
            hi_int_wait(INT_SINGLE_COMMAND, 2);
        }

        // TODO: should not call InterruptDetach()?
    }
#else
    /* delete interval timer */
    if(timer_delete(&timerid) == -1) {
    	perror("timer_delete()");
    }
#endif
}
// ------------------------------------------------------------------------


// ------------------------------------------------------------------------
uint64_t hardware_interface::read_write_hardware ( void )
{

    // ------------------------------------------------------------------------
    // Obsluga sprzetu: odczyt aktualnych wartosci polozenia i zapis wartosci
    // wypelnienia PWM

    int i;

    // zapis wartosci zadanych
    for (i = 0; i < IRP6_MECHATRONIKA_NUM_OF_SERVOS; i++ )
    {
        irq_data.md.robot_control[i].adr_offset_plus_0 = robot_control[i].adr_offset_plus_0;
    }

    // oczekiwanie na przerwanie
    hi_int_wait(INT_SERVOING, 0);

    if(master.test_mode)
    {
        // Tylko dla testow
        return irq_data.md.hardware_error;
    }

    //	 printf("hi rydz 1 current_absolute_position: %d, hex: %x\n", irq_data.md.current_absolute_position[5], irq_data.md.current_absolute_position[5] ); // debug

    for (i = 0; i < IRP6_MECHATRONIKA_NUM_OF_SERVOS; i++ )
    {

        // przepisanie wartosci pradu
        meassured_current[i] = (irq_data.md.robot_status[i].adr_offset_plus_2 & 0xFF00)>>8;

        current_absolute_position[i] = irq_data.md.current_absolute_position[i];
        current_position_inc[i] = current_absolute_position[i] -  previous_absolute_position[i];
        previous_absolute_position[i] = current_absolute_position[i];
    }

    if (!trace_resolver_zero)
    {
        //	printf("read_write_hardware: w mask resolver_zero\n");
        irq_data.md.hardware_error &= lib::MASK_RESOLVER_ZERO;
    }

    return irq_data.md.hardware_error;

}
// ------------------------------------------------------------------------


// ------------------------------------------------------------------------
// Zerowanie licznikow polozenia wszystkich osi
void hardware_interface::reset_counters ( void )
{
    for (int i = 0; i < IRP6_MECHATRONIKA_NUM_OF_SERVOS; i++ )
    {
        irq_data.md.card_adress=FIRST_SERVO_PTR + (lib::BYTE)i;
        irq_data.md.register_adress=SERVO_COMMAND1_ADR;
        irq_data.md.value=MICROCONTROLLER_MODE;
        hi_int_wait(INT_SINGLE_COMMAND, 2);
        irq_data.md.value=STOP_MOTORS;
        hi_int_wait(INT_SINGLE_COMMAND, 2);
        irq_data.md.value=RESET_MANUAL_MODE;
        hi_int_wait(INT_SINGLE_COMMAND, 2);
        irq_data.md.value=RESET_ALARM;
        hi_int_wait(INT_SINGLE_COMMAND, 2);

        if (!irq_data.md.is_synchronised)
        {
            irq_data.md.value=RESET_POSITION_COUNTER;
            hi_int_wait(INT_SINGLE_COMMAND, 2);
        }

        current_absolute_position[i] =  0;
        previous_absolute_position[i] = 0;
        current_position_inc[i] = 0.0;

        // 	in16(SERVO_REPLY_INT_ADR);
    }

    // Dwukrotny odczyt polozenia dla wyzerowania przyrostu wynikajacego z pierwszego
    // odczytu rezolwera
    // wyzerowanie wypelnienia
    for (int i = 0; i < IRP6_MECHATRONIKA_NUM_OF_SERVOS; i++ )
    {
        robot_control[i].adr_offset_plus_0 = 0x0200;
    }

    // wyzerowanie przyrostu pozycji
    read_write_hardware();
    read_write_hardware();
    read_write_hardware();
    // Odczyt polozenia osi slowo 32 bitowe - negacja licznikow 16-bitowych
    // out8(ADR_OF_SERVO_PTR, FIRST_SERVO_PTR);
    // out16(SERVO_COMMAND1_ADR, RESET_POSITION_COUNTER);
    // robot_status[0].adr_offset_plus_4 = 0xFFFF ^ in16(SERVO_REPLY_POS_LOW_ADR); // Mlodsze slowo 16-bitowe
    // robot_status[0].adr_offset_plus_6 = 0xFFFF ^ in16(SERVO_REPLY_POS_HIGH_ADR);// Starsze slowo 16-bitowe
    // printf("L=%x U=%x  \n",robot_status[0].adr_offset_plus_4, robot_status[0].adr_offset_plus_6);
}
// ------------------------------------------------------------------------


// ------------------------------------------------------------------------
bool hardware_interface::is_hardware_error ( void)
{
    bool h_error;
    lib::WORD MASK = 0x7E00;

    h_error = false;

    // oczekiwanie na przerwanie
    hi_int_wait(INT_SINGLE_COMMAND, 0);

    for (int i = 0; i < IRP6_MECHATRONIKA_NUM_OF_SERVOS; i++ )
    {
        if ( (irq_data.md.robot_status[i].adr_offset_plus_0 ^ 0x6000) & MASK )
        {
            h_error = true;
            //    printf(" \n => axis= %d r210H: %x ",i,robot_status[i].adr_offset_plus_0);
        }
    } // end: for
    return h_error;
}

// ------------------------------------------------------------------------


// synchronizacja automatyczna z wykrorzystaniem lm629
int hardware_interface::synchronise_via_lm629(void)
{
    int i;
    int wyjscie;

    for ( i = 0; i < IRP6_MECHATRONIKA_NUM_OF_SERVOS; i++ ) // UWAGA NA -1
    {
        // tryb pojedynczych polecen w obsludze przerwania
        irq_data.md.card_adress=FIRST_SERVO_PTR + (lib::BYTE)i;
        irq_data.md.register_adress=SERVO_COMMAND1_ADR;
        irq_data.md.value=LM629_VIA_MICROCONTROLLER_MODE;
        hi_int_wait(INT_SINGLE_COMMAND, 10);
        irq_data.md.value=FINISH_SYNCHRO;
        hi_int_wait(INT_SINGLE_COMMAND, 10);
        irq_data.md.value=START_SYNCHRO;
        hi_int_wait(INT_SINGLE_COMMAND, 10);
        irq_data.md.value=ZERO_ORDER;
        hi_int_wait(INT_SINGLE_COMMAND, 10);

        wyjscie=0;
        // dopoki nie osiagnieto pozycji synchronizacji
        while (!wyjscie)
        {
            // oczekiwanie na przerwanie
            hi_int_wait(INT_CHECK_STATE,0);
            if (0x0040&(irq_data.md.robot_status[i].adr_offset_plus_0))
            // jesli pojawi sie flaga zakonczenie synchronizacji
                wyjscie++;
        }

        // tryb pojedynczych polecen w obsludze przerwania
        irq_data.md.card_adress=FIRST_SERVO_PTR + (lib::BYTE)i;
        irq_data.md.register_adress=SERVO_COMMAND1_ADR;
        irq_data.md.value=MICROCONTROLLER_MODE;
        hi_int_wait(INT_SINGLE_COMMAND, 10);
    }

    // docelowo zwracac ew. bledy
    return 1;
}



int hardware_interface::hi_int_wait (int inter_mode, int lag)
{
#ifdef __QNXNTO__
    const uint64_t int_timeout = HI_RYDZ_INTR_TIMEOUT_HIGH;
    struct sigevent tim_event;

    static short interrupt_error = 0;
    static bool msg_send = false;

    tim_event.sigev_notify = SIGEV_UNBLOCK;

    //	printf("aaa\n");
    /*
    printf("1: %x, %x, %x, %x, %x, %x, %x\n", robot_control[0].adr_offset_plus_0, robot_control[1].adr_offset_plus_0,
    	robot_control[2].adr_offset_plus_0, robot_control[3].adr_offset_plus_0, robot_control[4].adr_offset_plus_0
    	, robot_control[5].adr_offset_plus_0, robot_control[6].adr_offset_plus_0);

    	*/
    TimerTimeout(CLOCK_REALTIME, _NTO_TIMEOUT_INTR ,  &tim_event, &int_timeout, NULL );
    irq_data.md.interrupt_mode=inter_mode;  // przypisanie odpowiedniego trybu oprzerwania
    //	irq_data.md.is_power_on = true;
    int iw_ret=InterruptWait (0, NULL);

    if (iw_ret==-1)
    { // jesli przerwanie nie przyjdzie na czas
        if (interrupt_error == 1)
            master.msg->message(lib::NON_FATAL_ERROR, "Nie odebrano przerwania - sprawdz szafe");
        interrupt_error++;
        master.controller_state_edp_buf.is_wardrobe_on = false;
    }
    else
    {
        if (interrupt_error >= 1)
            master.msg->message("Przywrocono obsluge przerwania");
        interrupt_error = 0;
        master.controller_state_edp_buf.is_wardrobe_on = true;
           master.controller_state_edp_buf.is_power_on = irq_data.md.is_power_on;
    }

    /*
    	if ((irq_data.md.robot_control[5].adr_offset_plus_0 > 810) && (irq_data.md.robot_control[5].adr_offset_plus_0 < 900))
    	printf("ttt: %d, %x, %d, %d\n", irq_data.md.current_absolute_position[5],  irq_data.md.robot_control[5].adr_offset_plus_0,
    		irq_data.md.robot_control[5].adr_offset_plus_0, irq_data.md.current_absolute_position[4]);
    */

    if ((interrupt_error>2) || (!master.controller_state_edp_buf.is_power_on))
    {
        if (msg_send == false) {
            master.msg->message(lib::NON_FATAL_ERROR, "Wylaczono moc - robot zablokowany");
            msg_send = true;
        }
        irq_data.md.is_robot_blocked = true;
    }

    master.controller_state_edp_buf.is_robot_blocked = irq_data.md.is_robot_blocked;

    if (lag!=0)
        delay(lag); // opoznienie niezbedne do przyjecia niektorych komend

    return iw_ret;
#else
    int sig;
    int s = sigwait(&mask, &sig);
    if (s != 0) {
    	perror("sigwait()");
    	return -1;
    }
    return 0;
#endif
}


void hardware_interface::start_synchro ( int drive_number )
{
    trace_resolver_zero = true;
    // Wlacz sledzenie zera rezolwera (synchronizacja robota)
    irq_data.md.card_adress=FIRST_SERVO_PTR + (lib::BYTE)drive_number;
    irq_data.md.register_adress=SERVO_COMMAND1_ADR;
    irq_data.md.value=START_SYNCHRO;
    hi_int_wait(INT_SINGLE_COMMAND, 2);
}


void hardware_interface::finish_synchro ( int drive_number )
{
    trace_resolver_zero = false;

    // Zakonczyc sledzenie zera rezolwera i przejdz do trybu normalnej pracy
    irq_data.md.card_adress=FIRST_SERVO_PTR + (lib::BYTE)drive_number;
    irq_data.md.register_adress=SERVO_COMMAND1_ADR;
    irq_data.md.value=FINISH_SYNCHRO;
    hi_int_wait(INT_SINGLE_COMMAND, 2);

    // by Y - UWAGA NIE WIEDZIEC CZEMU BEZ TEGO NIE ZAWSZE DZIALAJA RUCHY NA OSI PO SYNCHGORNIZACJi
    irq_data.md.value=MICROCONTROLLER_MODE;
    hi_int_wait(INT_SINGLE_COMMAND, 2);
}

} // namespace common
} // namespace edp
} // namespace mrrocpp
