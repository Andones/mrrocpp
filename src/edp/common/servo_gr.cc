/* --------------------------------------------------------------------- */
/*                          SERVO_GROUP Process                          */
// ostatnia modyfikacja - styczen 2005
/* --------------------------------------------------------------------- */

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/mis_fun.h"
#include "edp/common/edp.h"
#include "edp/common/reader.h"
#include "edp/common/hi_rydz.h"
#include "edp/common/servo_gr.h"

namespace mrrocpp {
namespace edp {
namespace common {


void * manip_and_conv_effector::servo_thread_start(void* arg)
{
    return static_cast<manip_and_conv_effector*> (arg)->servo_thread(arg);
}

void * manip_and_conv_effector::servo_thread(void* arg)
{
	// servo buffer has to be created before servo thread starts
//	std::auto_ptr<servo_buffer> sb(return_created_servo_buffer()); // bufor do komunikacji z EDP_MASTER

    lib::set_thread_priority(pthread_self(), MAX_PRIORITY+2);

    // signal master thread to continue executing
    {
    	boost::mutex::scoped_lock lock(thread_started_mutex);

    	thread_started = true;

    	thread_started_cond.notify_one();
    }

    /* BEGIN SERVO_GROUP */

    for (;;)
    {
        // komunikacja z transformation
        if (!sb->get_command())
        {

            rb_obj->lock_mutex();
            rb_obj->step_data.servo_mode = false; // tryb bierny
            rb_obj->unlock_mutex();

            /* Nie otrzymano nowego polecenia */
            /* Krok bierny - zerowy przyrost polozenia */
            // Wykonanie pojedynczego kroku ruchu
            sb->Move_passive();
        }
        else
        {
        	// nowe polecenie
            rb_obj->lock_mutex();
            rb_obj->step_data.servo_mode = true; // tryb czynny
            rb_obj->unlock_mutex();

            switch (sb->command_type())
            {
				case lib::SYNCHRONISE:
					sb->synchronise(); // synchronizacja
					break;
				case lib::MOVE:
					sb->Move(); // realizacja makrokroku ruchu
					break;
				case lib::READ:
					sb->Read(); // Odczyt polozen
					break;
				case lib::SERVO_ALGORITHM_AND_PARAMETERS:
					sb->Change_algorithm(); // Zmiana algorytmu serworegulacji lub jego parametrow
					break;
				default:
					// niezidentyfikowane polecenie (blad) nie moze wystapic, bo juz
					// wczesniej zostalo wychwycone przez get_command()
					break;
            }
        } // end: else
    }

    return NULL;
} // end: main() SERVO_GROUP




uint8_t servo_buffer::Move_a_step (void)
{
    return 0;
}

void servo_buffer::clear_reply_status ( void )
{
    // zeruje skladowe reply_status
    reply_status.error0 = OK;
    reply_status.error1 = OK;
}

void servo_buffer::clear_reply_status_tmp ( void )
{
    // zeruje skladowe reply_status_tmp
    reply_status_tmp.error0 = OK;
    reply_status_tmp.error1 = OK;
}

// input_buffer
lib::SERVO_COMMAND servo_buffer::command_type()
{
    return command.instruction_code;
}
// by Yoyek & 7 -  typ returna na lib::SERVO_COMMAND

// output_buffer
void servo_buffer::get_all_positions (void)
{}


servo_buffer::servo_buffer (manip_and_conv_effector &_master)
        : master(_master)
{}
             // konstruktor

void servo_buffer::synchronise (void)
{}
         // synchronizacja
uint64_t servo_buffer::compute_all_set_values (void)
{
    return 0;
}


/*-----------------------------------------------------------------------*/
bool servo_buffer::get_command (void)
{
    // Odczytanie polecenia z EDP_MASTER o ile zostalo przyslane
	bool new_command_available = false;

#ifdef __QNXNTO__
    struct sigevent msg_event;
    msg_event.sigev_notify = SIGEV_UNBLOCK;// by Y zamiast creceive

    TimerTimeout(CLOCK_REALTIME, _NTO_TIMEOUT_RECEIVE,  &msg_event, NULL, NULL ); // by Y zamiast creceive i flagi z EDP_MASTER
    if ((edp_caller = MsgReceive_r(master.servo_to_tt_chid, &command, sizeof(command), NULL)) >= 0)
    	new_command_available = true;
#else
    {
    	boost::lock_guard<boost::mutex> lock(master.servo_command_mtx);
    	if(master.servo_command_rdy) {
    		command = master.servo_command;
    		master.servo_command_rdy = false;
    		new_command_available = true;
    	}
    }
#endif

    if (new_command_available)
    { // jezeli jest nowa wiadomosc

        /* Otrzymano nowe polecenie */
        // Ewentualna reakcja na uprzednie wystapienie bledu
        if ( (reply_status.error0 != OK) || (reply_status.error1 != OK) )
        {
            // Powiadomienie EDP_MASTER o uprzednio wykrytym bledzie
            reply_status.error0 |= SERVO_ERROR_IN_PASSIVE_LOOP;
            clear_reply_status_tmp();
            reply_to_EDP_MASTER();
            return false; // Potraktowac jakby nie bylo polecenia
        } // end: if

        // Uprzednio nie bylo bledu => wstepna analiza polecenia
        switch (command_type())
        {
        case lib::SYNCHRONISE:
            return true; // wyjscie bez kontaktu z EDP_MASTER
        case lib::MOVE:
            return true; // wyjscie bez kontaktu z EDP_MASTER
        case lib::READ:
            return true; // wyjscie bez kontaktu z EDP_MASTER
        case lib::SERVO_ALGORITHM_AND_PARAMETERS:
            return true; // wyjscie bez kontaktu z EDP_MASTER
        default: // otrzymano niezidentyfikowane polecenie => blad
            reply_status.error0 = UNIDENTIFIED_SERVO_COMMAND;
            clear_reply_status_tmp();

            reply_to_EDP_MASTER();
            return false; // Potraktowac jakby nie bylo polecenia
        } // end: switch
    }
    else
#ifdef __QNXNTO__
    {
    	/* Nie otrzymano nowego polecenia ruchu */
        if (edp_caller != -ETIMEDOUT) {
        	// nastapil blad przy odbieraniu wiadomosci rozny od jej braku
            fprintf(stderr, "SERVO_GROUP: Receive error from EDP_MASTER\n");
        }
        return false;
    }
#else
	{
		/* Nie otrzymano nowego polecenia ruchu */
        return false;
	}
#endif
} // end: servo_buffer::get_command
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
uint8_t servo_buffer::Move_1_step (void)
{
    // wykonac ruch o krok
    // Odebranie informacji o uprzednio zrealizowanym polozeniu oraz ewentualnej awarii
    // Obliczenie nowej wartosci zadanej
    // Wyslanie wartosci zadanej do hardware'u

    master.rb_obj->set_new_step();// odwieszenie watku edp_reader - teraz moze odczytac dane pomiarowe

    reply_status_tmp.error1 = compute_all_set_values();  // obliczenie nowej wartosci zadanej dla regulatorow
    reply_status_tmp.error0 = hi->read_write_hardware();  // realizacja kroku przez wszystkie napedy oraz
    // odczyt poprzedniego polozenia
    master.step_counter++;

    master.rb_obj->lock_mutex();

    struct timespec step_time;

    if( clock_gettime( CLOCK_REALTIME , &step_time) == -1 )
    {
        perror("clock_gettime()");
    }

    master.rb_obj->step_data.step =  master.step_counter;
    master.rb_obj->step_data.msec=(int)(step_time.tv_nsec/1000000);

    master.rb_obj->unlock_mutex();


    if ( reply_status_tmp.error0 || reply_status_tmp.error1 )
    {
        // 	std::cout<<"w move 1 step error detected\n";
        return ERROR_DETECTED;  // info o awarii
    }
    else
        return NO_ERROR_DETECTED;
}
/*-----------------------------------------------------------------------*/



/*-----------------------------------------------------------------------*/
uint8_t servo_buffer::convert_error (void)
{
    // zwraca NO_ERROR_DETECTED, gdy OK lub wykryto SYNCHRO_SWITCH oraz SYNCHRO_ZERO,
    // w ktorejs osi, a w pozostalych przypadkach dokonuje konwersji numeru bledu

    // wycinamy po dwa najmlodsze bity z kazdej piatki zwiazanej z napedem
    uint64_t err1 = OK;
    for (int i = 0; i < master.number_of_servos; i++)
    {
        reply_status_tmp.error0 >>= 2;
        err1 |= (reply_status_tmp.error0 & 0x07) << (3*i);
        reply_status_tmp.error0 >>= 3;
    }

    reply_status_tmp.error0 = err1;
    if ( reply_status_tmp.error0 || reply_status_tmp.error1 )
    {
        return ERROR_DETECTED;  // info o awarii
    }
    else
        return NO_ERROR_DETECTED;
}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
void servo_buffer::Move_passive (void)
{ //
    // stanie w miejscu - krok bierny

    for (int j = 0; j < master.number_of_servos; j++)
    {
        regulator_ptr[j]->insert_new_step(0.0); // zerowy przyrost polozenia dla wszystkich napedow
    }

    // Wykonanie pojedynczego kroku ruchu
    if ( Move_a_step() )
    {
        // informacja dla EDP -> w trakcie realizacji petli biernej nastapila awaria
        reply_status.error0 = reply_status_tmp.error0 | SERVO_ERROR_IN_PASSIVE_LOOP;
        reply_status.error1 = reply_status_tmp.error1;
        clear_reply_status_tmp();
        // awaria w petli biernej nie bedzie naprawiana
    } // end: if

    /* Czy przeslac informacje o stanie SERVO do EDP_MASTER? */
    if (send_after_last_step)
    { // Tak, przeslac
        reply_to_EDP_MASTER();
    }
}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
void servo_buffer::Move (void)
{

    double new_increment[master.number_of_servos];

    // wykonanie makrokroku ruchu

    // okreslenie momentu wyslania informacji o zakonczeniu pierwszej fazy ruchu  do READING_BUFFER
    if (command.parameters.move.return_value_in_step_no > command.parameters.move.number_of_steps)
        send_after_last_step = true;
    else
        send_after_last_step = false;

    /*
    regulator_ptr[0]->insert_new_step((command.parameters.move.abs_position[0] - hi->get_position(0)*(2*M_PI)/IRP6_POSTUMENT_AXIS_0_TO_5_INC_PER_REVOLUTION) /
    	command.parameters.move.number_of_steps));
    */

    for (int  k = 0; k < master.number_of_servos; k++)
    {
        new_increment[k] = command.parameters.move.macro_step[k] / command.parameters.move.number_of_steps;
    }

    // realizacja makrokroku przez wszystkie napedy;  i - licznik krokow ruchu
    for (uint16_t j = 0; j < command.parameters.move.number_of_steps; j++)
    {
        // by Y
        // XXX by ptroja
        if ((command.parameters.move.return_value_in_step_no==0)&&(j == command.parameters.move.return_value_in_step_no))
        {
            // czy juz wyslac info do EDP_MASTER?
            // 	     std::cout<<"fsD\n";
            if ( reply_status.error0 || reply_status.error1 )
            {
                std::cout<<"w 1 reply error\n";
            }

            reply_to_EDP_MASTER();
        }
        // end by Y
        // Wykonanie pojedynczego kroku ruchu z jednoczesnym
        // sprawdzeniem, czy w jakims serwomechanizmie nastapila awaria

        for (int  k = 0; k < master.number_of_servos; k++)
        {
            regulator_ptr[k]->insert_new_step(new_increment[k]);
            if (master.test_mode)
            {
                master.update_servo_current_motor_pos_abs(
                    regulator_ptr[k]->previous_abs_position + new_increment[k]*j ,k);
            }
        }

      if ( Move_a_step() == NO_ERROR_DETECTED)
        { // NO_ERROR_DETECTED
            //  std::cout<<"NO_ERROR_DETECTED\n";
            if ((command.parameters.move.return_value_in_step_no>0)&&(j == command.parameters.move.return_value_in_step_no-1))
            {
                // czy juz wyslac info do EDP_MASTER?
                if ( reply_status.error0 || reply_status.error1 )
                {
                    std::cout<<"w drugim reply error\n";
                }
                reply_to_EDP_MASTER();
            }
        }
        else
        { // ERROR_DETECTED
            //  std::cout<<"ERROR_DETECTED\n";
            if (j > command.parameters.move.return_value_in_step_no-1 )
            {
                reply_status.error0 = reply_status_tmp.error0 | SERVO_ERROR_IN_PHASE_2;
                reply_status.error1 = reply_status_tmp.error1;
                clear_reply_status_tmp();
            }
            else
            {
                reply_status.error0 = reply_status_tmp.error0 | SERVO_ERROR_IN_PHASE_1;
                reply_status.error1 = reply_status_tmp.error1;
                clear_reply_status_tmp();
                std::cout<<"ERROR_DETECTED 2\n";
                reply_to_EDP_MASTER();
            }
            break; // przerwac ruch, bo byl blad
        }
    }

    for ( int i = 0;  i < master.number_of_servos; i++)
    {
        regulator_ptr[i]->previous_abs_position = command.parameters.move.abs_position[i];
    }
}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
void servo_buffer::reply_to_EDP_MASTER (void)
{
    // przeslanie stanu SERVO_GROUP do EDP_MASTER z inicjatywy SERVO_GROUP

    servo_data.error.error0 = reply_status.error0;
    servo_data.error.error1 = reply_status.error1;
    // W.S. cprintf("err0 = %lx    err1 = %lx\n",servo_data.error.error0,servo_data.error.error1);
    // Przepisanie aktualnych polozen servo do pakietu wysylkowego
    get_all_positions();

    // Wyslac informacje do EDP_MASTER
#ifdef __QNXNTO__
    if (MsgReply(edp_caller,EOK, &servo_data, sizeof(lib::servo_group_reply)) < 0)
        perror (" Reply to EDP_MASTER error");
#else
    {
    	boost::lock_guard<boost::mutex> lock(master.sg_reply_mtx);

    	master.sg_reply = servo_data;
    	master.sg_reply_rdy = true;
    }

	// TODO: should not call notify with mutex locked?
    master.sg_reply_cond.notify_one();
#endif
    // Wyzerowac zmienne sygnalizujace stan procesu
    clear_reply_status();
    send_after_last_step = false;
}
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
void servo_buffer::ppp (void) const
{
    // wydruk kontrolny polecenia przysylanego z EDP_MASTER
    std::cout << " macro_step= " << command.parameters.move.macro_step << "\n";
    std::cout << " number_of_steps= " << command.parameters.move.number_of_steps << "\n";
    std::cout << " return_value_in_step_no= " << command.parameters.move.return_value_in_step_no << "\n";
}
/*-----------------------------------------------------------------------*/




/*-----------------------------------------------------------------------*/
servo_buffer::~servo_buffer(void)
{
    // Destruktor grupy regulatorow
    // Zniszcyc regulatory
    for (int j = 0; j < master.number_of_servos; j++)
        delete regulator_ptr[j];
}
/*-----------------------------------------------------------------------*/




/*-----------------------------------------------------------------------*/
void servo_buffer::Read (void)
{
    // odczytac aktualne polozenie
    // wyslac do EDP_MASTER
    reply_to_EDP_MASTER();
}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
void servo_buffer::Change_algorithm (void)
{
    // Zmiana numeru algorytmu regulacji oraz numeru zestawu jego parametrow
    for (int j = 0; j < master.number_of_servos; j++)
    {
        regulator_ptr[j]->insert_algorithm_no(command.parameters.servo_alg_par.servo_algorithm_no[j]);
        regulator_ptr[j]->insert_algorithm_parameters_no(command.parameters.servo_alg_par.servo_parameters_no[j]);
    }
    reply_to_EDP_MASTER();
}
/*-----------------------------------------------------------------------*/



// regulator


/*-----------------------------------------------------------------------*/
regulator::regulator(uint8_t reg_no, uint8_t reg_par_no, common::manip_and_conv_effector &_master)
        : master(_master)
{
    // Konstruktor abstrakcyjnego regulatora
    // Inicjuje zmienne, ktore kazdy regulator konkretny musi miec i aktualizowac,
    // niezaleznie od tego czy sa mu niezbedne, czy nie.

    algorithm_no = reg_no;                         // przeslany numer algorytmu
    current_algorithm_no = reg_no;                 // numer aktualnie uzywanego algorytmu
    algorithm_parameters_no = reg_par_no;          // przeslany numer zestawu parametrow algorytmu
    current_algorithm_parameters_no = reg_par_no;  // numer aktualnie uzywanego zestawu parametrow algorytmu

    position_increment_old = 0;  // przedostatnio odczytany przyrost polozenie (delta y[k-2] -- mierzone w impulsach)
    position_increment_new = 0;  // ostatnio odczytany przyrost polozenie (delta y[k-1] -- mierzone w impulsach)
    step_old_pulse = 0;                // poprzednia wartosc zadana dla jednego kroku regulacji
    // (przyrost wartosci zadanej polozenia -- delta r[k-2] -- mierzone w radianach)
    step_old = 0.0;                // poprzednia wartosc zadana dla jednego kroku regulacji
    // (przyrost wartosci zadanej polozenia -- delta r[k-2] -- mierzone w radianach)

    step_new = 0;                // nastepna wartosc zadana dla jednego kroku regulacji
    // (przyrost wartosci zadanej polozenia -- delta r[k-1] -- mierzone w radianach)
    set_value_new = 0;           // wielkosc kroku do realizacji przez HIP (wypelnienie PWM -- u[k])
    set_value_old = 0;           // wielkosc kroku do realizacji przez HIP (wypelnienie PWM -- u[k-1])
    set_value_very_old = 0;      // wielkosc kroku do realizacji przez HIP (wypelnienie PWM -- u[k-2])

    delta_eint = 0.0;            // przyrost calki uchybu
    delta_eint_old = 0.0;        // przyrost calki uchybu w poprzednim kroku
    pos_increment_new_sum = 0;   // skumulowany przyrost odczytanego polozenia w trakcie realizacji makrokroku
    servo_pos_increment_new_sum =0;// by Y

    step_new_over_constraint_sum = 0.0;
    previous_abs_position = 0.0;

    meassured_current = 0;                 // prad zmierzony
    PWM_value = 0;               // zadane wypelnienie PWM
}
/*-----------------------------------------------------------------------*/



// BY Y i S - uwzglednie ograniczen na predkosc i przyspieszenie regulatorow  - stara wersja
/*
void regulator::constraint_detector(double max_acc_local, double max_vel_local, double max_diff_local, bool debug)
{
	double step_new_over_constraint_sum_tmp;
	double step_new_tmp = step_new;
	// przyspieszenie i roznica predkosci
	double current_acc, vel_diff;

	//	if (debug) printf("step_new: %f, step_new_over_constraint_sum: %f\n", step_new, step_new_over_constraint_sum);
	step_new += step_new_over_constraint_sum;

	step_new_over_constraint_sum_tmp = step_new;

	// ograniczenie na przekroczenie step_new (aby predkosc biezaca nie odbiegala zby mocno od zadanej,
	// co zmniejsza przeregulowania i oscylacje)
	vel_diff = step_new - step_new_tmp;
	if (vel_diff > max_diff_local) step_new = step_new_tmp + max_diff_local;
	if (vel_diff < -max_diff_local) step_new = step_new_tmp - max_diff_local;

	// sprawdzenie ograniczenia na predkosc (co do predkosci maksymalnej)
	if (step_new > max_vel_local) step_new = max_vel_local;
	if (step_new < -max_vel_local) step_new = -max_vel_local;

	// sprawdzenie ograniczenia na przyspieszenie
	current_acc = step_new - step_old;
	if (current_acc > max_acc_local) step_new = step_old + max_acc_local;
	if (current_acc < -max_acc_local) step_new = step_old - max_acc_local;

	step_new_over_constraint_sum = step_new_over_constraint_sum_tmp - step_new;

	//	if (debug) printf("acc: %f, vel: %f, const_sum: %f\n", current_acc, step_new, step_new_over_constraint_sum);

	step_old = step_new;
}
*/

// BY Y i S - uwzglednie ograniczen na predkosc i przyspieszenie regulatorow

void regulator::constraint_detector(double max_acc_local, double max_vel_local, bool debug)
{

    double sum_min = 0.0, sum_mid = 0.0, sum_max = 0.0;

    double step_new_beginning = step_new;

    double step_diff = step_old - step_new;
    double abs_step_diff = fabs (step_diff);

    // zliczanie minimalnej, dodatkowej drogi przebytej do czasu zrownania predkosci biezacej z zadana -
    // suma ciagu arytmetycznego dla wariantu minimalnego
    double abs_step_diff_min = abs_step_diff - max_acc_local;
    if (abs_step_diff_min < 0.0 )
        abs_step_diff_min = 0.0;

    double first_elem = abs_step_diff_min;
    int step_no = (int) (floor (abs_step_diff_min / max_acc_local));
    double last_elem = abs_step_diff_min - step_no * max_acc_local;

    sum_min = ((first_elem + last_elem) * step_no) / 2;
    if (step_diff < 0)
        sum_min = -sum_min;

    // liczenie sumy sredniej i maksymalnej
    sum_mid = sum_min + step_diff;

    if (step_diff >= 0.0)
        sum_max = sum_mid + step_diff + max_acc_local;
    else if (step_diff < 0.0)
        sum_max = sum_mid + step_diff - max_acc_local;

    // podjecie decyzji o zmianie predkosci
    // jezeli jestesmy dostatecznie blisko zadanej trajektorii
    if ((abs_step_diff <= max_acc_local) && (fabs(step_new_over_constraint_sum) <= max_acc_local))
    {
        step_new = step_new_beginning + step_new_over_constraint_sum;
    }
    else
    {
        if (step_diff >= 0.0)
        {
            if (step_new_over_constraint_sum > sum_max)
            {
                step_new = step_old + max_acc_local;
            }
            else if (step_new_over_constraint_sum >= sum_mid)
            {
                step_new = step_old;
            }
            else if (step_new_over_constraint_sum >= sum_min)
            {
                step_new = step_old - max_acc_local;
            }
            else
            {
                step_new = step_old - max_acc_local;
            }
        }
        else // 	if (step_diff >= 0.0)
            if (step_new_over_constraint_sum < sum_max)
            {
                step_new = step_old - max_acc_local;
            }
            else if (step_new_over_constraint_sum <= sum_mid)
            {
                step_new = step_old;
            }
            else if (step_new_over_constraint_sum <= sum_min)
            {
                step_new = step_old + max_acc_local;
            }
            else
            {
                step_new = step_old + max_acc_local;
            }
    }

    // sprawdzenie ograniczenia na predkosc (co do predkosci maksymalnej)
    if (step_new > max_vel_local)
        step_new = max_vel_local;
    if (step_new < -max_vel_local)
        step_new = -max_vel_local;


    step_new_over_constraint_sum = step_new_over_constraint_sum + step_new_beginning - step_new;

    if (debug)
        printf("sn: %f, snb: %f, sum: %f\n", step_new, step_new_beginning, step_new_over_constraint_sum);

    step_old = step_new;
}




double regulator::get_set_value ( void ) const
{
    // odczytanie aktualnej wartosci zadanej  - metoda konkretna
    return set_value_new;
}

void regulator::insert_new_step (double ns)
{
    // wstawienie nowej wartosci zadanej - metoda konkretna
    step_new = ns;
}

void regulator::insert_meassured_current (int meassured_current_l)
{
    // wstawienie wartosci zmierzonej pradu
    meassured_current = meassured_current_l;
}


double regulator::return_new_step ( void ) const
{
	// wstawienie nowej wartosci zadanej - metoda konkretna
	return step_new;
}

void regulator::insert_new_pos_increment (double inc)
{
    // wstawienie nowej wartosci odczytanej przyrostu polozenia - metoda konkretna
    // Do przepisywania zrealizowanego polozenia z HI do regulatora
    position_increment_new = inc;
}

double regulator::get_position_inc ( int tryb )
{ // by Y: 0 dla servo i 1 dla paczki dla edp;
    // odczytanie zrealizowanego przyrostu polozenia w makrokroku - metoda konkretna
    double pins;
    if (tryb==1)
    {
        pins = pos_increment_new_sum;
        pos_increment_new_sum = 0.0;
    }
    else if (tryb==0)
    {
        pins = servo_pos_increment_new_sum;
        servo_pos_increment_new_sum = 0.0;
    }
    return pins;
}

int regulator::get_meassured_current ( void ) const
{
    // odczytanie rzeczywistego pradu - metoda konkretna
    return meassured_current;
}

int regulator::get_PWM_value ( void ) const
{
    // odczytanie zadanego wypelnienia PWM - metoda abstrakcyjna
    return PWM_value;
}

// do odczytu stanu regulatora (w szczegolnosci regulatora chwytaka)
int regulator::get_reg_state ( void ) const
{
    // odczytanie zadanego wypelnienia PWM - metoda abstrakcyjna
    return reg_state;
}

int regulator::get_actual_inc ( void ) const
{
    // odczytanie rzeczywistego przyrostu polozenia w pojedynczym kroku
    return (int) position_increment_new;
}

// double get_desired_inc ( int axe_nr );


void regulator::insert_algorithm_no ( uint8_t new_number )
{
    // wpisanie nowego numeru algorytmu regulacji
    algorithm_no = new_number;
}

uint8_t regulator::get_algorithm_no ( void ) const
{
    // odczytanie aktualnie uzywanego numeru algorytmu regulacji
    return current_algorithm_no;
}

void regulator::insert_algorithm_parameters_no ( uint8_t new_number )
{
    // wpisanie nowego numeru zestawu parametrow algorytmu regulacji
    algorithm_parameters_no = new_number;
}

uint8_t regulator::get_algorithm_parameters_no ( void ) const
{
    // wpisanie nowego numeru zestawu parametrow algorytmu regulacji
    return current_algorithm_parameters_no;
}

void regulator::clear_regulator ()

{
    // zerowanie wszystkich zmiennych regulatora
    position_increment_old = 0.0;
    position_increment_new = 0.0;
    pos_increment_new_sum = 0.0;
    servo_pos_increment_new_sum = 0.0;
    step_old_pulse = 0.0;
    step_new = 0.0;
    set_value_new = 0.0;
    set_value_old = 0.0;
    set_value_very_old = 0.0;
    delta_eint = 0.0;
    delta_eint_old = 0.0;
}




/*-----------------------------------------------------------------------*/
NL_regulator::NL_regulator (uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::manip_and_conv_effector &_master)
        : regulator(reg_no, reg_par_no, _master)
{
    // Konstruktor regulatora konkretnego
    // Przy inicjacji nalezy dopilnowac, zeby numery algorytmu regulacji oraz zestawu jego parametrow byly
    // zgodne z faktycznie przekazywanym zestawem parametrow inicjujacych.

    a = aa;                      // parametr regulatora
    b0 = bb0;                    // parametr regulatora
    b1 = bb1;                    // parametr regulatora
    k_feedforward = k_ff;        // wspolczynnik wzmocnienia w petli "feedforward"

    EPS = 1.0e-10;
    MAX_PWM = 190; // Maksymalne wypelnienie PWM dla robota IRp-6 na torze
    integrator_off = 6;
    counter = 0;

    meassured_current = 0;
}
/*-----------------------------------------------------------------------*/

} // namespace common
} // namespace edp
} // namespace mrrocpp

