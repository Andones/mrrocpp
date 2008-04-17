// -------------------------------------------------------------------------
//                            vsp.h		dla QNX6.2
//
// Definicje klasy edp_force_sensor
//
// Ostatnia modyfikacja: 2005
// Autor: yoyek
// -------------------------------------------------------------------------

#if !defined(_EDP_FORCE_SENSOR_H)
#define _EDP_FORCE_SENSOR_H

#include "lib/ForceTrans.h"
#include "common/sensor.h"				// klasa bazowa sensor
#include "edp/common/edp.h"				// klasa bazowa sensor

#ifdef __cplusplus
extern "C"
{
#endif

    /********** klasa czujnikow po stronie EDP **************/
    class edp_force_sensor : public sensor
    {

    protected:


        short is_reading_ready;			// czy jakikolwiek odczyt jest gotowy?

        ForceTrans *gravity_transformation; // klasa likwidujaca wplyw grawitacji na czujnik

    public:
        sr_vsp *sr_msg;		//!< komunikacja z SR
        sem_t new_ms; //!< semafor dostepu do nowej wiadomosci dla vsp
        sem_t new_ms_for_edp; //!< semafor dostepu do nowej wiadomosci dla edp
        bool TERMINATE;			//!< zakonczenie obydwu watkow
        short is_sensor_configured;		// czy czujnik skonfigurowany?
        bool first_configure_done;
        int	set_command_execution_finish() ;
        int	check_for_command_execution_finish();

        double next_force_tool_position[3];
        double next_force_tool_weight;
        double current_force_tool_position[3];
        double current_force_tool_weight;

		bool new_edp_command;
        bool force_sensor_do_configure; // FLAGA ZLECENIA KONFIGURACJI CZUJNIKA
        bool force_sensor_do_first_configure; // pierwsza konfiguracja po synchronizacji lub uruchomieniu
        bool force_sensor_set_tool; // FLAGA ZLECENIA ZMIANY NARZEDZIA

        edp_irp6s_postument_track_effector &master;
        edp_force_sensor(edp_irp6s_postument_track_effector &_master);

        virtual void wait_for_event(void);			// oczekiwanie na zdarzenie
        void set_force_tool (void);

    }
    ; // end: class edp_force_sensor


    // Zwrocenie stworzonego obiektu - czujnika. Funkcja implementowana w plikach klas dziedziczacych.
    edp_force_sensor* return_created_edp_force_sensor (edp_irp6s_postument_track_effector &_master);


#ifdef __cplusplus
};
#endif
#endif
