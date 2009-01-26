// -------------------------------------------------------------------------
//                            ui.h
// Definicje struktur danych i metod dla procesu UI
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __UI_H
#define __UI_H

#include <semaphore.h>
#include <pthread.h>
#include <list>

#include "common/com_buf.h"
#include "lib/srlib.h"


#define CATCH_SECTION_UI catch (ecp_robot::ECP_main_error e) { \
	/* Obsluga bledow ECP */ \
	if (e.error_class == SYSTEM_ERROR) \
		printf("ECP SYSTEM_ERROR error in UI\n"); \
		ui_state.ui_state=2; \
	/*  exit(EXIT_FAILURE);*/ \
  } /*end: catch */ \
\
catch (ecp_robot::ECP_error er) { \
	/* Wylapywanie bledow generowanych przez modul transmisji danych do EDP */ \
	if ( er.error_class == SYSTEM_ERROR) { /* blad systemowy juz wyslano komunikat do SR */ \
		perror("ECP SYSTEM_ERROR in UI\n"); \
		/* PtExit( EXIT_SUCCESS ); */ \
	} else { \
	switch ( er.error_no ) { \
		case INVALID_POSE_SPECIFICATION: \
		case INVALID_ECP_COMMAND: \
		case INVALID_COMMAND_TO_EDP: \
		case EDP_ERROR: \
		case INVALID_EDP_REPLY: \
		case INVALID_RMODEL_TYPE: \
			/* Komunikat o bledzie wysylamy do SR */ \
			ui_msg.all_ecp->message (NON_FATAL_ERROR, er.error_no); \
		break; \
		default: \
			ui_msg.all_ecp->message (NON_FATAL_ERROR, 0, "ECP: Unidentified exception"); \
			perror("Unidentified exception"); \
		} /* end: switch */ \
	} \
} /* end: catch */ \
\
catch (...) {  /* Dla zewnetrznej petli try*/ \
	/* Wylapywanie niezdefiniowanych bledow*/ \
	/*Komunikat o bledzie wysylamy do SR*/ \
	printf("uneidentified error in UI\n"); \
} /*end: catch */\


enum TEACHING_STATE_ENUM
{
	FSTRAJECTORY, FSCONFIG
};

enum UI_NOTIFICATION_STATE_ENUM
{
	UI_N_STARTING, UI_N_READY, UI_N_BUSY, UI_N_EXITING, 	UI_N_COMMUNICATION, UI_N_PROCESS_CREATION,
	UI_N_SYNCHRONISATION
};

// FIXME: moved from proto.h for linux compatibility
int set_ui_state_notification ( UI_NOTIFICATION_STATE_ENUM new_notifacion );

enum UI_ECP_COMMUNICATION_STATE
{
	UI_ECP_AFTER_RECEIVE, UI_ECP_REPLY_READY, UI_ECP_AFTER_REPLY
};

enum UI_MP_STATE
{
	UI_MP_NOT_PERMITED_TO_RUN, UI_MP_PERMITED_TO_RUN, UI_MP_WAITING_FOR_START_PULSE, UI_MP_TASK_RUNNING,
	UI_MP_TASK_PAUSED
};

enum UI_ALL_EDPS_STATE
{
	UI_ALL_EDPS_NONE_EDP_ACTIVATED, UI_ALL_EDPS_NONE_EDP_LOADED,
		UI_ALL_EDPS_THERE_IS_EDP_LOADED_BUT_NOT_ALL_ARE_LOADED,
		UI_ALL_EDPS_LOADED_BUT_NOT_SYNCHRONISED, UI_ALL_EDPS_LOADED_AND_SYNCHRONISED
};


 // -1 mp jest wylaczone i nie moze zostac wlaczone , 0 - mp wylaczone ale wszystkie edp gotowe,  1- wlaczone czeka na start
				// 2 - wlaczone czeka na stop 3 -wlaczone czeka na resume


// czas jaki uplywa przed wyslaniem sygnalu w funkcji ualarm w mikrosekundach
#define SIGALRM_TIMEOUT 1000000


typedef struct{
	pid_t pid;
	int test_mode;
	char* node_name;
	char section_name[50]; // nazwa sekcji, w ktorej zapisana jest konfiguracja
	char* network_resourceman_attach_point;
	char* hardware_busy_attach_point; // do sprawdzenie czy edp juz nie istnieje o ile nie jest tryb testowy
	char* network_reader_attach_point;
	int node_nr;
	int reader_fd;
	bool is_synchronised;
	int state; // -1, edp nie aktywne, 0 - edp wylaczone 1- wlaczone czeka na reader start 2 - wlaczone czeka na reader stop
	int last_state;
	char* preset_sound_0; // dla EDP speaker
	char* preset_sound_1;
	char* preset_sound_2;
	double preset_position[3][8]; // pozycje zapisane w konfiguracji
	} edp_state_def;


typedef struct{
	pid_t pid;
	char* node_name;
	char section_name[50]; // nazwa sekcji, w ktorej zapisana jest konfiguracja
	char* network_trigger_attach_point;
	int node_nr;
	int trigger_fd;
	int state;
	int last_state;
	} ecp_state_def;


typedef struct{
	bool is_active;
	edp_state_def edp;
	ecp_state_def ecp;
	} ecp_edp_ui_robot_def;


typedef struct{
	pid_t pid;
	char* node_name;
	char* network_pulse_attach_point;
	int node_nr;
	int pulse_fd;
	UI_MP_STATE state;
	UI_MP_STATE last_state;
	} mp_state_def;


typedef struct{
	char* program_name;
	char* node_name;
	} program_node_def;


typedef struct {

	UI_ALL_EDPS_STATE all_edps;
	char binaries_network_path[100]; // sieciowa sciezka binariow mrrocpp
	char binaries_local_path[100]; // lokalna sciezka binariow mrrocpp
	char mrrocpp_local_path[100]; // lokalna sciezka mrrocpp: np. "/home/yoyek/mrrocpp/". W niej katalogi bin, configs etc.

	char teach_filesel_fullpath[100]; // sciezka domyslana dla fileselect dla generatora uczacego
	char config_file[30];// nazwa pliku konfiguracyjnego dla UI
	char session_name[20]; // nazwa sesji
	char config_file_fullpath[100]; // sciezka globalna do konfiguracji
	char config_file_relativepath[100]; // sciezka lokalana do konfiguracji wraz z plikiem konfiguracyjnym

	char* ui_attach_point;
	char* network_sr_attach_point;
	char* sr_attach_point;

	// listy sekcji i wezlow sieciowych plikow konfiguracyjnych
	std::list<char*> section_list, config_node_list, all_node_list;
	// lista nazw programow i wezlow na ktorych maja byc uruchamiane
	std::list<program_node_def> program_node_list;

	char ui_node_name[30]; // nazwa wezla na ktorym jest uruchamiany UI
	int	ui_node_nr; // numer wezla na ktorym jest uruchamiany UI
	pid_t ui_pid; // pid UI
	short ui_state; // 1 working, 2 exiting started, 3-5 exiting in progress - mrrocpp processes closing, 6 - exit imeditily

	ecp_edp_ui_robot_def irp6_on_track;
	ecp_edp_ui_robot_def irp6_postument;
	ecp_edp_ui_robot_def irp6_mechatronika;
	ecp_edp_ui_robot_def conveyor;
	ecp_edp_ui_robot_def speaker;

	mp_state_def mp;
	// bool is_any_edp_active;
	bool is_mp_and_ecps_active;

	int teachingstate; // dawne systemState do nauki
	TEACHING_STATE_ENUM file_window_mode;
	UI_NOTIFICATION_STATE_ENUM notification_state;

	bool is_task_window_open; // informacja czy okno zadania jest otwarte
	bool is_process_control_window_open; // informacja czy okno sterowania procesami jest otwarte
	bool process_control_window_renew; // czy okno ma zostac odswierzone

	bool is_wind_irp6ot_int_open; // informacja czy okno ruchow w radianach stawow jest otwarte
	bool is_wind_irp6p_int_open; // informacja czy okno ruchow w radianach stawow jest otwarte
	bool is_wind_irp6m_int_open; // informacja czy okno ruchow w radianach stawow jest otwarte

	bool is_wind_irp6ot_inc_open; // informacja czy okno ruchow w radianach na wale silnika jest otwarte
	bool is_wind_irp6p_inc_open;  // informacja czy okno ruchow w radianach na wale silnika jest otwarte
	bool is_wind_irp6m_inc_open;  // informacja czy okno ruchow w radianach na wale silnika jest otwarte

	bool is_wind_irp6ot_xyz_euler_zyz_open;  // informacja czy okno ruchow we wspolrzednych zewnetrznych jest otwarte
	bool is_wind_irp6p_xyz_euler_zyz_open;  // informacja czy okno ruchow we wspolrzednych zewnetrznych jest otwarte
	bool is_wind_irp6m_xyz_euler_zyz_open;  // informacja czy okno ruchow we wspolrzednych zewnetrznych jest otwarte

	bool is_wind_irp6ot_xyz_angle_axis_open;  // informacja czy okno ruchow we wspolrzednych zewnetrznych jest otwarte
	bool is_wind_irp6p_xyz_angle_axis_open;  // informacja czy okno ruchow we wspolrzednych zewnetrznych jest otwarte
	bool is_wind_irp6m_xyz_angle_axis_open;  // informacja czy okno ruchow we wspolrzednych zewnetrznych jest otwarte

	bool is_wind_irp6ot_xyz_angle_axis_ts_open;  // informacja czy okno definicji narzedzia we wspolrzednych zewnetrznych jest otwarte
	bool is_wind_irp6p_xyz_angle_axis_ts_open;  // informacja czy okno definicji narzedzia we wspolrzednych zewnetrznych jest otwarte
	bool is_wind_irp6m_xyz_angle_axis_ts_open;  // informacja czy okno definicji narzedzia we wspolrzednych zewnetrznych jest otwarte

	bool is_wind_irp6ot_xyz_euler_zyz_ts_open;  // informacja czy okno definicji narzedzia we wspolrzednych zewnetrznych jest otwarte
	bool is_wind_irp6p_xyz_euler_zyz_ts_open;  // informacja czy okno definicji narzedzia we wspolrzednych zewnetrznych jest otwarte
	bool is_wind_irp6m_xyz_euler_zyz_ts_open;  // informacja czy okno definicji narzedzia we wspolrzednych zewnetrznych jest otwarte

	bool is_wind_irp6ot_kinematic_open;  // informacja czy okno definicji kinematyki jest otwarte
	bool is_wind_irp6p_kinematic_open;  // informacja czy okno definicji kinematyki jest otwarte
	bool is_wind_irp6m_kinematic_open;  // informacja czy okno definicji kinematyki jest otwarte

	bool is_wind_irp6ot_servo_algorithm_open;  // informacja czy okno definicji kinematyki jest otwarte
	bool is_wind_irp6p_servo_algorithm_open;  // informacja czy okno definicji kinematyki jest otwarte
	bool is_wind_irp6m_servo_algorithm_open;  // informacja czy okno definicji kinematyki jest otwarte
	bool is_wind_conv_servo_algorithm_open;  // informacja czy okno definicji kinematyki jest otwarte

	bool is_wind_conveyor_moves_open;  // informacja czy okno ruchow dla robota conveyor

	bool is_wind_speaker_play_open; // informacja czy okno odtwarzania dzwiekow jest otwarte

	bool is_teaching_window_open; // informacja czy okno nauki jest otwarte
	bool is_file_selection_window_open; // informacja czy okno z wyborem pliku jest otwarte
} ui_state_def;


/**************************** ui_sr_buffer *****************************/

#define UI_SR_BUFFER_LENGHT 50

class ui_sr_buffer {
private:
	sem_t sem;
	pthread_mutex_t mutex; // = PTHREAD_MUTEX_INITIALIZER ;

public:
	sr_package_t message_buffer[UI_SR_BUFFER_LENGHT];
	int writer_buf_position;
	int reader_buf_position;

	ui_sr_buffer();

	int	set_new_msg(); // podniesienie semafora
	int	check_new_msg(); // oczekiwanie na semafor
	int	lock_mutex(); // zajecie mutex'a
	int	unlock_mutex(); // zwolnienie mutex'a

};


/**************************** ui_sr_buffer *****************************/

#define UI_SR_BUFFER_LENGHT 50

class ui_ecp_buffer
{
private:
	sem_t sem;

public:
	UI_ECP_COMMUNICATION_STATE communication_state;
	ECP_message ecp_to_ui_msg;
	UI_reply ui_rep;

	ui_ecp_buffer();
	int	post_sem(); // podniesienie semafora
	int	take_sem(); // oczekiwanie na semafor
	int	trywait_sem(); // oczekiwanie na semafor
};


typedef struct
{
	sr_ecp* all_ecp;        // Wskaznik na obiekt do komunikacji z SR z fukcja ECP dla wszystkich robotow
	sr_ui* ui;              // Wskaznik na obiekt do komunikacji z SR
} ui_msg_def;

void UI_close(void);

#endif

