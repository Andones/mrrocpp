// -------------------------------------------------------------------------
//
// Definicje struktur danych i metod dla procesow MP
//
// -------------------------------------------------------------------------

#if !defined(__MP_G_TEACH_IN_H)
#define __MP_G_TEACH_IN_H

#if defined(__QNXNTO__)
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#endif

#include <map>
#include <list>

#include "ecp_mp/ecp_mp_task.h"
// Konfigurator.
#include "lib/configurator.h"
#include "lib/timer.h"



class mp_teach_in_generator : public mp_generator {

	protected:
		std::list<mp_taught_in_pose> pose_list;
		std::list<mp_taught_in_pose>::iterator pose_list_iterator;
#if !defined(USE_MESSIP_SRR)
		const int UI_fd;
#else
		messip_channel_t * const UI_fd;
#endif

	public:
		// -------------------------------------------------------
		// konstruktor
		mp_teach_in_generator(mp_task& _mp_task);

		// -------------------------------------------------------
		// destruktor
		~mp_teach_in_generator (void);
		// -------------------------------------------------------


		// --------------------------------------------------------------------------
		// Wczytanie trajektorii z pliku
		bool load_file ();

		// --------------------------------------------------------------------------
		// Wczytanie trajektorii z pliku
		bool load_file_with_path (const char* file_name, short robot_number);

		// --------------------------------------------------------------------------
		// Zapis trajektorii do pliku
		void save_file (POSE_SPECIFICATION ps);
		// --------------------------------------------------------------------------

		void flush_pose_list ( void );
		// -------------------------------------------------------
		void initiate_pose_list(void);;
		// -------------------------------------------------------
		void next_pose_list_ptr (void);
		// -------------------------------------------------------
		void get_pose (mp_taught_in_pose& tip);
		// -------------------------------------------------------
		// Pobierz nastepna pozycje z listy
		void get_next_pose (double next_pose[MAX_SERVOS_NR]);

		void get_next_pose (double next_pose[MAX_SERVOS_NR], double irp6_next_pose[MAX_SERVOS_NR]);

		// -------------------------------------------------------
		void set_pose (POSE_SPECIFICATION ps, double motion_time, double coordinates[MAX_SERVOS_NR]);

		void set_pose (POSE_SPECIFICATION ps, double motion_time, double coordinates[MAX_SERVOS_NR],
				double irp6p_coordinates[MAX_SERVOS_NR]);

		// -------------------------------------------------------
		bool is_pose_list_element ( void );
		// -------------------------------------------------------
		bool is_last_list_element ( void ) ;
		// -------------------------------------------------------

		void create_pose_list_head (POSE_SPECIFICATION ps, double motion_time, double coordinates[MAX_SERVOS_NR]);

		// by Y

		void create_pose_list_head (POSE_SPECIFICATION ps, double motion_time, double coordinates[MAX_SERVOS_NR],
				double irp6p_coordinates[MAX_SERVOS_NR]) ;


		void create_pose_list_head (POSE_SPECIFICATION ps, double motion_time, int extra_info, double coordinates[MAX_SERVOS_NR]);

		void insert_pose_list_element (POSE_SPECIFICATION ps, double motion_time, double coordinates[MAX_SERVOS_NR]);

		// by Y

		void insert_pose_list_element (POSE_SPECIFICATION ps, double motion_time, double coordinates[MAX_SERVOS_NR],
				double irp6p_coordinates[MAX_SERVOS_NR]);


		void insert_pose_list_element (POSE_SPECIFICATION ps, double motion_time, int extra_info, double coordinates[MAX_SERVOS_NR]);

		// -------------------------------------------------------
		int pose_list_length(void);
		// -------------------------------------------------------
		virtual bool first_step ();
		// generuje pierwszy krok ruchu -
		// pierwszy krok czesto rozni sie od pozostalych,
		// np. do jego generacji nie wykorzystuje sie czujnikow
		// (zadanie realizowane przez klase konkretna)
		virtual bool next_step ();
		// generuje kazdy nastepny krok ruchu
		// (zadanie realizowane przez klase konkretna)

}; // end: class mp_teach_in_generator


// ########################################################################################################
// ####################################    KONIEC GENERATOROW   ###########################################
// ########################################################################################################



#endif
