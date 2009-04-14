#if !defined(_ECP_TASK_H)
#define _ECP_TASK_H

#include "ecp_mp/ecp_mp_task.h"
#include "ecp/common/ecp_robot.h"

#include <libxml/tree.h>

#include <map>
#include "mp/Trajectory.h"


namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {


class base;

// klasa globalna dla calego procesu MP
class base : public ecp_mp::task::base
{
	private:
		name_attach_t *ecp_attach, *trigger_attach; // by Y

		int MP_fd;
		// Wysyla puls do Mp przed oczekiwaniem na spotkanie
		void send_pulse_to_mp(int pulse_code, int pulse_value);

		// Receive of mp message
		int receive_mp_message(void);

		// Badanie typu polecenia z MP
		lib::MP_COMMAND mp_command_type(void) const;

	protected:
		// Oczekiwanie na nowy stan od MP
		void get_next_state(void);

	public: // TODO: following packages should be 'protected'
		// Odpowiedz ECP do MP, pola do ew. wypelnienia przez generatory
		lib::ECP_REPLY_PACKAGE ecp_reply;

		// Polecenie od MP dla TASKa
		lib::MP_COMMAND_PACKAGE mp_command;

	public:
		ecp_robot* ecp_m_robot;

		// sprawdza czy przeszedl puls do ECP lub MP
		bool pulse_check();

		// KONSTRUKTOR
		base(lib::configurator &_config);

		// dla gcc: `'class Foo' has virtual functions but non-virtualdestructor` warning.
		virtual ~base();

		void initialize_communication(void);

		// obsluga sygnalu
		virtual void catch_signal_in_ecp_task(int sig);

		virtual void terminate();

		// methods for ECP template to redefine in concrete classes
		virtual void task_initialization(void);
		virtual void main_task_algorithm(void);

		// Informacja dla MP o zakonczeniu zadania uzytkownika
		void ecp_termination_notice(void);

		// Oczekiwanie na polecenie START od MP
		bool ecp_wait_for_start(void);

		// Oczekiwanie na STOP
		void ecp_wait_for_stop(void);

		// funkcjonalnosc dodana na potrzeby czytania trajektorii z pliku xml
		struct str_cmp{
			bool operator()(char const *a, char const *b) const;
		};
		mp::common::Trajectory * createTrajectory(xmlNode *actNode, xmlChar *stateID);
		std::map<char*, mp::common::Trajectory, str_cmp>* loadTrajectories(char * fileName, ROBOT_ENUM propRobot);

	public: // TODO: what follows should be private method

		// Oczekiwanie na polecenie od MP
		bool mp_buffer_receive_and_send(void);

		// Ustawienie typu odpowiedzi z ECP do MP
		void set_ecp_reply(lib::ECP_REPLY ecp_r);
};

base* return_created_ecp_task (lib::configurator &_config);


// klasa podzadania
class ecp_sub_task
{
	protected:
		base &ecp_t;

	public:
		ecp_sub_task(base &_ecp_t);
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_TASK_H */
