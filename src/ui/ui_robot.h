// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_ROBOT_H
#define __UI_ROBOT_H

#include "ui/ui.h"

class Ui;

//
//
// KLASA UiRobot
//
//


// super klasa agregujaca porozrzucane dotychczas struktury


class UiRobot {
private:

public:
	Ui& ui;
	feb_thread* tid;
	function_execution_buffer eb;

	ecp_edp_ui_robot_def state;

	UiRobot(Ui& _ui, const std::string edp_section_name,
			const std::string ecp_section_name);
	virtual int reload_configuration()= 0;
	void create_thread();
	void abort_thread();
	bool pulse_reader_start_exec_pulse(void);
	bool pulse_reader_stop_exec_pulse(void);
	bool pulse_reader_trigger_exec_pulse(void);

};

#endif

