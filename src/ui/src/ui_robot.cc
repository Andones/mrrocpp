/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */
#include "ui/ui_robot.h"
#include "ui/ui_ecp.h"
#include "ui/ui_class.h"

extern Ui ui;

extern ui_state_def ui_state;

//
//
// KLASA UiRobot
//
//


UiRobot::UiRobot() :
	tid(NULL) {
	state.edp.state = -1; // edp nieaktywne
	state.edp.last_state = -1; // edp nieaktywne
	state.ecp.trigger_fd = -1;

}

void UiRobot::create_thread() {
	tid = new feb_thread(eb);
}

void UiRobot::abort_thread() {
	delete tid;
}
