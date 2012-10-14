// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_COMMON_012_H
#define __UI_R_COMMON_012_H

#include "../base/ui.h"
#include "../base/ui_robot.h"
#include "robot/conveyor/const_conveyor.h"

class wgt_single_motor_move;
class wgt_kinematic;

namespace Ui {
class MenuBar;
class MenuBarAction;
}

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
namespace common_012 {

class EcpRobot;

class UiRobot : public common::UiRobot
{
private:

public:

	common_012::EcpRobot *ui_ecp_robot;

	UiRobot(common::Interface& _interface, lib::robot_name_t _robot_name, int _number_of_servos);

	void setup_menubar();
	void manage_interface();
	virtual void unsynchronise();

protected:

	QMenu *menu_Pre_Synchro_Moves;

private:
	QMenu *menu_Preset_Positions;
	QMenu *menu_Special;

	QAction *action_UnSynchronisation;
	QAction *action_Synchronisation;
	QAction *action_Kinematics;
	QAction *action_Servo_Algorithm;

	QAction *action_Synchro_Position;
	QAction *action_Front_Position;
	QAction *action_Position_0;
	QAction *action_Position_1;
	QAction *action_Position_2;

};

}
} //namespace ui
} //namespace mrrocpp

#endif

