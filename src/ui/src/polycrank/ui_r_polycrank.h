// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_POLYCRANK_H
#define __UI_R_POLYCRANK_H

#include "ui/src/ui.h"
#include "ui/src/ui_robot.h"

namespace mrrocpp {
namespace uin {
namespace common {
class Ui;
}

namespace irp6 {
class ui_irp6_common_robot;
}
namespace polycrank {

//
//
// KLASA UiRobotIrp6ot_m
//
//


class UiRobot : public common::UiRobot
{
private:

public:

	bool is_wind_polycrank_int_open; // informacja czy okno ruchow w radianach stawow jest otwarte
	bool is_wind_polycrank_inc_open; // informacja czy okno ruchow w radianach na wale silnika jest otwarte


	irp6::ui_irp6_common_robot *ui_ecp_robot;

	UiRobot(common::Ui& _ui);
	int reload_configuration();
	int manage_interface();
	int delete_ui_ecp_robot();

};

}
} //namespace uin
} //namespace mrrocpp

#endif

