#include "ui_r_smb.h"
#include "ui_ecp_r_smb.h"
#include "robot/smb/const_smb.h"
#include "../base/interface.h"

#include "../base/mainwindow.h"
#include "ui_mainwindow.h"

#include "../base/signal_dispatcher.h"

#include "../base/menu_bar.h"
#include "../base/menu_bar_action.h"

namespace mrrocpp {
namespace ui {
namespace smb {

//
//
// KLASA UiRobotIrp6ot_m
//
//


int UiRobot::ui_get_edp_pid()
{
	return ui_ecp_robot->the_robot->get_EDP_pid();
}

void UiRobot::ui_get_controler_state(lib::controller_state_t & robot_controller_initial_state_l)
{
	ui_ecp_robot->get_controller_state(robot_controller_initial_state_l);

}

int UiRobot::synchronise()

{

	return 1;

}

UiRobot::UiRobot(common::Interface& _interface, lib::robot_name_t _robot_name) :
	common::UiRobot(_interface, _robot_name, lib::smb::NUM_OF_SERVOS), ui_ecp_robot(NULL)
{

}

int UiRobot::manage_interface()
{
	MainWindow *mw = interface.get_main_window();

	switch (state.edp.state)
	{
		case -1:
			mw->enable_menu_item(false, 1, menuSmb);
			/* TR
			 ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_smb, NULL);
			 */
			break;
		case 0:
			mw->enable_menu_item(false, 1, actionsmb_EDP_Unload);
			mw->enable_menu_item(true, 1, menuSmb);
			mw->enable_menu_item(true, 1, actionsmb_EDP_Load);
			/* TR
			 ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_smb_edp_unload,

			 NULL);
			 ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_smb, ABN_mm_smb_edp_load, NULL);
			 */
			break;
		case 1:
		case 2:
			mw->enable_menu_item(true, 1, menuSmb);
			/* TR
			 ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_smb, NULL);
			 */
			// jesli robot jest zsynchronizowany
			if (state.edp.is_synchronised) {
				mw->enable_menu_item(true, 1, mw->getMenuBar()->menuall_Preset_Positions);
				/* TR
				 ApModifyItemState(&robot_menu, AB_ITEM_DIM, NULL);
				 ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_preset_positions, NULL);
				 */
				switch (interface.mp.state)
				{
					case common::UI_MP_NOT_PERMITED_TO_RUN:
					case common::UI_MP_PERMITED_TO_RUN:
						mw->enable_menu_item(true, 1, actionsmb_EDP_Unload);
						mw->enable_menu_item(false, 1, actionsmb_EDP_Load);
						block_ecp_trigger();
						/* TR
						 ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_smb_edp_unload, NULL);
						 ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_smb_edp_load, NULL);
						 */
						break;
					case common::UI_MP_WAITING_FOR_START_PULSE:
						mw->enable_menu_item(false, 2, actionsmb_EDP_Unload, actionsmb_EDP_Load);
						block_ecp_trigger();
						/* TR
						 ApModifyItemState(&robot_menu, AB_ITEM_NORMAL,

						 NULL);
						 ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_smb_edp_load, ABN_mm_smb_edp_unload, NULL);
						 */
						break;
					case common::UI_MP_TASK_RUNNING:
						unblock_ecp_trigger();
						break;
					case common::UI_MP_TASK_PAUSED:
						block_ecp_trigger();
						/* TR
						 ApModifyItemState(&robot_menu, AB_ITEM_DIM, // modyfikacja menu - ruchy reczne zakazane

						 NULL);
						 */
						break;
					default:
						break;
				}
			} else // jesli robot jest niezsynchronizowany
			{
				mw->enable_menu_item(true, 1, actionsmb_EDP_Unload);
				mw->enable_menu_item(false, 1, actionsmb_EDP_Load);
				/* TR
				 ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_smb_edp_unload, NULL);
				 ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_smb_edp_load, NULL);
				 ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_synchronisation, NULL);
				 */
			}
			break;
		default:
			break;
	}

	return 1;
}

void UiRobot::make_connections()
{
	Ui::SignalDispatcher *signalDispatcher = interface.get_main_window()->getSignalDispatcher();

	connect(actionsmb_EDP_Load, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_EDP_Load_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
	connect(actionsmb_EDP_Unload, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_EDP_Unload_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
}

void UiRobot::setup_menubar()
{
	Ui::MenuBar *menuBar = interface.get_main_window()->getMenuBar();

	actionsmb_EDP_Load = new Ui::MenuBarAction(QString("EDP &Load"), this, menuBar);
	actionsmb_EDP_Unload = new Ui::MenuBarAction(QString("EDP &Unload"), this, menuBar);

	menuSmb = new QMenu(menuBar->menuRobot);
	menuSmb->setObjectName(QString::fromUtf8("menuSmb"));
	menuSmb->addAction(actionsmb_EDP_Load);
	menuSmb->addAction(actionsmb_EDP_Unload);

	menuBar->menuRobot->addAction(menuSmb->menuAction());

	actionsmb_EDP_Load->setText(QApplication::translate("MainWindow", "EDP &Load", 0, QApplication::UnicodeUTF8));
	actionsmb_EDP_Unload->setText(QApplication::translate("MainWindow", "EDP &Unload", 0, QApplication::UnicodeUTF8));
	menuSmb->setTitle(QApplication::translate("MainWindow", "S&mb", 0, QApplication::UnicodeUTF8));
}

void UiRobot::delete_ui_ecp_robot()
{
	delete ui_ecp_robot;
}

void UiRobot::null_ui_ecp_robot()
{
	ui_ecp_robot = NULL;
}

}
} //namespace ui
} //namespace mrrocpp


