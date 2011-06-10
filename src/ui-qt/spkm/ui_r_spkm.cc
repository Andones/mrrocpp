/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui_r_spkm.h"
#include "ui_ecp_r_spkm.h"
#include "wgt_spkm_inc.h"
#include "wgt_spkm_int.h"
#include "wgt_spkm_ext.h"

#include "robot/spkm/const_spkm.h"
#include "../base/interface.h"

#include "../base/mainwindow.h"
#include "ui_mainwindow.h"

#include "../base/signal_dispatcher.h"

#include "../base/menu_bar.h"
#include "../base/menu_bar_action.h"

namespace mrrocpp {
namespace ui {
namespace spkm {

//
//
// KLASA UiRobotIrp6ot_m
//
//


UiRobot::UiRobot(common::Interface& _interface) :
	common::UiRobot(_interface, lib::spkm::ROBOT_NAME, lib::spkm::NUM_OF_SERVOS), ui_ecp_robot(NULL)
{

	wgt_inc = new wgt_spkm_inc(interface, *this, interface.get_main_window());
	wndbase_m[WGT_SPKM_INC] = wgt_inc->dwgt;

	wgt_int = new wgt_spkm_int(interface, *this, interface.get_main_window());
	wndbase_m[WGT_SPKM_INT] = wgt_int->dwgt;

	wgt_ext = new wgt_spkm_ext(interface, *this, interface.get_main_window());
	wndbase_m[WGT_SPKM_EXT] = wgt_ext->dwgt;

}

int UiRobot::ui_get_edp_pid()
{
	return ui_ecp_robot->the_robot->get_EDP_pid();
}

void UiRobot::ui_get_controler_state(lib::controller_state_t & robot_controller_initial_state_l)
{
	ui_ecp_robot->get_controller_state(robot_controller_initial_state_l);

}

int UiRobot::create_ui_ecp_robot()
{
	ui_ecp_robot = new ui::spkm::EcpRobot(*this);
	return 1;
}

int UiRobot::edp_create_int_extra_operations()
{
	wgt_inc->synchro_depended_init();
	return 1;
}

int UiRobot::synchronise()

{

	eb.command(boost::bind(&ui::spkm::UiRobot::synchronise_int, &(*this)));

	return 1;

}

int UiRobot::synchronise_int()

{

	interface.set_ui_state_notification(UI_N_SYNCHRONISATION);

	// wychwytania ew. bledow ECP::robot
	try {
		// dla robota spkm

		if ((state.edp.state > 0) && (state.edp.is_synchronised == false)) {
			ui_ecp_robot->the_robot->synchronise();
			state.edp.is_synchronised = ui_ecp_robot->the_robot->is_synchronised();
		} else {
			// 	printf("edp spkm niepowolane, synchronizacja niedozwolona\n");
		}

	} // end try
	CATCH_SECTION_IN_ROBOT

	// modyfikacje menu
	interface.manage_interface();
	wgt_inc->synchro_depended_init();

	return 1;

}

int UiRobot::manage_interface()
{
	MainWindow *mw = interface.get_main_window();

	switch (state.edp.state)
	{
		case -1:
			mw->enable_menu_item(false, 1, menuSpkm);
			break;
		case 0:
			mw->enable_menu_item(false, 1, actionspkm_EDP_Unload);
			mw->enable_menu_item(false, 1, actionspkm_Clear_Fault);
			mw->enable_menu_item(false, 3, menuspkm_Pre_synchro_moves, menuspkm_Preset_positions, menuspkm_Post_synchro_moves);
			mw->enable_menu_item(true, 1, menuSpkm);
			mw->enable_menu_item(true, 1, actionspkm_EDP_Load);

			break;
		case 1:
		case 2:
			mw->enable_menu_item(true, 1, menuSpkm);
			mw->enable_menu_item(true, 1, actionspkm_Clear_Fault);

			// jesli robot jest zsynchronizowany
			if (state.edp.is_synchronised) {
				mw->enable_menu_item(false, 1, menuspkm_Pre_synchro_moves);
				mw->enable_menu_item(true, 1, mw->getMenuBar()->menuall_Preset_Positions);
				switch (interface.mp.state)
				{
					case common::UI_MP_NOT_PERMITED_TO_RUN:
					case common::UI_MP_PERMITED_TO_RUN:
						mw->enable_menu_item(true, 2, menuspkm_Preset_positions, menuspkm_Post_synchro_moves);
						mw->enable_menu_item(true, 1, actionspkm_EDP_Unload); //???
						mw->enable_menu_item(false, 1, actionspkm_EDP_Load);
						break;
					case common::UI_MP_WAITING_FOR_START_PULSE:
						mw->enable_menu_item(true, 2, menuspkm_Preset_positions, menuspkm_Post_synchro_moves);//???
						mw->enable_menu_item(false, 2, actionspkm_EDP_Load, actionspkm_EDP_Unload);

						break;
					case common::UI_MP_TASK_RUNNING:
					case common::UI_MP_TASK_PAUSED:
						mw->enable_menu_item(false, 2, menuspkm_Preset_positions, menuspkm_Post_synchro_moves);
						break;
					default:
						break;
				}
			} else // jesli robot jest niezsynchronizowany
			{
				mw->enable_menu_item(true, 1, actionspkm_EDP_Unload);
				mw->enable_menu_item(true, 1, menuspkm_Pre_synchro_moves);
				mw->enable_menu_item(false, 1, actionspkm_EDP_Load);
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

	connect(actionspkm_EDP_Load, 		SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_EDP_Load_triggered(mrrocpp::ui::common::UiRobot*)), 		Qt::AutoCompatConnection);
	connect(actionspkm_EDP_Unload, 		SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_EDP_Unload_triggered(mrrocpp::ui::common::UiRobot*)), 		Qt::AutoCompatConnection);
	connect(actionspkm_Synchronisation, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Synchronisation_triggered(mrrocpp::ui::common::UiRobot*)),	Qt::AutoCompatConnection);
	connect(actionspkm_Motors, 			SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Motors_triggered(mrrocpp::ui::common::UiRobot*)), 		Qt::AutoCompatConnection);
	connect(actionspkm_Motors_post, 	SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Motors_triggered(mrrocpp::ui::common::UiRobot*)), 		Qt::AutoCompatConnection);
	connect(actionspkm_Joints, 			SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Move_Joints_triggered(mrrocpp::ui::common::UiRobot*)), 		Qt::AutoCompatConnection);
	connect(actionspkm_External, 		SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_External_triggered(mrrocpp::ui::common::UiRobot*)), 		Qt::AutoCompatConnection);
	connect(actionspkm_Synchro_Position,SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Synchro_Position_triggered(mrrocpp::ui::common::UiRobot*)), 		Qt::AutoCompatConnection);
	connect(actionspkm_Front_Position, 	SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Front_Position_triggered(mrrocpp::ui::common::UiRobot*)), 		Qt::AutoCompatConnection);
	connect(actionspkm_Position_0, 		SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_0_triggered(mrrocpp::ui::common::UiRobot*)), 		Qt::AutoCompatConnection);
	connect(actionspkm_Position_1, 		SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_1_triggered(mrrocpp::ui::common::UiRobot*)), 		Qt::AutoCompatConnection);
	connect(actionspkm_Position_2, 		SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_2_triggered(mrrocpp::ui::common::UiRobot*)), 		Qt::AutoCompatConnection);
	connect(actionspkm_Clear_Fault, 	SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Clear_Fault_triggered(mrrocpp::ui::common::UiRobot*)), 		Qt::AutoCompatConnection);
}


void UiRobot::setup_menubar()
{
	Ui::MenuBar *menuBar = interface.get_main_window()->getMenuBar();

	actionspkm_EDP_Load			= new Ui::MenuBarAction(QString("EDP &Load"), this, menuBar);
	actionspkm_EDP_Unload		= new Ui::MenuBarAction(QString("EDP &Unload"), this, menuBar);
	actionspkm_Synchronisation	= new Ui::MenuBarAction(QString("&Synchronisation"), this, menuBar);
	actionspkm_Motors			= new Ui::MenuBarAction(QString("&Motors"), this, menuBar);
	actionspkm_Motors_post		= new Ui::MenuBarAction(QString("&Motors"), this, menuBar);
	actionspkm_Joints			= new Ui::MenuBarAction(QString("&Joints"), this, menuBar);
	actionspkm_External			= new Ui::MenuBarAction(QString("&External"), this, menuBar);
	actionspkm_Synchro_Position	= new Ui::MenuBarAction(QString("&Synchro Position"), this, menuBar);
	actionspkm_Front_Position	= new Ui::MenuBarAction(QString("&Front Position"), this, menuBar);
	actionspkm_Position_0		= new Ui::MenuBarAction(QString("Position &0"), this, menuBar);
	actionspkm_Position_1		= new Ui::MenuBarAction(QString("Position &1"), this, menuBar);
	actionspkm_Position_2		= new Ui::MenuBarAction(QString("Position &2"), this, menuBar);
	actionspkm_Clear_Fault		= new Ui::MenuBarAction(QString("&Clear Fault"), this, menuBar);

    menuSpkm = new QMenu(menuBar->menuRobot);

    menuspkm_Pre_synchro_moves = new QMenu(menuSpkm);
    menuspkm_Post_synchro_moves = new QMenu(menuSpkm);
    menuspkm_Preset_positions = new QMenu(menuSpkm);

	menuBar->menuRobot->addAction(menuSpkm->menuAction());

	menuSpkm->addAction(actionspkm_EDP_Load);
	menuSpkm->addAction(actionspkm_EDP_Unload);
	menuSpkm->addSeparator();
	menuSpkm->addAction(menuspkm_Pre_synchro_moves->menuAction());
	menuSpkm->addAction(menuspkm_Post_synchro_moves->menuAction());
	menuSpkm->addAction(menuspkm_Preset_positions->menuAction());
	menuSpkm->addSeparator();
	menuSpkm->addAction(actionspkm_Clear_Fault);
	menuspkm_Pre_synchro_moves->addAction(actionspkm_Synchronisation);
	menuspkm_Pre_synchro_moves->addAction(actionspkm_Motors);
	menuspkm_Post_synchro_moves->addAction(actionspkm_Motors_post);
	menuspkm_Post_synchro_moves->addAction(actionspkm_Joints);
	menuspkm_Post_synchro_moves->addAction(actionspkm_External);
	menuspkm_Preset_positions->addAction(actionspkm_Synchro_Position);
	menuspkm_Preset_positions->addAction(actionspkm_Front_Position);
	menuspkm_Preset_positions->addAction(actionspkm_Position_0);
	menuspkm_Preset_positions->addAction(actionspkm_Position_1);
	menuspkm_Preset_positions->addAction(actionspkm_Position_2);

    actionspkm_EDP_Load->setText(QApplication::translate("mainWindow", "EDP &Load", 0, QApplication::UnicodeUTF8));
    actionspkm_EDP_Unload->setText(QApplication::translate("MainWindow", "EDP &Unload", 0, QApplication::UnicodeUTF8));
    actionspkm_Synchronisation->setText(QApplication::translate("MainWindow", "&Synchronisation", 0, QApplication::UnicodeUTF8));
    actionspkm_Motors->setText(QApplication::translate("MainWindow", "&Motors", 0, QApplication::UnicodeUTF8));
    actionspkm_Motors_post->setText(QApplication::translate("MainWindow", "&Motors", 0, QApplication::UnicodeUTF8));
    actionspkm_Joints->setText(QApplication::translate("MainWindow", "&Joints", 0, QApplication::UnicodeUTF8));
    actionspkm_External->setText(QApplication::translate("MainWindow", "&External", 0, QApplication::UnicodeUTF8));
    actionspkm_Synchro_Position->setText(QApplication::translate("MainWindow", "&Synchro Position", 0, QApplication::UnicodeUTF8));
    actionspkm_Front_Position->setText(QApplication::translate("MainWindow", "&Front Position", 0, QApplication::UnicodeUTF8));
    actionspkm_Position_0->setText(QApplication::translate("MainWindow", "Position &0", 0, QApplication::UnicodeUTF8));
    actionspkm_Position_1->setText(QApplication::translate("MainWindow", "Position &1", 0, QApplication::UnicodeUTF8));
    actionspkm_Position_2->setText(QApplication::translate("MainWindow", "Position &2", 0, QApplication::UnicodeUTF8));

    actionspkm_Clear_Fault->setText(QApplication::translate("MainWindow", "&Clear Fault", 0, QApplication::UnicodeUTF8));
    menuSpkm->setTitle(QApplication::translate("MainWindow", "Sp&km", 0, QApplication::UnicodeUTF8));
    menuspkm_Pre_synchro_moves->setTitle(QApplication::translate("MainWindow", "P&re Synchro Moves", 0, QApplication::UnicodeUTF8));
    menuspkm_Post_synchro_moves->setTitle(QApplication::translate("MainWindow", "P&ost Synchro Moves", 0, QApplication::UnicodeUTF8));
    menuspkm_Preset_positions->setTitle(QApplication::translate("MainWindow", "Pr&eset Positions", 0, QApplication::UnicodeUTF8));
}


void UiRobot::delete_ui_ecp_robot()
{
	delete ui_ecp_robot;
}

int UiRobot::move_to_synchro_position()
{

	for (int i = 0; i < number_of_servos; i++) {
		desired_pos[i] = 0.0;
	}
	eb.command(boost::bind(&ui::spkm::UiRobot::execute_motor_motion, &(*this)));

	return 1;
}

int UiRobot::move_to_front_position()
{

	for (int i = 0; i < number_of_servos; i++) {
		desired_pos[i] = state.edp.front_position[i];
	}
	eb.command(boost::bind(&ui::spkm::UiRobot::execute_joint_motion, &(*this)));

	return 1;
}

int UiRobot::move_to_preset_position(int variant)
{

	for (int i = 0; i < number_of_servos; i++) {
		desired_pos[i] = state.edp.preset_position[variant][i];
	}
	eb.command(boost::bind(&ui::spkm::UiRobot::execute_joint_motion, &(*this)));

	return 1;
}

int UiRobot::execute_motor_motion()
{
	try {

		ui_ecp_robot->move_motors(desired_pos, lib::epos::NON_SYNC_TRAPEZOIDAL);

	} // end try
	CATCH_SECTION_IN_ROBOT

	return 1;
}

int UiRobot::execute_joint_motion()
{
	try {

		ui_ecp_robot->move_joints(desired_pos, lib::epos::NON_SYNC_TRAPEZOIDAL);

	} // end try
	CATCH_SECTION_IN_ROBOT

	return 1;
}

int UiRobot::execute_clear_fault()
{
	try {

		ui_ecp_robot->clear_fault();

	} // end try
	CATCH_SECTION_IN_ROBOT

	return 1;
}

int UiRobot::execute_stop_motor()
{
	try {

		ui_ecp_robot->stop_motors();

	} // end try
	CATCH_SECTION_IN_ROBOT

	return 1;
}

void UiRobot::null_ui_ecp_robot()
{
	ui_ecp_robot = NULL;

}

}
} //namespace ui
} //namespace mrrocpp

