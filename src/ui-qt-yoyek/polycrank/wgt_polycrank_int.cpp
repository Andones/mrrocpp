//#include "ui_ecp_r_polycrank.h"
#include "ui_r_polycrank.h"
#include "robot/polycrank/const_polycrank.h"
//#include "ui/src/ui_ecp_r_tfg_and_conv.h"
#include "../base/ui_ecp_robot/ui_ecp_r_tfg_and_conv.h"
#include "wgt_polycrank_int.h"
#include "../base/interface.h"
#include "../base/mainwindow.h"

wgt_polycrank_int::wgt_polycrank_int(mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::polycrank::UiRobot& _robot, QWidget *parent) :
	wgt_base("Polycrank incremental motion", _interface, parent), robot(_robot)
{
	ui.setupUi(this);
}

wgt_polycrank_int::~wgt_polycrank_int()
{

}

// slots
void wgt_polycrank_int::on_pushButton_read_clicked()
{
	printf("read\n");
	init();
}

int wgt_polycrank_int::init()
{

	try {

		if (robot.state.edp.pid != -1) {
			if (robot.state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				ui.pushButton_execute->setDisabled(false);
				robot.ui_ecp_robot->read_joints(robot.current_pos);

				ui.doubleSpinBox_cur_p1->setValue(robot.current_pos[0]);
				ui.doubleSpinBox_cur_p2->setValue(robot.current_pos[1]);
				ui.doubleSpinBox_cur_p3->setValue(robot.current_pos[2]);
				ui.doubleSpinBox_cur_p4->setValue(robot.current_pos[3]);
				ui.doubleSpinBox_cur_p5->setValue(robot.current_pos[4]);
				ui.doubleSpinBox_cur_p6->setValue(robot.current_pos[5]);
				ui.doubleSpinBox_cur_p7->setValue(robot.current_pos[6]);

				for (int i = 0; i < robot.number_of_servos; i++) {
					robot.desired_pos[i] = robot.current_pos[i];
				}

			} else {
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				ui.pushButton_execute->setDisabled(true);
			}
		}

	} // end try
	CATCH_SECTION_UI

	return 1;
}

void wgt_polycrank_int::on_pushButton_import_clicked()
{
	double val[robot.number_of_servos];

	for (int i = 0; i < robot.number_of_servos; i++) {
		val[i] = 0.0;
	}

	interface.mw->get_lineEdit_position(val, robot.number_of_servos);

	ui.doubleSpinBox_des_p1->setValue(val[0]);
	ui.doubleSpinBox_des_p2->setValue(val[1]);
	ui.doubleSpinBox_des_p3->setValue(val[2]);
	ui.doubleSpinBox_des_p4->setValue(val[3]);
	ui.doubleSpinBox_des_p5->setValue(val[4]);
	ui.doubleSpinBox_des_p6->setValue(val[5]);
	ui.doubleSpinBox_des_p7->setValue(val[6]);

}

void wgt_polycrank_int::on_pushButton_export_clicked()
{

	std::stringstream buffer(std::stringstream::in | std::stringstream::out);

	buffer << "edp_polycrank INCREMENTAL POSITION\n " << ui.doubleSpinBox_des_p1->value() << " "
			<< ui.doubleSpinBox_des_p2->value() << " " << ui.doubleSpinBox_des_p3->value() << " "
			<< ui.doubleSpinBox_des_p4->value() << " " << ui.doubleSpinBox_des_p5->value() << " "
			<< ui.doubleSpinBox_des_p6->value() << " " << ui.doubleSpinBox_des_p7->value();

	interface.ui_msg->message(buffer.str());
}

void wgt_polycrank_int::on_pushButton_copy_clicked()
{
	copy();
}

int wgt_polycrank_int::copy()
{

	if (robot.state.edp.pid != -1) {
		if (robot.state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			ui.pushButton_execute->setDisabled(false);

			ui.doubleSpinBox_des_p1->setValue(ui.doubleSpinBox_cur_p1->value());
			ui.doubleSpinBox_des_p2->setValue(ui.doubleSpinBox_cur_p2->value());
			ui.doubleSpinBox_des_p3->setValue(ui.doubleSpinBox_cur_p3->value());
			ui.doubleSpinBox_des_p4->setValue(ui.doubleSpinBox_cur_p4->value());
			ui.doubleSpinBox_des_p5->setValue(ui.doubleSpinBox_cur_p5->value());
			ui.doubleSpinBox_des_p6->setValue(ui.doubleSpinBox_cur_p6->value());
			ui.doubleSpinBox_des_p7->setValue(ui.doubleSpinBox_cur_p7->value());

		} else {
			// Wygaszanie elementow przy niezsynchronizowanym robocie
			ui.pushButton_execute->setDisabled(true);
		}

	}

	return 1;
}

void wgt_polycrank_int::on_pushButton_execute_clicked()
{
	get_desired_position();
	move_it();
}

void wgt_polycrank_int::on_pushButton_1l_clicked()
{
	get_desired_position();
	robot.desired_pos[0] -= ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_polycrank_int::on_pushButton_2l_clicked()
{
	get_desired_position();
	robot.desired_pos[1] -= ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_polycrank_int::on_pushButton_3l_clicked()
{
	get_desired_position();
	robot.desired_pos[2] -= ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_polycrank_int::on_pushButton_4l_clicked()
{
	get_desired_position();
	robot.desired_pos[3] -= ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_polycrank_int::on_pushButton_5l_clicked()
{
	get_desired_position();
	robot.desired_pos[4] -= ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_polycrank_int::on_pushButton_6l_clicked()
{
	get_desired_position();
	robot.desired_pos[5] -= ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_polycrank_int::on_pushButton_7l_clicked()
{
	get_desired_position();
	robot.desired_pos[6] -= ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_polycrank_int::on_pushButton_1r_clicked()
{
	get_desired_position();
	robot.desired_pos[0] += ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_polycrank_int::on_pushButton_2r_clicked()
{
	get_desired_position();
	robot.desired_pos[1] += ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_polycrank_int::on_pushButton_3r_clicked()
{
	get_desired_position();
	robot.desired_pos[2] += ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_polycrank_int::on_pushButton_4r_clicked()
{
	get_desired_position();
	robot.desired_pos[3] += ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_polycrank_int::on_pushButton_5r_clicked()
{
	get_desired_position();
	robot.desired_pos[4] += ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_polycrank_int::on_pushButton_6r_clicked()
{
	get_desired_position();
	robot.desired_pos[5] += ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_polycrank_int::on_pushButton_7r_clicked()
{
	get_desired_position();
	robot.desired_pos[6] += ui.doubleSpinBox_step->value();
	move_it();
}

int wgt_polycrank_int::get_desired_position()
{

	if (robot.state.edp.pid != -1) {

		if (robot.state.edp.is_synchronised) {

			robot.desired_pos[0] = ui.doubleSpinBox_des_p1->value();
			robot.desired_pos[1] = ui.doubleSpinBox_des_p2->value();
			robot.desired_pos[2] = ui.doubleSpinBox_des_p3->value();
			robot.desired_pos[3] = ui.doubleSpinBox_des_p4->value();
			robot.desired_pos[4] = ui.doubleSpinBox_des_p5->value();
			robot.desired_pos[5] = ui.doubleSpinBox_des_p6->value();
			robot.desired_pos[6] = ui.doubleSpinBox_des_p7->value();

		} else {

			for (int i = 0; i < robot.number_of_servos; i++) {
				robot.desired_pos[i] = 0.0;
			}
		}
	}
	return 1;
}

int wgt_polycrank_int::move_it()
{

	// wychwytania ew. bledow ECP::robot
	try {

		if (robot.state.edp.pid != -1) {

			//robot.ui_ecp_robot->move_motors(robot.desired_pos);

			//robot.ui_ecp_robot->move_motors(robot.desired_pos);
			robot.ui_ecp_robot->move_joints(robot.desired_pos);

			//robot.ui_ecp_robot->interface.polycrank->ui_ecp_robot->move_joints(robot.desired_pos);
			//interface.polycrank->ui_ecp_robot->move_motors(interface.polycrank->desired_pos);
			//interface.polycrank->ui_ecp_robot->move_joints(interface.polycrank->desired_pos);
			//robot.ui_ecp_robot->move_motors(robot.desired_pos);

			if ((robot.state.edp.is_synchronised) /* TR && (is_open)*/) { // by Y o dziwo nie dziala poprawnie 	 if (robot.state.edp.is_synchronised)
				ui.doubleSpinBox_des_p1->setValue(robot.desired_pos[0]);
				ui.doubleSpinBox_des_p2->setValue(robot.desired_pos[1]);
				ui.doubleSpinBox_des_p3->setValue(robot.desired_pos[2]);
				ui.doubleSpinBox_des_p4->setValue(robot.desired_pos[3]);
				ui.doubleSpinBox_des_p5->setValue(robot.desired_pos[4]);
				ui.doubleSpinBox_des_p6->setValue(robot.desired_pos[5]);
				ui.doubleSpinBox_des_p7->setValue(robot.desired_pos[6]);
			}
		} // end if (robot.state.edp.pid!=-1)
	} // end try

	CATCH_SECTION_UI

	return 1;
}

