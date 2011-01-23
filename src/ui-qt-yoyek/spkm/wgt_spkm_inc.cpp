#include "ui_ecp_r_spkm.h"
#include "ui_r_spkm.h"
#include "robot/spkm/const_spkm.h"

#include "wgt_spkm_inc.h"
#include "../interface.h"
#include "../mainwindow.h"

wgt_spkm_inc::wgt_spkm_inc(mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::spkm::UiRobot& _robot, QWidget *parent) :
	wgt_base("Spkm incremental motion", _interface, parent), robot(_robot)
{
	ui.setupUi(this);

}

wgt_spkm_inc::~wgt_spkm_inc()
{

}

// slots

void wgt_spkm_inc::on_pushButton_read_clicked()
{
	init();
}

int wgt_spkm_inc::init()
{

	try {

		if (robot.state.edp.pid != -1) {
			if (robot.state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				ui.pushButton_execute->setDisabled(false);

				robot.ui_ecp_robot->epos_reply_data_request_port->set_request();
				robot.ui_ecp_robot->execute_motion();
				robot.ui_ecp_robot->epos_reply_data_request_port->get();

				set_single_axis(0, ui.doubleSpinBox_mcur_0, ui.doubleSpinBox_cur_p0, ui.radioButton_mip_0);
				set_single_axis(1, ui.doubleSpinBox_mcur_1, ui.doubleSpinBox_cur_p1, ui.radioButton_mip_1);
				set_single_axis(2, ui.doubleSpinBox_mcur_2, ui.doubleSpinBox_cur_p2, ui.radioButton_mip_2);
				set_single_axis(3, ui.doubleSpinBox_mcur_3, ui.doubleSpinBox_cur_p3, ui.radioButton_mip_3);
				set_single_axis(4, ui.doubleSpinBox_mcur_4, ui.doubleSpinBox_cur_p4, ui.radioButton_mip_4);
				set_single_axis(5, ui.doubleSpinBox_mcur_5, ui.doubleSpinBox_cur_p5, ui.radioButton_mip_5);

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

int wgt_spkm_inc::set_single_axis(int axis, QDoubleSpinBox* qdsb_mcur, QDoubleSpinBox* qdsb_cur_p, QAbstractButton* qab_mip)
{

	lib::epos::epos_reply &er = robot.ui_ecp_robot->epos_reply_data_request_port->data;
	qdsb_mcur->setValue(er.epos_controller[axis].current);
	qdsb_cur_p->setValue(er.epos_controller[axis].position);

	if (er.epos_controller[axis].motion_in_progress) {
		qab_mip->setChecked(true);
	} else {
		qab_mip->setChecked(false);
	}

	return 1;
}

void wgt_spkm_inc::on_pushButton_import_clicked()
{
	double val[robot.number_of_servos];

	interface.mw->get_lineEdit_position(val, robot.number_of_servos);

	ui.doubleSpinBox_des_p0->setValue(val[0]);
	ui.doubleSpinBox_des_p1->setValue(val[1]);
	ui.doubleSpinBox_des_p2->setValue(val[2]);
	ui.doubleSpinBox_des_p3->setValue(val[3]);
	ui.doubleSpinBox_des_p4->setValue(val[4]);
	ui.doubleSpinBox_des_p5->setValue(val[5]);

}

void wgt_spkm_inc::on_pushButton_export_clicked()
{
	std::stringstream buffer(std::stringstream::in | std::stringstream::out);

	buffer << "edp_spkm INCREMENTAL POSITION\n " << ui.doubleSpinBox_des_p0->value() << " "
			<< ui.doubleSpinBox_des_p1->value() << " " << ui.doubleSpinBox_des_p2->value() << " "
			<< ui.doubleSpinBox_des_p3->value() << " " << ui.doubleSpinBox_des_p4->value() << " "
			<< ui.doubleSpinBox_des_p5->value();

	interface.ui_msg->message(buffer.str());

}

void wgt_spkm_inc::on_pushButton_copy_clicked()
{
	copy();
}

int wgt_spkm_inc::copy()
{

	if (robot.state.edp.pid != -1) {
		if (robot.state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			ui.pushButton_execute->setDisabled(false);

			ui.doubleSpinBox_des_p0->setValue(ui.doubleSpinBox_cur_p0->value());
			ui.doubleSpinBox_des_p1->setValue(ui.doubleSpinBox_cur_p1->value());
			ui.doubleSpinBox_des_p2->setValue(ui.doubleSpinBox_cur_p2->value());
			ui.doubleSpinBox_des_p3->setValue(ui.doubleSpinBox_cur_p3->value());
			ui.doubleSpinBox_des_p4->setValue(ui.doubleSpinBox_cur_p4->value());
			ui.doubleSpinBox_des_p5->setValue(ui.doubleSpinBox_cur_p5->value());

		} else {
			// Wygaszanie elementow przy niezsynchronizowanym robocie
			ui.pushButton_execute->setDisabled(true);
		}

	}

	return 1;
}

int wgt_spkm_inc::motion(/* TR PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo*/)
{
	/* TR
	 double *wektor[robot.number_of_servos];
	 double *step1;
	 */
	// wychwytania ew. bledow ECP::robot
	try {

		if (robot.state.edp.pid != -1) {

			if (robot.state.edp.is_synchronised) {
				/* TR
				 PtGetResource(ABW_PtNumericFloat_wind_spkm_inc_p0, Pt_ARG_NUMERIC_VALUE, &wektor[0], 0);
				 PtGetResource(ABW_PtNumericFloat_wind_spkm_inc_p1, Pt_ARG_NUMERIC_VALUE, &wektor[1], 0);
				 PtGetResource(ABW_PtNumericFloat_wind_spkm_inc_p2, Pt_ARG_NUMERIC_VALUE, &wektor[2], 0);
				 PtGetResource(ABW_PtNumericFloat_wind_spkm_inc_p3, Pt_ARG_NUMERIC_VALUE, &wektor[3], 0);
				 PtGetResource(ABW_PtNumericFloat_wind_spkm_inc_p4, Pt_ARG_NUMERIC_VALUE, &wektor[4], 0);
				 PtGetResource(ABW_PtNumericFloat_wind_spkm_inc_p5, Pt_ARG_NUMERIC_VALUE, &wektor[5], 0);

				 for (int i = 0; i < robot.number_of_servos; i++) {
				 robot.desired_pos[i] = *wektor[i];
				 }
				 */
			} else {

				for (int i = 0; i < robot.number_of_servos; i++) {
					robot.desired_pos[i] = 0.0;
				}
			}
			/* TR
			 PtGetResource(ABW_PtNumericFloat_wind_spkm_inc_step, Pt_ARG_NUMERIC_VALUE, &step1, 0);

			 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_spkm_inc_0l) {
			 robot.desired_pos[0] -= (*step1);
			 } else

			 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_spkm_inc_1l) {
			 robot.desired_pos[1] -= (*step1);
			 } else

			 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_spkm_inc_2l) {
			 robot.desired_pos[2] -= (*step1);
			 } else

			 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_spkm_inc_3l) {
			 robot.desired_pos[3] -= (*step1);
			 } else

			 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_spkm_inc_4l) {
			 robot.desired_pos[4] -= (*step1);
			 } else

			 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_spkm_inc_5l) {
			 robot.desired_pos[5] -= (*step1);
			 } else

			 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_spkm_inc_0r) {
			 robot.desired_pos[0] += (*step1);
			 } else

			 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_spkm_inc_1r) {
			 robot.desired_pos[1] += (*step1);
			 } else

			 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_spkm_inc_2r) {
			 robot.desired_pos[2] += (*step1);
			 } else

			 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_spkm_inc_3r) {
			 robot.desired_pos[3] += (*step1);
			 } else

			 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_spkm_inc_4r) {
			 robot.desired_pos[4] += (*step1);
			 } else

			 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_spkm_inc_5r) {
			 robot.desired_pos[5] += (*step1);
			 }

			 //	std::cout << "UI desired_pos[4]" << desired_pos[4] << std::endl;
			 */
			robot.ui_ecp_robot->move_motors(robot.desired_pos);

			if ((robot.state.edp.is_synchronised) /* TR && (is_open)*/) { // by Y o dziwo nie dziala poprawnie 	 if (robot.state.edp.is_synchronised)
				/* TR
				 PtSetResource(ABW_PtNumericFloat_wind_spkm_inc_p0, Pt_ARG_NUMERIC_VALUE, &robot.desired_pos[0], 0);
				 PtSetResource(ABW_PtNumericFloat_wind_spkm_inc_p1, Pt_ARG_NUMERIC_VALUE, &robot.desired_pos[1], 0);
				 PtSetResource(ABW_PtNumericFloat_wind_spkm_inc_p2, Pt_ARG_NUMERIC_VALUE, &robot.desired_pos[2], 0);
				 PtSetResource(ABW_PtNumericFloat_wind_spkm_inc_p3, Pt_ARG_NUMERIC_VALUE, &robot.desired_pos[3], 0);
				 PtSetResource(ABW_PtNumericFloat_wind_spkm_inc_p4, Pt_ARG_NUMERIC_VALUE, &robot.desired_pos[4], 0);
				 PtSetResource(ABW_PtNumericFloat_wind_spkm_inc_p5, Pt_ARG_NUMERIC_VALUE, &robot.desired_pos[5], 0);
				 */

			}
		} // end if (robot.state.edp.pid!=-1)
	} // end try

	CATCH_SECTION_UI

	return 1;
}

