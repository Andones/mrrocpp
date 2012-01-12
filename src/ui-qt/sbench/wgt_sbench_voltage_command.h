#ifndef WGT_SBENCH_VOLTAGE_COMMAND_H
#define WGT_SBENCH_VOLTAGE_COMMAND_H

/*!
 * @file
 * @brief File contains wgt_sbench_voltage_command class declaration for SwarmItFix Bench
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup sbench
 */

#include <QtGui/QWidget>
#include <QVBoxLayout>
#include <QDockWidget>
#include "wgt_sbench_command.h"

namespace mrrocpp {
namespace ui {
namespace sbench {
const std::string WGT_SBENCH_VOLTAGE_COMMAND = "WGT_SBENCH_VOLTAGE_COMMAND";
}
}
}

class wgt_sbench_voltage_command : public wgt_sbench_command
{
	Q_OBJECT

public:
	wgt_sbench_voltage_command(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *_robot, QWidget *parent =
			0);
	~wgt_sbench_voltage_command();

	void init();
	void execute();
	void read_and_copy();

};

#endif // WGT_SBENCH_COMMAND_H
