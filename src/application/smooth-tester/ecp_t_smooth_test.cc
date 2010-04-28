#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <iostream>


#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp_t_smooth_test.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

//Constructors
smooth_test::smooth_test(lib::configurator &_config): task(_config)
{
	ecp_m_robot = new robot(*this);

	//delay(20000);
	smoothgen2 = new common::generator::smooth(*this, true);
	sr_ecp_msg->message("ECP loaded smooth_test");
};

void smooth_test::main_task_algorithm(void ) {
	//ecp_m_robot = new ecp_irp6_on_track_robot(*this);
	//smoothgen2 = new ecp_smooth_generator(*this, true);
	//sr_ecp_msg->message("ECP loaded smooth_test");

	sr_ecp_msg->message("ECP smooth_test ready");

	//smoothgen2->set_relative();
	smoothgen2->set_absolute();
	smoothgen2->load_file_with_path("/net/olin/mnt/mrroc/src/application/smooth-tester/trj/smooth2test2.trj");
		/*char size[10];
		double size2 = smoothgen2->pose_list_length();
		sprintf(size,"%f",size2);
		sr_ecp_msg->message(size);*/

	smoothgen2->Move();

	//printf("wielkosc listy: %d\n", smoothgen2->pose_list_length());
	//fflush();

	  //smoothgen2->Move();
	  //sr_ecp_msg->message("jest git");
	smoothgen2->reset();

	ecp_termination_notice();
};

}
} // namespace irp6ot

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config){
	return new irp6ot::task::smooth_test(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


