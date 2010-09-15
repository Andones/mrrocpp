//#include "robot/sarkofag/edp_e_sarkofag.h"
#include "robot/hi_moxa/hi_moxa.h"

//#include "hi_sarkofag.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <fcntl.h>

#include <exception>
#include <stdexcept>
#include <cstring>
#include <iostream>

#define TOTAL_DRIVES 5

#include "base/edp/edp_e_motor_driven.h"

namespace mrrocpp {
namespace edp {
namespace hi_moxa {

HI_moxa::HI_moxa(common::motor_driven_effector &_master) :
	common::HardwareInterface(_master) {
#ifdef T_INFO_FUNC
	std::cout << "[func] Hi, Moxa!" << std::endl;
#endif
}

HI_moxa::~HI_moxa() {
#ifdef T_INFO_FUNC
	std::cout << "[func] Bye, Moxa!" << std::endl;
#endif
	for (unsigned int i = 0; i < TOTAL_DRIVES; i++) {
		if (fd[i] > 0) {
			tcsetattr(fd[i], TCSANOW, &oldtio[i]);
			close( fd[i]);
		}
	}
}

void HI_moxa::init() {

	std::string port = PORT;

#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::init()" << std::endl;
#endif

	// inicjalizacja zmiennych
	for (unsigned int i = 0; i < TOTAL_DRIVES; i++) {
		servo_data[i].first_hardware_read = true;
		servo_data[i].command_params = 0;
	}

	fd_max = 0;
	for (unsigned int i = 0; i < TOTAL_DRIVES; i++) {
		std::cout << "[info] opening port : "
				<< (port + (char) (i + 50)).c_str();
		fd[i] = open((port + (char) (i + 50)).c_str(), O_RDWR | O_NOCTTY
				| O_NDELAY);
		if (fd[i] < 0) {
			//	throw(std::runtime_error("unable to open device!!!"));
			std::cout << std::endl << "[error] fd == " << (int) fd[i]
					<< std::endl;
		} else {
			std::cout << "...OK (" << (int) fd[i] << ")" << std::endl;
			if(fd[i] > fd_max)
				fd_max = fd[i];
		}
		tcgetattr(fd[i], &oldtio[i]);

		// set up new settings
		struct termios newtio;
		memset(&newtio, 0, sizeof(newtio));
		newtio.c_cflag = CS8 | CLOCAL | CREAD | CSTOPB;
		newtio.c_iflag = INPCK; //IGNPAR;
		newtio.c_oflag = 0;
		newtio.c_lflag = 0;
		if (cfsetispeed(&newtio, BAUD) < 0 || cfsetospeed(&newtio, BAUD) < 0) {
			tcsetattr(fd[i], TCSANOW, &oldtio[i]);
			close( fd[i]);
			fd[i] = -1;
			throw(std::runtime_error("unable to set baudrate !!!"));
			return;
		}
		// activate new settings
		tcflush(fd[i], TCIFLUSH);
		tcsetattr(fd[i], TCSANOW, &newtio);
	}
	std::cout << "[info] fd_max: " << fd_max << std::endl;

	clock_gettime(CLOCK_MONOTONIC, &wake_time);
}

void HI_moxa::insert_set_value(int drive_number, double set_value) {
#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::insert_set_value(" << drive_number << ", " << set_value << ")" << std::endl;
#endif

//	if(drive_number == 0){
//		std::cout << "[func] HI_moxa::insert_set_value(" << drive_number << ", " << set_value << ")" << std::endl;
//	}

	servo_data[drive_number].buf[0] = 0x00;
	servo_data[drive_number].buf[1] = 0x00;
	servo_data[drive_number].buf[2] = 0x00;
	servo_data[drive_number].buf[3] = 0x00;
	servo_data[drive_number].buf[4] = START_BYTE;
	servo_data[drive_number].buf[5] = COMMAND_MODE_PWM | servo_data[drive_number].command_params;
	struct pwm_St* temp = (pwm_St*) &(servo_data[drive_number].buf[6]);
	//temp->pwm = set_value / 0.500;
	temp->pwm = set_value / 1.000;
	//temp->pwm = set_value * (1000.0/ 190.0);


#ifdef T_INFO_CALC
	std::cout << "[calc] pwm: (" << temp->pwm << ")" << std::endl;
#endif
}

int HI_moxa::get_current(int drive_number) {
	int ret;

	ret = servo_data[drive_number].drive_status.current;

#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::get_current(" << drive_number << ") = " << ret << std::endl;
#endif
	return ret;
	//return 0;
}

double HI_moxa::get_increment(int drive_number) {
	double ret;
	ret = servo_data[drive_number].current_position_inc;

#ifdef T_INFO_FUNC
	if(drive_number == 4)
		std::cout << "[func] HI_moxa::get_increment(" << drive_number << ") = " << ret << std::endl;
#endif
	return ret;
}

long int HI_moxa::get_position(int drive_number) {
	int ret;

	ret = servo_data[drive_number].current_absolute_position;

#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::get_position(" << drive_number << ") = " << ret << std::endl;
#endif
	return ret;
}

uint64_t HI_moxa::read_write_hardware(void) {
	static int64_t receive_attempts = 0, receive_timeouts = 0;
	static uint8_t error_power_stage = 0;
	bool hardware_read_ok = true;
	bool all_hardware_read = true;
	unsigned int bytes_received[TOTAL_DRIVES];
	fd_set rfds;
	uint64_t ret = 0;
	uint8_t drive_number;

	/*std::cout << "[info] pos:";
	for(drive_number = 0; drive_number < TOTAL_DRIVES; drive_number++){
		std::cout << " " << servo_data[drive_number].current_absolute_position;
	}
	std::cout << std::endl;*/
	
//	std::cout << "[info] przed write: " << std::endl;

	for(drive_number = 0; drive_number < TOTAL_DRIVES; drive_number++)
	{
		write(fd[drive_number], servo_data[drive_number].buf, WRITE_BYTES);
		bytes_received[drive_number] = 0;
	}

//	std::cout << "[info] po write: " << std::endl;


	receive_attempts++;
	for (int i = 0; i < (TOTAL_DRIVES*READ_BYTES); i++)
	//while(1)
	{
		FD_ZERO(&rfds);
		for(drive_number = 0; drive_number < TOTAL_DRIVES; drive_number++) {
			if(bytes_received[drive_number] < READ_BYTES){
				FD_SET(fd[drive_number], &rfds);
			}
		}

		// timeout
		struct timeval timeout;
		timeout.tv_sec = (time_t) 0;
		timeout.tv_usec = 500;

//		std::cout << "[info] przed select: " << std::endl;

		int select_retval = select(fd_max + 1, &rfds, NULL, NULL, &timeout);

//		std::cout << "[info] po select, select_retval: " << select_retval << std::endl;

		if (select_retval == 0) {
			//throw(std::runtime_error("communication timeout !!!"));
			/*std::cout << "[error] communication timeout ("
					<< ++receive_timeouts << "/" << receive_attempts << "="
					<< ((float) receive_timeouts / receive_attempts) << ")";
//					<< std::endl;*/
			for(drive_number=0; drive_number<TOTAL_DRIVES; drive_number++) {
				//if(bytes_received[drive_number] < READ_BYTES)
					//std::cout << " " << (int) drive_number << "(" << READ_BYTES - bytes_received[drive_number] << ")" ;
			}
			//std::cout << std::endl;


			hardware_read_ok = false;
			break;
		} else {
			all_hardware_read = true;
			for(drive_number=0; drive_number<TOTAL_DRIVES; drive_number++) {
				if(FD_ISSET(fd[drive_number], &rfds)) {
					bytes_received[drive_number] += read(fd[drive_number],
							(char*) (&(servo_data[drive_number].drive_status)) + bytes_received[drive_number],
							READ_BYTES - bytes_received[drive_number]);
				}
				if(bytes_received[drive_number] < READ_BYTES){
					all_hardware_read = false;
				}
			}
			if(all_hardware_read) {
				break;
			}
			/*else{
				for(drive_number=0; drive_number<TOTAL_DRIVES; drive_number++) {
					if(bytes_received[drive_number] < READ_BYTES){
						std::cout << "Waiting for " << (int)drive_number << std::endl;	//############################
					}
				}
			}*/
//			std::cout << std::endl;	//############################
		}
	}

	// Wypelnienie pol odebranymi danymi


	// ########### TODO:
	drive_number=0;

	for(drive_number=0; drive_number<TOTAL_DRIVES; drive_number++) {

		// Wypelnienie pol odebranymi danymi
		if(bytes_received[drive_number] >= READ_BYTES) {
			servo_data[drive_number].previous_absolute_position = servo_data[drive_number].current_absolute_position;
			servo_data[drive_number].current_absolute_position = servo_data[drive_number].drive_status.position;
		}

		// W pierwszym odczycie danych z napedu przyrost pozycji musi byc 0.
		if (servo_data[drive_number].first_hardware_read && hardware_read_ok) {
			servo_data[drive_number].previous_absolute_position = servo_data[drive_number].current_absolute_position;
			servo_data[drive_number].first_hardware_read = false;
		}

		servo_data[drive_number].current_position_inc = (double) (servo_data[drive_number].current_absolute_position - servo_data[drive_number].previous_absolute_position);
	}

	// ########### TODO:
	drive_number=0;

	master.controller_state_edp_buf.is_robot_blocked = (servo_data[drive_number].drive_status.powerStageFault != 0) ? true : false;
	if (servo_data[drive_number].drive_status.powerStageFault != 0) {

		if (error_power_stage == 0) {
			master.msg->message(lib::NON_FATAL_ERROR, "Wylaczono moc - robot zablokowany");
			error_power_stage++;
		}

	} else {
		error_power_stage = 0;
	}

	master.controller_state_edp_buf.is_synchronised = (servo_data[drive_number].drive_status.isSynchronized != 0) ? true : false;

	if (servo_data[drive_number].drive_status.sw1 != 0)
		ret |= UPPER_LIMIT_SWITCH;
	if (servo_data[drive_number].drive_status.sw2 != 0)
		ret |= LOWER_LIMIT_SWITCH;
	if (servo_data[drive_number].drive_status.swSynchr != 0)
		ret |= SYNCHRO_SWITCH_ON;
	if (servo_data[drive_number].drive_status.synchroZero != 0)
		ret |= SYNCHRO_ZERO;
	if (servo_data[drive_number].drive_status.overcurrent != 0)
		ret |= OVER_CURRENT;

	while ((wake_time.tv_nsec += COMMCYCLE_TIME_NS) > 1000000000) {
		wake_time.tv_sec += 1;
		wake_time.tv_nsec -= 1000000000;
	}

	clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wake_time, NULL);

	return ret;
}

void HI_moxa::reset_counters(void) {
	//#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::reset_counters" << std::endl;
	//#endif
}

void HI_moxa::start_synchro(int drive_number) {
	servo_data[drive_number].command_params |= COMMAND_PARAM_SYNCHRO;

	//#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::start_synchro(" << drive_number << ")"
			<< std::endl;
	//#endif
}

void HI_moxa::finish_synchro(int drive_number) {
	servo_data[drive_number].command_params &= 0;

	//#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::finish_synchro(" << drive_number << ")"
			<< std::endl;
	//#endif
}

bool HI_moxa::is_impulse_zero(int drive_number) {
	//#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::is_impulse_zero(" << drive_number << ")"
			<< std::endl;
	//#endif
	return false;
}

void HI_moxa::reset_position(int drive_number) {
	servo_data[drive_number].current_absolute_position = 0L;
	servo_data[drive_number].previous_absolute_position = 0L;
	servo_data[drive_number].current_position_inc = 0.0;
	servo_data[drive_number].first_hardware_read = true;
	//#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::reset_position(" << drive_number << ")" << std::endl;
	//#endif
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

