/*
 * generator/ecp_g_sleep.cc
 *
 *Author: Tomasz Bem
 */

#include "ecp_g_smb.h"

namespace mrrocpp {
namespace ecp {
namespace smb {
namespace generator {

//constructor with parameters: task and time to sleep [s]
pin_lock::pin_lock (common::task::task& _ecp_task, double s): generator (_ecp_task){
	if (the_robot) the_robot->communicate_with_edp=false;	//do not communicate with edp
	waittime=s*1000;			//wait time[ns] conversting from given seconds to nanoseconds
	sleeptime.tv_nsec=20000000;	//sleep time[ns]
	sleeptime.tv_sec=0;
}

//allow for later change of a sleep time
void pin_lock::init_time(double s){
	waittime=s*1000; //TODO: conversion from seconds to nanoseconds (?!)
}

bool pin_lock::first_step(){
	if( clock_gettime( CLOCK_REALTIME , &acttime) == -1 ){	//acquiring actual time
		printf("sleep generator: first step time measurement error");
		return false;
	}

	starttime=acttime;
	return true;
}

bool pin_lock::next_step(){
	double diff;

	prevtime=acttime;
	if( clock_gettime( CLOCK_REALTIME , &acttime) == -1 ){
		printf("sleep generator: next step time measurement error");
	}

	//difference between consecutive next_steeps, check if the pause button was pressed (difference bigger than 100ms)
	diff=(acttime.tv_sec-prevtime.tv_sec)*1000+(acttime.tv_nsec-prevtime.tv_nsec)/1000000;
	if(diff>100)
		waittime=waittime+diff;

	//difference between start time and actual time, check if wait time already passed
	diff=(acttime.tv_sec-starttime.tv_sec)*1000+(acttime.tv_nsec-starttime.tv_nsec)/1000000;
	if(diff>waittime)
		return false;
	else{
		nanosleep(&sleeptime,NULL);
		return true;
	}
}












//constructor with parameters: task and time to sleep [s]
pin_unlock::pin_unlock (common::task::task& _ecp_task, double s): generator (_ecp_task){
	if (the_robot) the_robot->communicate_with_edp=false;	//do not communicate with edp
	waittime=s*1000;			//wait time[ns] conversting from given seconds to nanoseconds
	sleeptime.tv_nsec=20000000;	//sleep time[ns]
	sleeptime.tv_sec=0;
}

//allow for later change of a sleep time
void pin_unlock::init_time(double s){
	waittime=s*1000; //TODO: conversion from seconds to nanoseconds (?!)
}

bool pin_unlock::first_step(){
	if( clock_gettime( CLOCK_REALTIME , &acttime) == -1 ){	//acquiring actual time
		printf("sleep generator: first step time measurement error");
		return false;
	}

	starttime=acttime;
	return true;
}

bool pin_unlock::next_step(){
	double diff;

	prevtime=acttime;
	if( clock_gettime( CLOCK_REALTIME , &acttime) == -1 ){
		printf("sleep generator: next step time measurement error");
	}

	//difference between consecutive next_steeps, check if the pause button was pressed (difference bigger than 100ms)
	diff=(acttime.tv_sec-prevtime.tv_sec)*1000+(acttime.tv_nsec-prevtime.tv_nsec)/1000000;
	if(diff>100)
		waittime=waittime+diff;

	//difference between start time and actual time, check if wait time already passed
	diff=(acttime.tv_sec-starttime.tv_sec)*1000+(acttime.tv_nsec-starttime.tv_nsec)/1000000;
	if(diff>waittime)
		return false;
	else{
		nanosleep(&sleeptime,NULL);
		return true;
	}
}






//constructor with parameters: task and time to sleep [s]
pin_rise::pin_rise (common::task::task& _ecp_task, double s): generator (_ecp_task){
	if (the_robot) the_robot->communicate_with_edp=false;	//do not communicate with edp
	waittime=s*1000;			//wait time[ns] conversting from given seconds to nanoseconds
	sleeptime.tv_nsec=20000000;	//sleep time[ns]
	sleeptime.tv_sec=0;
}

//allow for later change of a sleep time
void pin_rise::init_time(double s){
	waittime=s*1000; //TODO: conversion from seconds to nanoseconds (?!)
}

bool pin_rise::first_step(){
	if( clock_gettime( CLOCK_REALTIME , &acttime) == -1 ){	//acquiring actual time
		printf("sleep generator: first step time measurement error");
		return false;
	}

	starttime=acttime;
	return true;
}

bool pin_rise::next_step(){
	double diff;

	prevtime=acttime;
	if( clock_gettime( CLOCK_REALTIME , &acttime) == -1 ){
		printf("sleep generator: next step time measurement error");
	}

	//difference between consecutive next_steeps, check if the pause button was pressed (difference bigger than 100ms)
	diff=(acttime.tv_sec-prevtime.tv_sec)*1000+(acttime.tv_nsec-prevtime.tv_nsec)/1000000;
	if(diff>100)
		waittime=waittime+diff;

	//difference between start time and actual time, check if wait time already passed
	diff=(acttime.tv_sec-starttime.tv_sec)*1000+(acttime.tv_nsec-starttime.tv_nsec)/1000000;
	if(diff>waittime)
		return false;
	else{
		nanosleep(&sleeptime,NULL);
		return true;
	}
}











//constructor with parameters: task and time to sleep [s]
pin_lower::pin_lower (common::task::task& _ecp_task, double s): generator (_ecp_task){
	if (the_robot) the_robot->communicate_with_edp=false;	//do not communicate with edp
	waittime=s*1000;			//wait time[ns] conversting from given seconds to nanoseconds
	sleeptime.tv_nsec=20000000;	//sleep time[ns]
	sleeptime.tv_sec=0;
}

//allow for later change of a sleep time
void pin_lower::init_time(double s){
	waittime=s*1000; //TODO: conversion from seconds to nanoseconds (?!)
}

bool pin_lower::first_step(){
	if( clock_gettime( CLOCK_REALTIME , &acttime) == -1 ){	//acquiring actual time
		printf("sleep generator: first step time measurement error");
		return false;
	}

	starttime=acttime;
	return true;
}

bool pin_lower::next_step(){
	double diff;

	prevtime=acttime;
	if( clock_gettime( CLOCK_REALTIME , &acttime) == -1 ){
		printf("sleep generator: next step time measurement error");
	}

	//difference between consecutive next_steeps, check if the pause button was pressed (difference bigger than 100ms)
	diff=(acttime.tv_sec-prevtime.tv_sec)*1000+(acttime.tv_nsec-prevtime.tv_nsec)/1000000;
	if(diff>100)
		waittime=waittime+diff;

	//difference between start time and actual time, check if wait time already passed
	diff=(acttime.tv_sec-starttime.tv_sec)*1000+(acttime.tv_nsec-starttime.tv_nsec)/1000000;
	if(diff>waittime)
		return false;
	else{
		nanosleep(&sleeptime,NULL);
		return true;
	}
}




} // namespace generator
} // namespace smb
} // namespace ecp
} // namespace mrrocpp

