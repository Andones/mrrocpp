/*
 * datastr.cc
 *
 *  Created on: Oct 21, 2009
 *      Author: ptroja
 *
 */

#include <sstream>
#include <string.h>
#include <stdlib.h>

#include "lib/datastr.h"

namespace mrrocpp {
namespace lib {

std::string toString(double valArr[], int length)
{
	std::ostringstream stm;
	for(int i=0; i<length; i++)
	{
		if(i==0)
			stm << valArr[i];
		else
			stm << "\t" << valArr[i];
	}

//	std::cout<<stm.str()<<std::endl;
	return stm.str();
}

std::string toString(int numberOfPoses)
{
	std::ostringstream stm;

	stm << numberOfPoses;

	return stm.str();
}

std::string toString(lib::ROBOT_ENUM robot)
{
	using namespace lib;
	switch (robot)
	{
		case ROBOT_IRP6_ON_TRACK:
			return std::string("ROBOT_IRP6_ON_TRACK");
		case ROBOT_IRP6_POSTUMENT:
			return std::string("ROBOT_IRP6_POSTUMENT");
		case ROBOT_CONVEYOR:
			return std::string("ROBOT_CONVEYOR");
		case ROBOT_SPEAKER:
			return std::string("ROBOT_SPEAKER");
		case ROBOT_IRP6_MECHATRONIKA:
			return std::string("ROBOT_IRP6_MECHATRONIKA");
		case ROBOT_ELECTRON:
			return std::string("ROBOT_ELECTRON");
		case ROBOT_FESTIVAL:
			return std::string("ROBOT_FESTIVAL");
		case ROBOT_HAND:
			return std::string("ROBOT_HAND");
		case ROBOT_SPEECHRECOGNITION:
			return std::string("ROBOT_SPEECHRECOGNITION");
		default:
			return std::string("ROBOT_UNDEFINED");
	}
}

std::string toString(lib::POSE_SPECIFICATION ps)
{
	switch (ps)
	{
		case lib::MOTOR:
			return std::string("MOTOR");
		case lib::JOINT:
			return std::string("JOINT");
		case lib::XYZ_ANGLE_AXIS:
			return std::string("XYZ_ANGLE_AXIS");
		case lib::XYZ_EULER_ZYZ:
			return std::string("XYZ_EULER_ZYZ");
		default:
			return std::string("INVALID_END_EFFECTOR");
	}
}

//-----------------------------------------------------------------------------------------------------------

lib::ROBOT_ENUM returnProperRobot(const std::string & robotName)
{
	if(robotName == "ROBOT_IRP6_ON_TRACK")
		return lib::ROBOT_IRP6_ON_TRACK;
	else if(robotName == "ROBOT_IRP6_POSTUMENT")
		return lib::ROBOT_IRP6_POSTUMENT;
	else if(robotName == "ROBOT_CONVEYOR")
		return lib::ROBOT_CONVEYOR;
	else if(robotName == "ROBOT_SPEAKER")
		return lib::ROBOT_SPEAKER;
	else if(robotName == "ROBOT_IRP6_MECHATRONIKA")
		return lib::ROBOT_IRP6_MECHATRONIKA;
	else if(robotName == "ROBOT_ELECTRON")
		return lib::ROBOT_ELECTRON;
	else if(robotName == "ROBOT_FESTIVAL")
		return lib::ROBOT_FESTIVAL;
	else if(robotName == "ROBOT_HAND")
		return lib::ROBOT_HAND;
	else if(robotName == "ROBOT_SPEECHRECOGNITION")
		return lib::ROBOT_SPEECHRECOGNITION;
	else
		return lib::ROBOT_UNDEFINED;
}

lib::POSE_SPECIFICATION returnProperPS(const std::string & poseSpecification)
{
	if (poseSpecification == "MOTOR")
	{	return lib::MOTOR;	}
	if (poseSpecification == "JOINT")
	{	return lib::JOINT;	}
	if (poseSpecification == "XYZ_ANGLE_AXIS")
	{	return lib::XYZ_ANGLE_AXIS;	}
	if (poseSpecification == "XYZ_EULER_ZYZ")
	{	return lib::XYZ_EULER_ZYZ;	}
	else
		return lib::INVALID_END_EFFECTOR;
}


int setValuesInArray(double arrayToFill[], const char *dataString)
{
	int index = 0;
	char *value;
	char *toSplit = strdup(dataString);

	value = strtok(toSplit, " \t");
	arrayToFill[index++] = atof(value);
	while((value = strtok(NULL, " \t"))!=NULL)
		arrayToFill[index++] = atof(value);

	free(toSplit);

	return index;
}

}
}
