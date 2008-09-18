
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "mp/Trajectory.h"

Trajectory::Trajectory()
{
	trjName = new char[80];
	trjPoses	= new std::list<ecp_smooth_taught_in_pose>();
}

Trajectory::Trajectory(char *numOfPoses, char *trajectoryName, char *poseSpecification)
{
	trjName = new char[80];
	strcpy(trjName, trajectoryName);
	this->numOfPoses = (uint64_t)atoi(numOfPoses);
	poseSpec = returnProperPS(poseSpecification);
	trjPoses	= new std::list<ecp_smooth_taught_in_pose>();
	
}

Trajectory::Trajectory(const Trajectory &trajectory)
{
	trjName = new char[80];
	strcpy(trjName, trajectory.trjName);
	numOfPoses = trajectory.numOfPoses;
	poseSpec = trajectory.poseSpec;
	trjPoses = new std::list<ecp_smooth_taught_in_pose>(trajectory.trjPoses->begin(), trajectory.trjPoses->end());
}

Trajectory::~Trajectory()
{
	delete []trjName;
	delete trjPoses;
}

void Trajectory::setName(char *trjName)
{
	strcpy(this->trjName, trjName);
}

char * Trajectory::getName() const
{
	return trjName;
}

POSE_SPECIFICATION Trajectory::returnProperPS(char *poseSpecification)
{
	if ( !strcmp(poseSpecification, (const char *)"MOTOR") )
	{	return MOTOR;	}
	if ( !strcmp(poseSpecification, (const char *)"JOINT") ) 
	{	return JOINT;	}
	if ( !strcmp(poseSpecification, (const char *)"XYZ_ANGLE_AXIS") ) 
	{	return XYZ_ANGLE_AXIS;	}
	if ( !strcmp(poseSpecification, (const char *)"XYZ_EULER_ZYZ") ) 
	{	return XYZ_EULER_ZYZ;	}
	else
		return INVALID_END_EFFECTOR;
}

void Trajectory::setValuesInArray(double arrayToFill[], char *dataString)
{
	int index = 0;
	char *value;
	char *toSplit = strdup(dataString);
	
	value = strtok(toSplit, " \t");
	arrayToFill[index++] = atof(value);
	while((value = strtok(NULL, " \t"))!=NULL)
		arrayToFill[index++] = atof(value);
}

void Trajectory::createNewPose()
{
	actPose = new ecp_smooth_taught_in_pose();
	actPose->arm_type = this->poseSpec;
}

void Trajectory::addPoseToTrajectory()
{
	trjPoses->push_back(*actPose);
}

void Trajectory::setNumOfPoses(uint64_t numOfPoses)
{
	this->numOfPoses = numOfPoses;
}

uint64_t Trajectory::getNumberOfPoses() const
{
	return numOfPoses;
}

void Trajectory::setPoseSpecification(char *poseSpecification)
{
	poseSpec = returnProperPS(poseSpecification);
}

POSE_SPECIFICATION Trajectory::getPoseSpecification() const
{
	return poseSpec;
}

void Trajectory::setStartVelocities(char *startVelocities)
{
	setValuesInArray(actPose->v_p, startVelocities);
}

double * Trajectory::getStartVelocities() const
{
	return actPose->v_p;
}

void Trajectory::setEndVelocities(char *endVelocities)
{
	setValuesInArray(actPose->v_k, endVelocities);
}

double * Trajectory::getEndVelocities() const
{
	return actPose->v_k;
}

void Trajectory::setVelocities(char *Velocities)
{
	setValuesInArray(actPose->v, Velocities);
}

double * Trajectory::getVelocities() const
{
	return actPose->v;
}

void Trajectory::setAccelerations(char *accelerations)
{
	setValuesInArray(actPose->a, accelerations);
}

double * Trajectory::getAccelerations() const
{
	return actPose->a;
}

void Trajectory::setCoordinates(char *cCoordinates)
{
	setValuesInArray(actPose->coordinates, cCoordinates);
}

double * Trajectory::getCoordinates() const
{
	return actPose->coordinates;
}

void Trajectory::showTime()
{
	std::list<ecp_smooth_taught_in_pose>::iterator it;
	printf("Nazwa: %s, PoseSpec: %d, NumOfPoses: %d\n", trjName, poseSpec, numOfPoses);
	for(it=trjPoses->begin(); it!=trjPoses->end(); ++it)
	{
		printf("%f %f %f %f %f %f %f %f \n", (*it).v_p[0], (*it).v_p[1], (*it).v_p[2], (*it).v_p[3], 
				(*it).v_p[4], (*it).v_p[5], (*it).v_p[6], (*it).v_p[7]);
		printf("%f %f %f %f %f %f %f %f \n", (*it).v_k[0], (*it).v_k[1], (*it).v_k[2], (*it).v_k[3], 
				(*it).v_k[4], (*it).v_k[5], (*it).v_k[6], (*it).v_k[7]);
		printf("%f %f %f %f %f %f %f %f \n", (*it).v[0], (*it).v[1], (*it).v[2], (*it).v[3], 
				(*it).v[4], (*it).v[5], (*it).v[6], (*it).v[7]);
		printf("%f %f %f %f %f %f %f %f \n", (*it).a[0], (*it).a[1], (*it).a[2], (*it).a[3], 
				(*it).a[4], (*it).a[5], (*it).a[6], (*it).a[7]);
		printf("%f %f %f %f %f %f %f %f \n***\n\n", (*it).coordinates[0], (*it).coordinates[1], (*it).coordinates[2], (*it).coordinates[3], 
				(*it).coordinates[4], (*it).coordinates[5], (*it).coordinates[6], (*it).coordinates[7]);
	}

}

std::list<ecp_smooth_taught_in_pose> * Trajectory::getPoses()
{
	return trjPoses;
}
	
