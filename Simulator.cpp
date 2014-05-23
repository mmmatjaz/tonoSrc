/*
 * Simulator.cpp
 *
 *  Created on: Apr 17, 2014
 *      Author: m
 */

#include "Simulator.h"


Simulator::Simulator(RosPose * meas, RosPose * cmd, Vector6f * ft) {
	this->pose=meas;
	this->cmd=cmd;
	this->forceATI=ft;

	pose->position.x=1.5;
	pose->position.y=1.5;
	pose->position.z=1.3;

	pose->orientation.w=0.0157;
	pose->orientation.z=0.0001;
	pose->orientation.y=0.9999;
	pose->orientation.x=0.0050;

	(*forceATI)<<0,0,0,0,0,0;
}

void Simulator::simulateMotion() {
	memcpy(pose,cmd,sizeof(RosPose));
	usleep(1e3);
}

void Simulator::simulateForce(Matrix3f& rotTono, Matrix3f& rotFt, Vector3f& tonoPos, Vector3f& tonoInitPos) {
	float fz;
	if (tonoPos[2]>tonoInitPos[2])
		fz=0;
	else fz=(-SIM_STIFF*(tonoPos[2]-tonoInitPos[2]));
	f<<0,0,fz;
	f= rotFt* rotTono*f;
	*forceATI << f[0],f[1],f[2],0,0,0;
}
