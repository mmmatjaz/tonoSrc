/*
 * RobotState.cpp
 *
 *  Created on: Apr 16, 2014
 *      Author: m
 */

#include "RobotState.h"


RobotState::RobotState(RosPose * meas, RosPose * cmd, Vector6f * ft) {
	this->pose=meas;
	this->cmd=cmd;
	this->forceATI=ft;

	transEeff2Tono<< TONO_TRANS_X, TONO_TRANS_Y, TONO_TRANS_Z;

	//float fi=FT_ROT_Z;
	rotEef2Ft<< cos(FT_ROT_Z), -sin(FT_ROT_Z), 	0,
				sin(FT_ROT_Z),  cos(FT_ROT_Z), 	0,
					0, 		0, 		FT_Z;
}

void RobotState::readRosPose() {
	endeffTrans<<
			pose->position.x,
			pose->position.y,
			pose->position.z;

	rosQuat2DCM();
	rosQuat2Euler();

	tonoTrans= endeffTrans + orientDCM*transEeff2Tono;

	if (tonoTransInit.norm()==0) {
		tonoTransInit=tonoTrans;
		orientEulerInit=orientEuler;
	}
}

void RobotState::readForceSensor() {
	tonoForce<<(*forceATI)[0],(*forceATI)[1],(*forceATI)[2];// forceATI[1], forceATI[2];
	tonoForce=orientDCM.transpose()* rotEef2Ft.transpose()* tonoForce;
}

void RobotState::rosQuat2DCM() {
	float qx=pose->orientation.x;
	float qy=pose->orientation.y;
	float qz=pose->orientation.z;
	float qw=pose->orientation.w;

	orientDCM <<
		1 - 2*qy*qy - 2*qz*qz,	2*qx*qy - 2*qz*qw,	2*qx*qz + 2*qy*qw,
		2*qx*qy + 2*qz*qw,	1 - 2*qx*qx - 2*qz*qz,	2*qy*qz - 2*qx*qw,
		2*qx*qz - 2*qy*qw,	2*qy*qz + 2*qx*qw,	1 - 2*qx*qx - 2*qy*qy;
}

void RobotState::rosQuat2Euler() {
	RosQuat * q1 = &(pose->orientation);
	double heading, attitude, bank;
	double test = q1->x*q1->y + q1->z*q1->w;
	if (test > 0.499) { // singularity at north pole
		heading = 2 * atan2(q1->x,q1->w);
		attitude = PI/2;
		bank = 0;
		return;
	}
	if (test < -0.499) { // singularity at south pole
		heading = -2 * atan2(q1->x,q1->w);
		attitude = - PI/2;
		bank = 0;
		return;
	}
	double sqx = q1->x*q1->x;
	double sqy = q1->y*q1->y;
	double sqz = q1->z*q1->z;
	heading = atan2(2*q1->y*q1->w-2*q1->x*q1->z , 1 - 2*sqy - 2*sqz);
	attitude = asin(2*test);
	bank = atan2(2*q1->x*q1->w-2*q1->y*q1->z , 1 - 2*sqx - 2*sqz);
	orientEuler<<heading,attitude,bank;
}

Vector3f& RobotState::getTonometerPosition() {
	return tonoTrans;
}

Vector3f& RobotState::getEffPosition() {
	return endeffTrans;
}

Vector3f& RobotState::getTonometerInitPosition() {
	return tonoTransInit;
}

Vector3f& RobotState::getOrientationEuler() {
	return orientEuler;
}

Matrix3f& RobotState::getOrientationDCM() {
	return orientDCM;
}

Matrix3f& RobotState::getRotEef2FtDCM() {
	return rotEef2Ft;
}

Vector3f& RobotState::getOrientationEulerInit() {
	return orientEulerInit;
}

float RobotState::getNormalForce() {
	return tonoForce[2];
}

void RobotState::angles2matrix() {
	float heading	=cmdEuler[0];
	float attitude 	=cmdEuler[1];
	float bank		=cmdEuler[2];

	float ch = cos(heading);
	float sh = sin(heading);
	float ca = cos(attitude);
	float sa = sin(attitude);
	float cb = cos(bank);
	float sb = sin(bank);
	//cout<<"here?"<<endl;
	cmdDCM << ch * ca, sh*sb - ch*sa*cb, ch*sa*sb + sh*cb,
		sa, ca*cb, -ca*sb,
		-sh*ca, sh*sa*cb + ch*sb, -sh*sa*sb + ch*cb;
}

void RobotState::matrix2quat() {
	RosQuat q;// = &(pose->orientation);
	//Matrix3f a=rse.orientRot;
	float trace = cmdDCM(0,0) + cmdDCM(1,1) + cmdDCM(2,2); // I removed + 1.0f; see discussion with Ethan
	if( trace > 0 ) {// I changed M_EPSILON to 0
		float s = 0.5f / sqrtf(trace+ 1.0f);
		q.w = 0.25f / s;
		q.x = ( cmdDCM(2,1) - cmdDCM(1,2) ) * s;
		q.y = ( cmdDCM(0,2) - cmdDCM(2,0) ) * s;
		q.z = ( cmdDCM(1,0) - cmdDCM(0,1) ) * s;
	} else {
		if ( cmdDCM(0,0) > cmdDCM(1,1) && cmdDCM(0,0) > cmdDCM(2,2) ) {
			float s = 2.0f * sqrtf( 1.0f + cmdDCM(0,0) - cmdDCM(1,1) - cmdDCM(2,2));
			q.w = (cmdDCM(2,1) - cmdDCM(1,2) ) / s;
			q.x = 0.25f * s;
			q.y = (cmdDCM(0,1) + cmdDCM(1,0) ) / s;
			q.z = (cmdDCM(0,2) + cmdDCM(2,0) ) / s;
		} else if (cmdDCM(1,1) > cmdDCM(2,2)) {
			float s = 2.0f * sqrtf( 1.0f + cmdDCM(1,1) - cmdDCM(0,0) - cmdDCM(2,2));
			q.w = (cmdDCM(0,2) - cmdDCM(2,0) ) / s;
			q.x = (cmdDCM(0,1) + cmdDCM(1,0) ) / s;
			q.y = 0.25f * s;
			q.z = (cmdDCM(1,2) + cmdDCM(2,1) ) / s;
		} else {
			float s = 2.0f * sqrtf( 1.0f + cmdDCM(2,2) - cmdDCM(0,0) - cmdDCM(1,1) );
			q.w = (cmdDCM(1,0) - cmdDCM(0,1) ) / s;
			q.x = (cmdDCM(0,2) + cmdDCM(2,0) ) / s;
			q.y = (cmdDCM(1,2) + cmdDCM(2,1) ) / s;
			q.z = 0.25f * s;
		}
	}
	memcpy(&(cmd->orientation),&q,sizeof(RosQuat));
		//cart_pos_cmd_.orientation=q;
}

void RobotState::applyPoseCommand(Vector3f& dp, Vector3f& dphi, double dt) {
	if (dphi.norm() > MAX_ANG_VEL*dt){
		dphi.normalize();
		cmdEuler=orientEuler+dphi*MAX_ANG_VEL*dt;
	}
	else
		cmdEuler=orientEuler+dphi;

	// copy to ros state vars
	angles2matrix();

	if (dp.norm() > MAX_VEL*dt){
		dp.normalize();
		cmdTrans=tonoTrans+dp*MAX_VEL*dt - cmdDCM*transEeff2Tono;
	}
	else
		cmdTrans=tonoTrans+dp - cmdDCM*transEeff2Tono;

	//cout<<"cmd: "<<cmdTrans.transpose()<<endl;
	matrix2quat();
	cmd->position.x=cmdTrans[0];
	cmd->position.y=cmdTrans[1];
	cmd->position.z=cmdTrans[2];
}

void RobotState::applyPoseIncrement(Vector3f& dp, Vector3f& dphi, double dt, bool limit) {
	// rotate
	if (limit && (dphi.norm() > MAX_ANG_VEL*dt)){
		dphi.normalize();
		dphi*=MAX_ANG_VEL*dt;
		cmdEuler=orientEuler+dphi;
		//cout<<"l "<<dphi.transpose();
	}
	else
		cmdEuler=orientEuler+dphi;

	// copy to ros state vars
	angles2matrix();

	dp= tonoTrans+dp-(endeffTrans+cmdDCM*transEeff2Tono);
	//cout << dp.transpose() <<endl;

/*
	if (limit && (dp.norm() > MAX_VEL*dt)){
		//dp.normalize();
		dp.normalize();
		cmdTrans=endeffTrans+dp*MAX_VEL*dt;
	}
	else*/
		cmdTrans=endeffTrans+dp;

	//cout<<"cmd: "<<cmdTrans.transpose()<<endl;
	matrix2quat();
	cmd->position.x=cmdTrans[0];
	cmd->position.y=cmdTrans[1];
	cmd->position.z=cmdTrans[2];
}

