/*
 * TonoProtocol.cpp
 *
 *  Created on: Apr 16, 2014
 *      Author: m
 */

#include "TonoProtocol.h"

float TonoProtocol::trialForceList[]=TRIAL_FORCES
float TonoProtocol::trialRotList[]=TRIAL_ANGLES

TonoProtocol::TonoProtocol() {
	trialTime=0.;
	controlMode=CONTROL_POS;
	trialForceIndex=0;
	trialRotIndex=0;
	finished=0;
	trialRot[0]=trialRotList[trialRotIndex];
}

int TonoProtocol::isInForceMode() {
	return controlMode==CONTROL_FORCE;
}

int TonoProtocol::isInPositionMode() {
	return controlMode==CONTROL_POS;
}

void TonoProtocol::setPositionReached() {
	controlMode=CONTROL_FORCE;
}

int TonoProtocol::isForceTrialFinished() {
	return trialTime>=TRIAL_DURATION || (getDesiredForce()==0 && trialTime>=1);
}

void TonoProtocol::setForceReached(float dt) {
	trialTime+=dt;
}

int TonoProtocol::incrementTrial() {
	trialTime=0.;
	//cout << "force held for 7s, ending trial" << endl;
	if (trialForceIndex<sizeof(trialForceList)/sizeof(float)-1){
		trialForceIndex++;
		//trialForce[3]=-1*trialForceList[trialForceIndex];
		return 1;
	}
	else {
		// all forces at this pose tested, move to next pose or finish
		if (trialRotIndex<sizeof(trialRotList)/sizeof(float)-1) {
			controlMode=CONTROL_POS;
			trialRotIndex++;
			trialForceIndex=0;
			trialRot[0]=trialRotList[trialRotIndex];
			//cout << "all forces tested, move to next position" << endl;
			return 1;
		}
		else {
			return 0;
		}
	}
}

Vector3f& TonoProtocol::getDesiredOrientation() {
	return trialRot;
}

float TonoProtocol::getDesiredForce() {
	return trialForceList[trialForceIndex];
}
