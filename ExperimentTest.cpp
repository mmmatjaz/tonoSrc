/*
 * ExperimentTest.cpp
 *
 *  Created on: Apr 22, 2014
 *      Author: m
 */

#include "ExperimentTest.h"


ExperimentTest::ExperimentTest() {
	sim=new Simulator(&cart_pos_, &cart_pos_cmd_, &forceATI);
	state=new RobotState(&cart_pos_,&cart_pos_cmd_,&forceATI);
	protocol=new TonoProtocol();
	printData();
	shouldRun=true;

	files.push_back(new ofstream("logAll.log"));
	files.at(0)->precision(8);
}

bool ExperimentTest::startHook() {
	gettimeofday(&t_start, NULL);
	gettimeofday(&t_now, NULL);
	gettimeofday(&t_prev, NULL);
	gettimeofday(&t_log, NULL);
}

void ExperimentTest::updateHook() {
	gettimeofday(&t_now, NULL);
	double dt=getDeltaT(t_now,t_prev);
	memcpy(&t_prev,&t_now,sizeof(timeval));

	if (true) { // new data?

		state->readRosPose();
		state->readForceSensor();

		errorTrans=state->getTonometerInitPosition()-state->getTonometerPosition();
		errorRot=/*desired*/ state->getOrientationEulerInit()+protocol->getDesiredOrientation()
				/*feedback*/-state->getOrientationEuler();

		cmdTrans=errorTrans;  	if(cmdTrans.norm()>0)	cmdTrans.normalize();
		cmdRot=errorRot;		if(cmdRot.norm()>0)		cmdRot.normalize();

		if (protocol->isInPositionMode()) {
			if (errorTrans.norm()<SMALL_DISTANCE &&
							errorRot.norm()<SMALL_ANGLE) {
				protocol->setPositionReached();
				cmdTrans*=0;
				cmdRot*=0;
				cout<<"position reached "
					<<" fi: " << protocol->getDesiredOrientation().transpose()
					<<" F: " << protocol->getDesiredForce()<<endl;
			} else {
				cmdTrans=GAIN_TRANSLATORY * errorTrans;
				cmdRot  =GAIN_ANGULAR * errorRot;

				float errorForce=0.-(state->getNormalForce());
				if (fabs(errorForce)<SMALL_FORCE){
					//cmdTrans[2]=0;
				} else {
					//cmdTrans[2]=1e-5 * (errorForce>0 ? -1. : 1.);
				}
			}
		} else {
			if (protocol->isForceTrialFinished()) {
				cmdTrans*=0;
				cmdRot*=0;
				if (protocol->incrementTrial())
					cout<<"force trial finished"<<endl;
				else {
					cout<<"all done"<<endl;
					stop();
				}
			} else {
				//cmdTrans*=.5*SMALL_DISTANCE;//GAIN_TRANSLATORY * errorTrans;
				cmdTrans=GAIN_TRANSLATORY * errorTrans;
				//cmdRot  *=.5*SMALL_ANGLE;//GAIN_ANGULAR * errorRot;
				cmdRot  =GAIN_ANGULAR * errorRot;

				float errorForce=protocol->getDesiredForce()-(state->getNormalForce());

				// increase the timer if force error is small enough
				if (fabs(errorForce)<SMALL_FORCE){
					protocol->setForceReached(dt);
					//cmdTrans[2]=0;
				} else {
					//cmdTrans[2]=1e-5 * -1;// (errorForce>0 ? -1. : 1.);
				}

				// force tracking - P control
				cmdTrans[2]=-GAIN_FORCE*errorForce;

				// saturate command
				float maxIncrement=1e-3;
				if (fabs(cmdTrans[2]) > maxIncrement)
					cmdTrans[2]=maxIncrement * (errorForce>0 ? -1. : 1.);
			}
		}

		state->applyPoseIncrement(cmdTrans,cmdRot,dt,true);

		sim->simulateMotion();
		sim->simulateForce( state->getOrientationDCM(),
							state->getRotEef2FtDCM(),
							state->getTonometerPosition(),
							state->getTonometerInitPosition());

		double dtPrint=getDeltaT(t_now,t_print);
			if (dtPrint>=1.) {
			printData();
			memcpy(&t_print,&t_now,sizeof(timeval));
		}

		double dtLog=getDeltaT(t_now,t_log);
		if (dtLog>=.02) {
			//cout<<".";
			logData();
			memcpy(&t_log,&t_now,sizeof(timeval));
		}
	}
}


void ExperimentTest::cleanupHook() {
	for (int i=0; i<files.size();i++) {
		files.at(i)->close();
	}

}

void ExperimentTest::printData() {

	cout<<(protocol->isInPositionMode() ? "p" : "f")
		<<" fi: "<<state->getOrientationEuler().transpose()
		<<" pe: "<<state->getEffPosition().transpose()
		<<" F= " <<state->getNormalForce()<<endl;
	/*
	cout<<state->getOrientationDCM()<<endl
		<<cart_pos_.orientation
		<<state->getOrientationEuler().transpose()<<endl
		<<endl;*/
}

void ExperimentTest::logData() {
	int i=0;
	*(files.at(i))<<getDeltaT(t_now,t_start)<<" ";//<<endl;
	*(files.at(i))<<state->getTonometerPosition().transpose()<<" ";//<<endl;
	*(files.at(i))<<state->getEffPosition().transpose()<<" ";//<<endl;
	*(files.at(i))<<state->getOrientationEuler().transpose()<<" ";//<<endl;
	char tmp[12];
	sprintf(tmp," %5.3f",state->getNormalForce());
	*(files.at(i))<<tmp<<endl;
}


void ExperimentTest::loop() {
	ofstream logTonoPos;
	logTonoPos.open ("logtp.log");

	do {
		updateHook();
	} while (!isFinished());//error.norm()>.001 && errorPhi.norm()>.001);

	logTonoPos.close();
}

int main() {
	ExperimentTest t=ExperimentTest();
	t.startHook();
	t.loop();
	t.cleanupHook();
	//t.printData();
}

void ExperimentTest::stop() {
	shouldRun=false;
}

bool ExperimentTest::isFinished() {
	return !shouldRun;
}

double ExperimentTest::getDeltaT(timeval& t_now, timeval& t_prev) {
	return ((t_now.tv_sec  - t_prev.tv_sec) * 1000
			+(t_now.tv_usec - t_prev.tv_usec)/1000.0 + 0.5)/1000.;
}
