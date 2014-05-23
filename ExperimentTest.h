/*
 * ExperimentTest.h
 *
 *  Created on: Apr 22, 2014
 *      Author: m
 */

#ifndef EXPERIMENTTEST_H_
#define EXPERIMENTTEST_H_

#include <iostream>
#include <fstream>
#include <time.h>
#include <string>
#include <vector>

#include <ctime>
#include <iomanip>

#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>

#include "RobotState.h"
#include "TonoProtocol.h"
#include "Simulator.h"

using namespace std;
class ExperimentTest {
private:
	// helper classes
	RobotState * state;
	TonoProtocol * protocol;
	Simulator * sim;

	// orocos data
	RosPose cart_pos_;
	RosPose cart_pos_cmd_;
	Vector6f forceATI;

	// aditional state vars
	Vector3f errorTrans, errorRot, errorForce;
	Vector3f cmdTrans, cmdRot;
	timeval t_start, t_now, t_print, t_log, t_prev;

	// logging
	//std::vector<ofstream*> files;
	std::vector<ofstream*> files;

	bool shouldRun;

	double getDeltaT(timeval & t_now, timeval & t_prev);
	void printData();
	void logData();
public:
	ExperimentTest();
	bool startHook();
	void updateHook();
	void cleanupHook();
	void loop();
	void stop();
	bool isFinished();

};

#endif /* EXPERIMENTTEST_H_ */
