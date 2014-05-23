#ifndef OROCOS_tonometer_COMPONENT_HPP
#define OROCOS_tonometer_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>

#include <lwr_fri/typekit/Types.hpp>
#include <geometry_msgs/typekit/Types.hpp>
#include <rtt/os/TimeService.hpp>

#include <iostream>
#include <fstream>
#include <time.h>
#include <string>
#include <vector>

#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>

#include "RobotState.h"
#include "TonoProtocol.h"
#include "Simulator.h"

#include <Eigen/Geometry>

using namespace std;



class tonometer : public RTT::TaskContext{

	// helper classes
	RobotState * state;
	TonoProtocol * protocol;
	Simulator * sim;
	
	//Input Port
	InputPort<geometry_msgs::Pose> port_cart_pos_;
	InputPort<Vector6f> port_ft;
	OutputPort<geometry_msgs::Wrench> port_cart_wrench_cmd_;
	OutputPort<lwr_fri::CartesianImpedance> port_cart_imp_cmd_;
	OutputPort<geometry_msgs::Pose> port_cart_pos_cmd_;

	RosWrench cart_wrench_cmd_;
	lwr_fri::CartesianImpedance cart_imp_cmd_;

	RosPose cart_pos_;
	RosPose cart_pos_cmd_;
	Vector6f forceATI;

	float kp_;
	float kd_;
	float t_out_;

	// aditional state vars
	Vector3f errorTrans, errorRot, errorForce;
	Vector3f cmdTrans, cmdRot;
	timeval t_start, t_now, t_print, t_log, t_prev;

	// logging
	//std::vector<ofstream*> files;
	std::vector<ofstream*> files;


	public:
		tonometer(std::string const& name);
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();

	private:
		double getDeltaT(timeval & t_now, timeval & t_prev);
		void printData();
		void logData();
};
#endif
