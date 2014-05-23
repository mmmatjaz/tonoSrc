/*
 * RobotState.h
 *
 *  Created on: Apr 16, 2014
 *      Author: m
 */

#ifndef ROBOTSTATE_H_
#define ROBOTSTATE_H_

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Wrench.h>
#include <Eigen/Geometry>
#include <math.h>

#include "Definitions.h"



/**
 * Holds the robot pose state and methods to control the tonomoter pose rather than
 * endeffector pose.
 */
class RobotState {
private:
	/** Pointer to ROS/orocos endeffector readings and commands
	 * that live in the orocos module */
	RosPose * pose;
	RosPose * cmd;
	/** FT sensor data in FT frame*/
	Vector6f * forceATI;

	/** translation from endeffector to tonometer tip */
	Vector3f transEeff2Tono;

	/** robot and tonometer pose */
	Vector3f endeffTrans;
	Vector3f tonoTrans;
	Vector3f tonoTransInit;
	Matrix3f orientDCM;
	Vector3f orientEuler;
	Vector3f orientEulerInit;

	/** endEff command */
	Vector3f cmdTrans;
	Vector3f cmdEuler;
	Matrix3f cmdDCM;

	/** FT force data in world frame*/
	Vector3f tonoForce;
	Matrix3f rotEef2Ft;// force in eef frame=rotEff2Ft.transpose() * Fft
	/** transforms ROS quat orientation to eigen matrix */
	void rosQuat2DCM();
	/** transforms ROS quat orientation to euler angles as eigen vector */
	void rosQuat2Euler();
	/** increments the pose */
	//void setDesiredPose();
	void angles2matrix();
	void matrix2quat();

public:
	/** Takes pointers to orocos module state objects */
	RobotState(RosPose * meas, RosPose * cmd, Vector6f * ft);
	/** reads ROS data and does conversions to tonometer tip */
	void readRosPose();
	/** reads and transforms force from FT sensor to world frame */
	void readForceSensor();

	/** returns reference to eff position (read only)*/
	Vector3f& getEffPosition();
	/** returns reference to position (read only)*/
	Vector3f& getTonometerPosition();
	/** returns reference to initial position (read only)*/
	Vector3f& getTonometerInitPosition();
	/** returns reference to orientation (read only)*/
	Vector3f& getOrientationEuler();
	/** returns reference to orientation (read only)*/
	Matrix3f& getOrientationDCM();
	/** returns reference to orientation (read only)*/
	Matrix3f& getRotEef2FtDCM();
		/** returns reference to init orientation (read only)*/
	Vector3f& getOrientationEulerInit();

	float getNormalForce();

	/** Increments the command by delta pose dp and dphi and applies the velocity limit */
	void applyPoseCommand(Vector3f &dp, Vector3f &dphi, double dt);

	/** Increments the command by delta pose dp and dphi and applies the velocity limit */
	void applyPoseIncrement(Vector3f &dp, Vector3f &dphi, double dt, bool limit);

	void printData();

};

#endif /* ROBOTSTATE_H_ */
