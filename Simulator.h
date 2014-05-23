/*
 * Simulator.h
 *
 *  Created on: Apr 17, 2014
 *      Author: m
 */

#ifndef SIMULATOR_H_
#define SIMULATOR_H_

#include <ros/geometry_msgs/Pose.h>
#include <ros/geometry_msgs/Quaternion.h>
#include <ros/geometry_msgs/Wrench.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <math.h>

#define SIM_STIFF 1e3
#define SIM_ARM_POS .03

typedef Eigen::Matrix<float, 3, 1> Vector3f;
typedef Eigen::Matrix<float, 6, 1> Vector6f;

typedef geometry_msgs::Pose RosPose;
typedef geometry_msgs::Quaternion RosQuat;
typedef geometry_msgs::Wrench RosWrench;

using namespace Eigen;

class Simulator {
private:
	RosPose * pose;
	RosPose * cmd;
	Vector6f * forceATI;

	/** temp eigen force vector */
	Vector3f f;
public:
	Simulator(RosPose * meas, RosPose * cmd, Vector6f * ft);
	void simulateMotion();
	void simulateForce(Matrix3f & rotTono,Matrix3f & rotFt, Vector3f& tonoPos, Vector3f& tonoInitPos);
};

#endif /* SIMULATOR_H_ */
