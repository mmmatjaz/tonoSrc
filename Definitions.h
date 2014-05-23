/*
 * Definitions.h
 *
 *  Created on: Apr 24, 2014
 *      Author: m
 */

#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

#define PI 3.141

#define TONO_TRANS_X 0.0185
#define TONO_TRANS_Y 0.
#define TONO_TRANS_Z 0.215
#define MAX_VEL	1e-2
#define MAX_ANG_VEL 0.1 // rad/s
#define FT_Z		1.	// 1 if z aligned to eff
#define FT_ROT_Z	0.	// rot around z - mind the FT_Z !


// these are in TonoProtocol.cpp and TonoProtocol.h
#define SMALL_ANGLE			5e-3 // rad
#define SMALL_DISTANCE		1e-4 // m
#define SMALL_FORCE			1e-1 // N
#define GAIN_ANGULAR		1e-2 //
#define GAIN_TRANSLATORY	1e-1 //
#define GAIN_FORCE			2e-6 // m/N



typedef Eigen::Matrix<float, 3, 1> Vector3f;
typedef Eigen::Matrix<float, 6, 1> Vector6f;

typedef geometry_msgs::Pose RosPose;
typedef geometry_msgs::Quaternion RosQuat;
typedef geometry_msgs::Wrench RosWrench;

using namespace Eigen;
using namespace std;


#endif /* DEFINITIONS_H_ */
