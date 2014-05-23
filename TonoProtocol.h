/*
 * TonoProtocol.h
 *
 *  Created on: Apr 16, 2014
 *      Author: m
 */

#ifndef TONOPROTOCOL_H_
#define TONOPROTOCOL_H_

#include <Eigen/Geometry>

#define TRIAL_FORCES {0.5, 2., 4., 0};
#define TFN 4
//#define TRIAL_ANGLES {-.4, -.8};
//#define TAN 2
#define TRIAL_ANGLES {-.0};
#define TAN 1
#define PI 3.141
typedef Eigen::Matrix<float, 3, 1> Vector3f;
typedef Eigen::Matrix<float, 6, 1> Vector6f;

class TonoProtocol {
private:
	int controlMode;

	//Vector6f trialForce;
	int trialForceIndex;

	Vector3f trialRot;
	int trialRotIndex;

	static const float TRIAL_DURATION=7.;
	float trialTime;
	int finished;
public:
	static float trialForceList[TFN];
	static float trialRotList[TAN];
	static const int CONTROL_POS=1;
	static const int CONTROL_FORCE=2;


	TonoProtocol();
	int isInPositionMode();
	int isInForceMode();
	void setPositionReached();
	int isForceTrialFinished();
	void setForceReached(float dt);
	//void endTrial();
	int incrementTrial();

	Vector3f& getDesiredOrientation();
	float getDesiredForce();
};

#endif /* TONOPROTOCOL_H_ */
