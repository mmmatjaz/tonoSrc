#include "tonometer.hpp"


tonometer::tonometer(std::string const& name) : TaskContext(name), kp_(1), kd_(0), t_out_(1),   t_disp_(0), t_last_(0){
	//Setup data ports
	this->addPort("CartesianPosition",port_cart_pos_);

	this->addPort("CartesianWrenchCommand", port_cart_wrench_cmd_);
	this->addPort("CartesianImpedanceCommand", port_cart_imp_cmd_);
	this->addPort("CartesianPositionCommand",port_cart_pos_cmd_);

	//Setup properties
	this->addProperty("kp",kp_);
	this->addProperty("kd",kd_);
	this->addProperty("T_out",t_out_);

	//FT sensor
	this->addPort("FT", port_ft);

	//sim=new Simulator(&cart_pos_, &cart_pos_cmd_, &forceATI);
	state=new RobotState(&cart_pos_,&cart_pos_cmd_,&forceATI);
	protocol=new TonoProtocol();

	files.push_back(new ofstream("logAll.log"));
	files.at(0)->precision(8);
}

bool tonometer::configureHook(){
	return true;
}

bool tonometer::startHook(){

	while(port_cart_pos_.read(cart_pos_)!=NewData) {
		std::cout << "Waiting for data from KUKA" << std::endl;
		usleep(1e6);
	}
	//Set Cartesian Impedance
	cart_imp_cmd_.stiffness.linear.x = 5000;
	cart_imp_cmd_.stiffness.linear.y = 5000;
	cart_imp_cmd_.stiffness.linear.z = 5000;
	cart_imp_cmd_.stiffness.angular.x = 50;
	cart_imp_cmd_.stiffness.angular.y = 50;
	cart_imp_cmd_.stiffness.angular.z = 50;
	cart_imp_cmd_.damping.linear.x = 1;
	cart_imp_cmd_.damping.linear.y = 1;
	cart_imp_cmd_.damping.linear.z = 1;
	cart_imp_cmd_.damping.angular.x = 1;
	cart_imp_cmd_.damping.angular.y = 1;
	cart_imp_cmd_.damping.angular.z = 1;
	port_cart_imp_cmd_.write(cart_imp_cmd_);

	//Initialize outputs
	cart_wrench_cmd_.force.x = 0;
	cart_wrench_cmd_.force.y = 0;
	cart_wrench_cmd_.force.z = 0;
	cart_wrench_cmd_.torque.x = 0;
	cart_wrench_cmd_.torque.y = 0;
	cart_wrench_cmd_.torque.z = 0;

/*
	//Assume zero initial velocity
	for (size_t ii(0); ii < 3; ++ii) {
		cart_vel_.push_back(0);
	}
*/
	//Time setup
	gettimeofday(&t_start, NULL);
	gettimeofday(&t_now, NULL);
	gettimeofday(&t_prev, NULL);
	gettimeofday(&t_log, NULL);

	port_cart_imp_cmd_.write(cart_imp_cmd_);
	port_cart_wrench_cmd_.write(cart_wrench_cmd_);
	port_cart_pos_cmd_.write(cart_pos_);

	return true;
}

void tonometer::updateHook(){
	gettimeofday(&t_now, NULL);
	double dt=getDeltaT(t_now,t_prev);
	memcpy(&t_prev,&t_now,sizeof(timeval));


	if(port_cart_pos_.read(cart_pos_) == NewData) {

		port_ft.read(forceATI);

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
				cmdTrans=GAIN_TRANSLATORY * errorTrans;
				cmdRot  =GAIN_ANGULAR * errorRot;

				float errorForce=protocol->getDesiredForce()-(state->getNormalForce());
				//cmdTrans[2]=GAIN_FORCE*errorForce;

				if ((errorForce)<SMALL_FORCE){
					protocol->setForceReached(dt);
					cmdTrans[2]=0;
				} else {
					cmdTrans[2]=1e-3 * -1;//(errorForce>0 ? -1. : 1.);
				}
			}
		}


		state->applyPoseIncrement(cmdTrans,cmdRot,dt,true);

	

		//Write Desired Position
		port_cart_pos_cmd_.write(cart_pos_cmd_);
		port_cart_imp_cmd_.write(cart_imp_cmd_);
		port_cart_wrench_cmd_.write(cart_wrench_cmd_);


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

void tonometer::stopHook() {

}

void tonometer::cleanupHook() {
	for (int i=0; i<files.size();i++) {
		files.at(i)->close();
	}
}

void tonometer::printData() {

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

void tonometer::logData() {
	int i=0;
	*(files.at(i))<<getDeltaT(t_now,t_start)<<" ";//<<endl;
	*(files.at(i))<<state->getTonometerPosition().transpose()<<" ";//<<endl;
	*(files.at(i))<<state->getEffPosition().transpose()<<" ";//<<endl;
	*(files.at(i))<<state->getOrientationEuler().transpose()<<" ";//<<endl;
	char tmp[12];
	sprintf(tmp," %5.3f",state->getNormalForce());
	*(files.at(i))<<tmp<<endl;
}

double tonometer::getDeltaT(timeval& t_now, timeval& t_prev) {
	return ((t_now.tv_sec  - t_prev.tv_sec) * 1000
			+(t_now.tv_usec - t_prev.tv_usec)/1000.0 + 0.5)/1000.;
}
/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(tonometer)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(tonometer)

