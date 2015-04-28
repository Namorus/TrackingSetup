/*
 * forward_calc.cpp
 *
 *  Created on: Mar 31, 2015
 *      Author: thomas
 */
#include <sys/time.h>
#include <sstream>

#include <iostream> 			//
#include <armadillo> 			// matrix library

//#include <trackingsetup/forward_calc.h>
#include <trackingsetup/target_pos_est.h>

//using namespace arma;

namespace tracking {

ForwardCalc::ForwardCalc() {
	// TODO Auto-generated constructor stub

}

ForwardCalc::~ForwardCalc() {
	// TODO Auto-generated destructor stub
}

void ForwardCalc::predictEstimate(){ // predicts position and velocity from given data (old velocity, old position, old roll-angle,...)
// define state vector (matrix library)
//vec xhat(6);
//xhat << targetPosLocal_.y << endr
//	 << targetGlobalPos_.velocity.lat << endr
//	 << targetPosLocal_.x  << endr
//	 << targetGlobalPos_.velocity.lon << endr
//	 << targetPosLocal_.z << endr
//	 << -targetGlobalPos_.velocity.alt << endr;

double gravity = 9.81;
double phi;
double var_phi;
double dt;

// 0. calculation of turn rate omega ----------------------------------------------
	double omega = tan(phi)*gravity/sqrt(xhat(1)^2+xhat(3)^2+xhat(5)^2);

// TODO: add wind calculation,... here!!!!!!

// 1. compute F matrix  -----------------------------------------------------------

// in continuous time

mat F_c(6,6);
F_c << 0 << 1 << 0 << 0 << 0 << 0 << endr
	<< 0 << gravity*xhat(1)*xhat(3)*tan(phi)/(xhat(1)^2+xhat(3)^2+xhat(5)^2)^(3./2.) << 0 << gravity*xhat(3)^2*tan(phi)/(xhat(1)^2+xhat(3)^2+xhat(5)^2)^(3./2.)-gravity*tan(phi)/sqrt(xhat(1)^2+xhat(3)^2+xhat(5)^2) << 0 << gravity*xhat(3)*xhat(5)*tan(phi)/(xhat(1)^2+xhat(3)^2+xhat(5)^2)^(3./2.) << endr
	<< 0 << 0 << 0 << 1 << 0 << 0 << endr
    << 0 << gravity*tan(phi)/sqrt(xhat(1)^2+xhat(3)^2+xhat(5)^2)-gravity*xhat(1)^2*tan(phi)/(xhat(1)^2+xhat(3)^2+xhat(5)^2)^(3./2.) << 0 << -gravity*xhat(1)*xhat(3)*tan(phi)/(xhat(1)^2+xhat(3)^2+xhat(5)^2)^(3./2.) << 0 << -gravity*xhat(1)*xhat(5)*tan(phi)/(xhat(1)^2+xhat(3)^2+xhat(5)^2)^(3./2.) << endr
	<< 0 << 0 << 0 << 0 << 0 << 1 << endr
	<< 0 << 0 << 0 << 0 << 0 << 0 << endr;

// to discrete time
mat Id_6(6,6);
Id_6.eye();

mat F_d = Id_6+F_c*dt+(F_c*F_c)*dt^2/2.;

// 2. compute measurement covariance matrix Q  -----------------------------------------------------------

vec b(6);
b   << 0 << endr
	 << -4.*gravity*xhat(3)*cos(phi)^2./((cos(2.*phi)+1)^2.*sqrt(xhat(1)^2+xhat(3)^2+xhat(5)^2.)) << endr
	 << 0 << endr
	 << 4*gravity*xhat(1)*cos(phi)^2./((cos(2.*phi)+1)^2.*sqrt(xhat(1)^2+xhat(3)^2+xhat(5)^2)) << endr
	 << 0 << endr
	 << 0 << endr;

mat Q_c = b*var_phi*b.t();

mat R(12,12);
R 	<< -F_c << Q_c << endr
	<< Id_6 << F_c.t() << endr;

// to discrete time
mat Id_12(12,12);
Id_12.eye();

mat G = Id_12+R*dt+(R*R)*dt^2/2.;

mat F_dTQ_d(6,6);
F_dTQ_d << G(0,6) << G(0,7) << G(0,8) << G(0,9) << G(0,10) <<  G(0,11) << endr
		<< G(1,6) << G(1,7) << G(1,8) << G(1,9) << G(1,10) <<  G(1,11) << endr
		<< G(2,6) << G(2,7) << G(2,8) << G(2,9) << G(2,10) <<  G(2,11) << endr
		 << G(3,6) << G(3,7) << G(3,8) << G(3,9) << G(3,10) <<  G(3,11) << endr
		 << G(4,6) << G(4,7) << G(4,8) << G(4,9) << G(4,10) <<  G(4,11) << endr
		 << G(5,6) << G(5,7) << G(5,8) << G(5,9) << G(5,10) <<  G(5,11) << endr;

mat F_dT(6,6);
F_dT  << G(6,6) << G(6,7) << G(6,8) << G(6,9) << G(6,10) <<  G(6,11) << endr
		<< G(7,6) << G(7,7) << G(7,8) << G(7,9) << G(7,10) <<  G(7,11) << endr
		<< G(8,6) << G(8,7) << G(8,8) << G(8,9) << G(8,10) <<  G(8,11) << endr
		 << G(9,6) << G(9,7) << G(9,8) << G(9,9) << G(9,10) <<  G(9,11) << endr
		 << G(10,6) << G(10,7) << G(10,8) << G(10,9) << G(10,10) <<  G(10,11) << endr
		 << G(11,6) << G(11,7) << G(11,8) << G(11,9) << G(11,10) <<  G(11,11) << endr;

mat Q_d(6,6);
Q_d = F_dT.t()*F_dTQ_d;

mat Q_additional(6,6);
Q_additional << dt^4/4.*q << dt^3/2.*q << 0 << 0 << 0 << 0 << endr
			 << dt^3/2.*q << dt^2*q << 0 << 0 << 0 << 0 << endr
			 << 0 << 0 << dt^4/4.*q << dt^3/2.*q << 0 << 0 <<  endr
			 << 0 << 0 << dt^3/2.*q << dt^2*q << 0 << 0 << endr
			 << 0 << 0 << 0 << 0 << dt^3/2.*q << dt^2*q << endr
			 << 0 << 0 << 0 << 0 << dt^2*q << dt*q << endr;

mat Q = Q_d + Q_additional;

// 3.1 Predict state estimate (a priori) -----------------------------------------------------------
vec xhat_new(6);
xhat_new << xhat(1)*dt << endr
		<< -omega*xhat(3)*dt << endr
		<< xhat(3)*dt << endr
		<< omega*xhat(1)*dt << endr
		<< xhat(5)*dt << endr
		<< 0 << endr;

xhat_new = xhat + xhat_new;

// 3.2 Predicted (a priori) estimate covariance -----------------------------------------------------------

mat P_new(6,6);
P_new = F_d*P*F_d.t()+Q;

// transform from NED to ENU
targetEstimatedVel_.x = xhat(3);		//targetGlobalPos_.velocity.lon;
targetEstimatedVel_.y = xhat(1);		//targetGlobalPos_.velocity.lat;
targetEstimatedVel_.z = xhat(5); 		//-targetGlobalPos_.velocity.alt;

// update position
targetEstimatedPosLocal_.x = xhat(2);	//targetPosLocal_.x + targetEstimatedVel_.x*dT*1e-6;
targetEstimatedPosLocal_.y = xhat(0);	//targetPosLocal_.y + targetEstimatedVel_.y*dT*1e-6;
targetEstimatedPosLocal_.z = xhat(4);	//targetPosLocal_.z + targetEstimatedVel_.z*dT*1e-6;

// convert back to WGS84
antennaLocalCartesian_.Reverse(targetEstimatedPosLocal_.x,targetEstimatedPosLocal_.y,targetEstimatedPosLocal_.z,targetEstimatedPos_.lat,targetEstimatedPos_.lon,targetEstimatedPos_.alt);

std::stringstream logmessage;
logmessage << "predict local pos: " << targetEstimatedPosLocal_ << ", local vel: " << targetEstimatedVel_;
addLogMessage(vl_DEBUG,logmessage.str());
}


void ForwardCalc::updateEstimate() { // updates latest position estimate by new ground position, velocity
	// define measurement vector Z (matrix library)
	vec Z(6);
	Z << targetPosLocal_.y << endr
		 << targetGlobalPos_.velocity.lat << endr
		 << targetPosLocal_.x  << endr
		 << targetGlobalPos_.velocity.lon << endr
		 << targetPosLocal_.z << endr
		 << -targetGlobalPos_.velocity.alt << endr;
// 1. Calculate R and H -----------------------------------------------------------

mat R(6,6);
R << r_a^2 << 0 << 0 << 0 << 0 << 0 << endr
	<< 0 << r_b^2 << 0 << 0 << 0 << 0 << endr
	<< 0 << 0 << r_a^2 << 0 << 0 << 0 << endr
	<< 0 << 0 << 0 << r_b^2 << 0 << 0 << endr
	<< 0 << 0 << 0 << 0 << r_a^2 << 0 << endr
	<< 0 << 0 << 0 << 0 << 0 << r_b^2 << endr;

mat H(6,6);
H.eye();

// 2.1 Measurement residual -----------------------------------------------------------
vec Y(6);
Y = Z-H*xhat;

// 2.2 Residual covariance -----------------------------------------------------------

mat S = H*P*H.t() + R;

// 2.3 Optimal Kalman gain -----------------------------------------------------------

mat K = P*H.t()/S;

// 2.4 Update (a posteriori) state estimate ------------------------------------------

xhat_new = xhat + K*Y;

// 2.5 Update (a posteriori) estimate covariance --------------------------------------
mat Id_6(6,6);
Id_6.eye();

P_new =(Id_6-K*H)*P;

	// calculate time since last GPOS update
	//timeval now;
	//gettimeofday(&now,NULL);
	//uint64_t currentTimestamp = now.tv_sec*1e6 + now.tv_usec;
	//uint32_t dT = currentTimestamp - targetGlobalPos_.localTimestamp;

// velocity estimate is previous groundspeed measurement
// transform from NED to ENU
targetEstimatedVel_.x = xhat(3);		//targetGlobalPos_.velocity.lon;
targetEstimatedVel_.y = xhat(1);		//targetGlobalPos_.velocity.lat;
targetEstimatedVel_.z = xhat(5); 		//-targetGlobalPos_.velocity.alt;

// update based on ground speed
targetEstimatedPosLocal_.x = xhat(2);	//targetPosLocal_.x + targetEstimatedVel_.x*dT*1e-6;
targetEstimatedPosLocal_.y = xhat(0);	//targetPosLocal_.y + targetEstimatedVel_.y*dT*1e-6;
targetEstimatedPosLocal_.z = xhat(4);	//targetPosLocal_.z + targetEstimatedVel_.z*dT*1e-6;

// convert back to WGS84
antennaLocalCartesian_.Reverse(targetEstimatedPosLocal_.x,targetEstimatedPosLocal_.y,targetEstimatedPosLocal_.z,targetEstimatedPos_.lat,targetEstimatedPos_.lon,targetEstimatedPos_.alt);

	std::stringstream logmessage;
	logmessage << "update local pos: " << targetEstimatedPosLocal_ << ", local vel: " << targetEstimatedVel_;
	addLogMessage(vl_DEBUG,logmessage.str());
}

} /* namespace tracking */
