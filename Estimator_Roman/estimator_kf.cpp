/*
 * estimator_kf.cpp
 *
 *  Created on: May 10, 2015
 *      Author: Roman
 */
#include <sys/time.h>
#include <sstream>
#include <trackingsetup/estimator_kf.h>

using namespace arma;

namespace tracking {

/*
EstimatorKF::EstimatorKF() {
	// TODO Auto-generated constructor stub

}

EstimatorKF::~EstimatorKF() {
	// TODO Auto-generated destructor stub
}
*/


void EstimatorKF::KFpredictEstimate() { // predicts position and velocity from given data (old velocity, old position, old roll-angle,...)

// 0.1 calculation of turn rate omega ----------------------------------------------
	double v_total2 = pow(xhat(1),2) + pow(xhat(3),2) + pow(xhat(5),2); //calculate total velocity^2

	double omega = tan(KFphi) * gravity / sqrt(v_total2);

// TODO 0.2 calculation q according to wind ----------------------------------------------

	//double v_wind = sqrt(v_total2) - fabs(v_air);
	//float a_LS = 0.066;
	//float b_LS = 0.07;

	//double q_add = a_LS * v_wind + b_LS + b_LS * fabs(phi);

	//q = q + q_add;

// 1. compute F matrix  -----------------------------------------------------------

// in continuous time

	mat F_c(6, 6);
	F_c << 0 << 1 << 0 << 0 << 0 << 0 << endr
		<< 0 << gravity * xhat(1) * xhat(3) * tan(KFphi)/ pow(v_total2,(3. / 2.)) << 0 << gravity * pow(xhat(3),2) * tan(KFphi) / pow(v_total2,(3. / 2.)) - gravity * tan(KFphi) / sqrt(v_total2) << 0 << gravity * xhat(3) * xhat(5) * tan(KFphi)/ pow(v_total2,(3. / 2.)) << endr
		<< 0 << 0 << 0 << 1 << 0 << 0 << endr
		<< 0 << gravity * tan(KFphi) / sqrt(v_total2) - gravity * pow(xhat(1),2) * tan(KFphi) / pow(v_total2,(3. / 2.)) << 0<< -gravity * xhat(1) * xhat(3) * tan(KFphi)/ pow(v_total2,(3. / 2.)) << 0 << -gravity * xhat(1) * xhat(5) * tan(KFphi)/ pow(v_total2,(3. / 2.)) << endr
		<< 0 << 0 << 0 << 0 << 0 << 1 << endr
		<< 0 << 0 << 0 << 0 << 0 << 0 << endr;

// to discrete time
	mat Id_6(6, 6);
	Id_6.eye();

	mat F_d = Id_6 + F_c * dt + pow((F_c * F_c) * dt, 2) / 2.;

// 2. compute measurement covariance matrix Q  -----------------------------------------------------------

	vec b(6);
	b 	<< 0 << endr
		<< -4.0 * gravity * xhat(3) * pow(cos(KFphi), 2) / (pow((cos(2. * KFphi) + 1),2)* sqrt(v_total2)) << endr
		<< 0 << endr
		<< 4.0 * gravity * xhat(1) * pow(cos(KFphi),2) / (pow((cos(2. * KFphi) + 1),2) * sqrt(v_total2)) << endr
		<< 0<< endr
		<< 0 << endr;

	mat Q_c = b * var_phi * b.t();

	mat Zero_6(6,6);	// 6x6 matrix of zeros
	Zero_6.zeros();

	mat X(6,12);
	X = join_horiz(-F_c, Q_c);

	mat Y(6,12);
	Y = join_horiz(Zero_6,F_c.t());


	mat R(12, 12);
	R = join_vert(X,Y);

// to discrete time
	mat Id_12(12, 12);
	Id_12.eye();

	mat G = Id_12 + R * dt + pow((R * R) * dt,2) / 2.;

	mat F_dTQ_d(6, 6);
	F_dTQ_d << G(0, 6) << G(0, 7) << G(0, 8) << G(0, 9) << G(0, 10) << G(0, 11) << endr
			<< G(1, 6) << G(1, 7) << G(1, 8) << G(1, 9) << G(1, 10) << G(1, 11) << endr
			<< G(2, 6) << G(2, 7) << G(2, 8) << G(2, 9) << G(2, 10) << G(2, 11) << endr
			<< G(3, 6) << G(3, 7) << G(3, 8) << G(3, 9) << G(3, 10) << G(3, 11) << endr
			<< G(4, 6) << G(4, 7) << G(4, 8) << G(4, 9) << G(4, 10) << G(4, 11) << endr
			<< G(5, 6) << G(5, 7) << G(5, 8) << G(5, 9) << G(5, 10) << G(5, 11) << endr;

	mat F_dT(6, 6);
	F_dT 	<< G(6, 6) << G(6, 7) << G(6, 8) << G(6, 9) << G(6, 10) << G(6, 11) << endr
			<< G(7, 6) << G(7, 7) << G(7, 8) << G(7, 9) << G(7, 10) << G(7, 11) << endr
			<< G(8, 6) << G(8, 7) << G(8, 8) << G(8, 9) << G(8, 10) << G(8, 11) << endr
			<< G(9, 6) << G(9, 7) << G(9, 8) << G(9, 9) << G(9, 10) << G(9, 11) << endr
			<< G(10, 6) << G(10, 7) << G(10, 8) << G(10, 9) << G(10, 10) << G(10, 11) << endr
			<< G(11, 6) << G(11, 7) << G(11, 8) << G(11, 9) << G(11, 10) << G(11, 11) << endr;

	mat Q_d(6, 6);
	Q_d = F_dT.t() * F_dTQ_d;

	arma::mat Q_additional(6, 6);
	Q_additional << pow(dt,4) / 4. * q << pow(dt, 3) / 2. * q << 0 << 0 << 0 << 0 << endr
			<< pow(dt, 3) / 2. * q << pow(dt, 2) * q << 0 << 0 << 0 << 0 << endr
			<< 0 << 0 << pow(dt, 4) / 4. * q << pow(dt , 3) / 2. * q << 0 << 0 << endr
			<< 0 << 0 << pow(dt, 3) / 2. * q << pow(dt, 2) * q << 0 << 0 << endr
			<< 0 << 0 << 0 << 0 << pow(dt, 3) / 2. * q << pow(dt, 2) * q << endr
			<< 0 << 0 << 0 << 0 << pow(dt, 2) * q << dt * q << endr;

	mat Q = Q_d + Q_additional;

// 3.1 Predict state estimate (a priori) -----------------------------------------------------------
	vec xhat_new(6);
	xhat_new << xhat(1) * dt << endr
			<< -omega * xhat(3) * dt << endr
			<< xhat(3) * dt << endr
			<< omega * xhat(1) * dt << endr
			<< xhat(5) * dt << endr
			<< 0 << endr;

	xhat = xhat + xhat_new;

// 3.2 Predicted (a priori) estimate covariance -----------------------------------------------------------

	P = F_d * P * F_d.t() + Q;

	/*
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
	 */

}

void EstimatorKF::KFupdateEstimate() { // updates latest position estimate by new ground position, velocity

// 1. Calculate R and H -----------------------------------------------------------

	arma::mat R(6, 6);
	R 	<< pow(r_a, 2) << 0 << 0 << 0 << 0 << 0 << endr
		<< 0 << pow(r_b, 2) << 0 << 0 << 0 << 0 << endr
		<< 0 << 0 << pow(r_a, 2) << 0 << 0 << 0 << endr
		<< 0 << 0 << 0 << pow(r_b, 2) << 0 << 0 << endr
		<< 0 << 0 << 0 << 0 << pow(r_a, 2) << 0 << endr
		<< 0 << 0 << 0 << 0 << 0 << pow(r_b, 2) << endr;

	mat H(6, 6);
	H.eye();

// 2.1 Measurement residual -----------------------------------------------------------
	vec Y(6);
	Y = GPS_pos_vel - H * xhat;

// 2.2 Residual covariance -----------------------------------------------------------

	mat S = H * P * H.t() + R;

// 2.3 Optimal Kalman gain -----------------------------------------------------------

	mat K = (P * H.t()) / S;

// 2.4 Update (a posteriori) state estimate ------------------------------------------

	xhat = xhat + K * Y;

// 2.5 Update (a posteriori) estimate covariance --------------------------------------
	mat Id_6(6, 6);
	Id_6.eye();

	P = (Id_6 - K * H) * P;

	// calculate time since last GPOS update
	//timeval now;
	//gettimeofday(&now,NULL);
	//uint64_t currentTimestamp = now.tv_sec*1e6 + now.tv_usec;
	//uint32_t dT = currentTimestamp - targetGlobalPos_.localTimestamp;

// velocity estimate is previous groundspeed measurement
// transform from NED to ENU
//targetEstimatedVel_.x = xhat(3);		//targetGlobalPos_.velocity.lon;
//targetEstimatedVel_.y = xhat(1);		//targetGlobalPos_.velocity.lat;
//targetEstimatedVel_.z = xhat(5); 		//-targetGlobalPos_.velocity.alt;

// update based on ground speed
//targetEstimatedPosLocal_.x = xhat(2);	//targetPosLocal_.x + targetEstimatedVel_.x*dT*1e-6;
//targetEstimatedPosLocal_.y = xhat(0);	//targetPosLocal_.y + targetEstimatedVel_.y*dT*1e-6;
//targetEstimatedPosLocal_.z = xhat(4);	//targetPosLocal_.z + targetEstimatedVel_.z*dT*1e-6;

// convert back to WGS84
//antennaLocalCartesian_.Reverse(targetEstimatedPosLocal_.x,targetEstimatedPosLocal_.y,targetEstimatedPosLocal_.z,targetEstimatedPos_.lat,targetEstimatedPos_.lon,targetEstimatedPos_.alt);

	//std::stringstream logmessage;
	//logmessage << "update local pos: " << targetEstimatedPosLocal_ << ", local vel: " << targetEstimatedVel_;
	//addLogMessage(vl_DEBUG,logmessage.str());
}

void EstimatorKF::KF_pos_attitude(){	// case new position & attitude available

	// set new inputs
	GPS_pos_vel << targetPosLocal_.y << endr
			<< targetGlobalPos_.velocity.lat << endr
			<< targetPosLocal_.x << endr
			<< targetGlobalPos_.velocity.lon << endr
			<< targetPosLocal_.z << endr
			<< targetGlobalPos_.velocity.alt << endr;

	phi_current=targetAttitude_.roll;

	// order timestamps
	if (targetGlobalPos_.localTimestamp > targetAttitude_.localTimestamp) { // check if position or attitude was first
		double timestamp_1 = targetAttitude_.localTimestamp;
		double timestamp_2 = targetGlobalPos_.localTimestamp;

						dt = timestamp_1 - KFlastTimestamp; // calculate dt
						KFphi=phi_old;

						// predict with phi_old
						KFpredictEstimate();

						dt = timestamp_2 - timestamp_1; // calculate dt
						KFphi=phi_current;

						// predict with phi_new
						KFpredictEstimate();

						// udpate position
						KFupdateEstimate();

						dt = KFcurrentTimestamp - timestamp_2; // calculate dt

						// predict with new position and new phi
						KFpredictEstimate();

					} else if (targetGlobalPos_.localTimestamp < targetAttitude_.localTimestamp) { // check if GPOS was first
						double timestamp_1 = targetGlobalPos_.localTimestamp;
						double timestamp_2 = targetAttitude_.localTimestamp;

						dt = timestamp_1 - KFlastTimestamp; // calculate dt
						KFphi=phi_old;

						// predict
						KFpredictEstimate();

						// udpate position
						KFupdateEstimate();

						dt = timestamp_2 - timestamp_1; // calculate dt

						// predict with new position and old phi
						KFpredictEstimate();

						dt = KFcurrentTimestamp - timestamp_1; // calculate dt
						KFphi=phi_current;

						// predict with new position and new phi
						KFpredictEstimate();

					} else if (targetGlobalPos_.localTimestamp == targetAttitude_.localTimestamp) {
						double timestamp = targetAttitude_.localTimestamp;

						dt = timestamp - KFlastTimestamp; // calculate dt
						KFphi=phi_old;

						// predict
						KFpredictEstimate();

						// udpate position
						KFupdateEstimate();

						dt = KFcurrentTimestamp - timestamp; // calculate dt
						KFphi=phi_current;

						// predict
						KFpredictEstimate();

					}

					phi_old = phi_current;
					//v_air_old = v_air_current;
					KFlastTimestamp=KFcurrentTimestamp;
					dt=0;

}

void EstimatorKF::KF_pos(){	// case ONLY new position available

	// set new inputs
	GPS_pos_vel << targetPosLocal_.y << endr
			<< targetGlobalPos_.velocity.lat << endr
			<< targetPosLocal_.x << endr
			<< targetGlobalPos_.velocity.lon << endr
			<< targetPosLocal_.z << endr
			<< targetGlobalPos_.velocity.alt << endr;

	dt = targetGlobalPos_.localTimestamp - KFlastTimestamp; // calculate dt

	KFcurrentTimestamp=

	KFphi=phi_old;
					// predict
					KFpredictEstimate();

					// udpate position
					KFupdateEstimate();//execution of update function

	dt = KFcurrentTimestamp - targetGlobalPos_.localTimestamp; // calculate dt

					// predict
					KFpredictEstimate();

					KFlastTimestamp=KFcurrentTimestamp;
					dt=0;

}

void EstimatorKF::KF_attitude(){	// case ONLY new attitude available

	// set new inputs
	phi_current=targetAttitude_.roll;

	dt = targetAttitude_.localTimestamp - KFlastTimestamp; // calculate dt
	KFphi=phi_old;
				// predict with old phi
				KFpredictEstimate();

	dt = KFcurrentTimestamp - targetAttitude_.localTimestamp; // calculate dt
	KFphi=phi_current;

				// predict with new phi
				KFpredictEstimate();

				phi_old = phi_current;
				//v_air_old = v_air_current;
				KFlastTimestamp=KFcurrentTimestamp;
				dt=0;

}

void EstimatorKF::KF_NoNewInformation(){	// case NO new information available
	dt = KFcurrentTimestamp - KFlastTimestamp; // calculate dt
	KFphi=phi_old;

				KFpredictEstimate();

				KFlastTimestamp=KFcurrentTimestamp;
				dt=0;

}

} /* namespace tracking */

