/*
 * estimator_kf.h
 *
 *  Created on: May 10, 2015
 *      Author: Roman
 */

#ifndef ESTIMATOR_KF_H_
#define ESTIMATOR_KF_H_

#include <trackingsetup/trackingEstimator.h>
#include <armadillo> // ROMAN	 matrix calculation library

using namespace arma;

namespace tracking {

class EstimatorKF: public TrackingEstimator {

private:
	//parameters of member function

	//constant values
	const double gravity; 		// gravity constant
	const double var_phi;		// variance of roll angle phi
	const double r_a;			// measurement covariance parameter (position)
	const double r_b;			// measurement covariance parameter (velocity)
	const double q;				// initial auxiliary process covariance parameter

	// variable values
	arma::vec xhat; 			// state declaration
	arma::vec GPS_pos_vel;		// GPS position & velocity measurement
	arma::mat P;	 			// covariance matrix declaration

	double phi_current;			// roll angle phi
	double phi_old;				// save old roll angle phi
	double KFphi;				// input roll angle phi into member function

	//double old_v_air;			// save old air speed
	uint64_t KFcurrentTimestamp;// current Kalman Filter time of execution
	uint64_t KFlastTimestamp;	// last Kalman Filter time of execution

	// 3 time differences (new position & new attitude), 2 time differences (only new position or new attitude), 1 time difference (no new information)
	double dt;
	//double dt2;
	//double dt1;
public:

	EstimatorKF() { 								//constructor: initialize parameters at first call of class

		//constant values
		gravity = 9.81;
		var_phi = pow(0.4*M_PI/ 180.0,2);
		r_a = pow(0.8,2);
		r_b = pow(0.5,2);
		q = 0.025;

		// variable values
		xhat 	<< targetPosLocal_.y << endr
				<< targetGlobalPos_.velocity.lat << endr
				<< targetPosLocal_.x << endr
				<< targetGlobalPos_.velocity.lon << endr
				<< targetPosLocal_.z << endr
				<< targetGlobalPos_.velocity.alt << endr;

		GPS_pos_vel << targetPosLocal_.y << endr
				<< targetGlobalPos_.velocity.lat << endr
				<< targetPosLocal_.x << endr
				<< targetGlobalPos_.velocity.lon << endr
				<< targetPosLocal_.z << endr
				<< targetGlobalPos_.velocity.alt << endr;

		P 	<< r_a << 0 << 0 << 0 << 0 << 0 << endr
			<< 0 << r_b << 0 << 0 << 0 << 0 << endr
			<< 0 << 0 << r_a << 0 << 0 << 0 << endr
			<< 0 << 0 << 0 << r_b << 0 << 0 << endr
			<< 0 << 0 << 0 << 0 << r_a << 0 << endr
			<< 0 << 0 << 0 << 0 << 0 << r_b << endr;

		KFphi=0;
		phi_current = targetAttitude_.roll;
		phi_old = targetAttitude_.roll;
		//old_v_air=;
		KFcurrentTimestamp = 0;	// current Kalman Filter time of execution
		KFlastTimestamp = 0;	// last Kalman Filter time of execution
		dt=0;

	}
	virtual ~EstimatorKF(); 	//(virtual?) destructor


	void KFpredictEstimate();	// KF predict
	void KFupdateEstimate();	// KF update

	void KF_pos_attitude();		// case new position & attitude available
	void KF_pos();				// case ONLY new position available
	void KF_attitude();			// case ONLY new attitude available
	void KF_NoNewInformation();	// case NO new information available
};

} /* namespace tracking */

#endif /* ESTIMATOR_KF_H_ */
