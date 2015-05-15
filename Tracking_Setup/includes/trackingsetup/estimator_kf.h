/*
 * estimator_kf.h
 *
 *  Created on: May 15, 2015
 *      Author: Roman Meier
 */


#ifndef ESTIMATOR_KF_H_
#define ESTIMATOR_KF_H_

#include <trackingsetup/trackingEstimator.h>
#include <armadillo> 	// matrix calculation library

using namespace arma;

namespace tracking {

class EstimatorKF: public TrackingEstimator {

private:
	//parameters of member function

	//constant values
	double gravity; 				// gravity constant
	double var_phi;					// variance of roll angle phi
	long double r_a;				// measurement covariance parameter (position)
	long double r_b;				// measurement covariance parameter (velocity)
	long double q;					// initial auxiliary process covariance parameter

	// variable values
	arma::vec xhat; 				// state declaration
	arma::vec GPS_pos_vel;			// GPS position & velocity measurement
	arma::mat P;	 				// covariance matrix declaration

	long double phi_current;		// roll angle phi
	long double phi_old;			// save old roll angle phi
	long double KFphi;				// input roll angle phi into member function

	//double old_v_air;				// save old air speed
	uint64_t KFcurrentTimestamp;	// current Kalman Filter time of execution
	uint64_t KFlastTimestamp;		// last Kalman Filter time of execution

	double dt;

public:

	bool newAttitude;				// check if new attitude available
	bool newPosition;				// check if new position available


	EstimatorKF();
	virtual ~EstimatorKF();

	void updateEstimate();
	void KF_PredictEstimate();		// KF predict
	void KF_UpdateEstimate();		// KF update

	void KF_Pos_Attitude();			// case new position & attitude available
	void KF_Pos();					// case ONLY new position available
	void KF_Attitude();				// case ONLY new attitude available
	void KF_NoNewInformation();		// case NO new information available

	// high level function to handle member functions and timestamps
	void getEstimate(GlobalPos remoteGlobalPosition, Attitude remoteAtt);

};

} /* namespace tracking */

#endif /* ESTIMATOR_KF_H_ */
