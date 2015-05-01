/*
 * forward_calc.h
 *
 *  Created on: Mar 31, 2015
 *      Author: thomas
 */

#ifndef FORWARD_CALC_H_
#define FORWARD_CALC_H_

#include <trackingsetup/trackingEstimator.h>
#include <armadillo> // ROMAN	 matrix calculation library


namespace tracking {

class ForwardCalc: public TrackingEstimator {

private:			//are parameters of member function
	const float gravity;		// gravity constant
	const float var_phi;		// variance of roll angle phi
	const float r_a;			// measurement covariance parameter (position)
	const float r_b;			// measurement covariance parameter (velocity)
	const float q;				// initial auxiliary process covariance parameter

public:
	vec xhat; // state declaration
	mat P;	 // covariance matrix declaration

	ForwardCalc(){ 				//constructor: initialize parameters at first call of class
		gravity=9.81;
		var_phi=(0.4*M_PI/180)^2;
		r_a=(0.7)^2;
		r_b=(0.4)^2;
		q=0.02;
		xhat << targetPosLocal_.y << endr
			 << targetGlobalPos_.velocity.lat << endr
			 << targetPosLocal_.x  << endr
			 << targetGlobalPos_.velocity.lon << endr
			 << targetPosLocal_.z << endr
			 << targetGlobalPos_.velocity.alt << endr;

		P 	<< r_a << 0 << 0 << 0 << 0 << 0 << endr
			<< 0 << r_b << 0 << 0 << 0 << 0 << endr
			<< 0 << 0 << r_a << 0 << 0 << 0 << endr
		    << 0 << 0 << 0 << r_b << 0 << 0 << endr
			<< 0 << 0 << 0 << 0 << r_a << 0 << endr
			<< 0 << 0 << 0 << 0 << 0 << r_b << endr;
	}

	virtual ~ForwardCalc(); //(virtual?) destructor

	void predictEstimate(double phi, double v_air, double dt); // Roman, element function
	void updateEstimate(vec Z);

};

} /* namespace tracking */

#endif /* FORWARD_CALC_H_ */
