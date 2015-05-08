/*
 * estimator_kf.h
 *
 *  Created on: Mar 31, 2015
 *      Author: thomas
 */

#ifndef FORWARD_CALC_H_
#define FORWARD_CALC_H_

#include <trackingsetup/trackingEstimator.h>

namespace tracking {

class EstimatorKF: public TrackingEstimator {
public:
	EstimatorKF();
	virtual ~EstimatorKF();

	void updateEstimate();
};

} /* namespace tracking */

#endif /* FORWARD_CALC_H_ */
