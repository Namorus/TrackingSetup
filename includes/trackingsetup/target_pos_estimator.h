/*
 * forward_calc.h
 *
 *  Created on: Mar 31, 2015
 *      Author: thomas
 */

#ifndef FORWARD_CALC_H_
#define FORWARD_CALC_H_

#include <trackingsetup/trackingEstimator.h>

namespace tracking {

class ForwardCalc: public TrackingEstimator {
public:
	ForwardCalc();
	virtual ~ForwardCalc();

	void predictEstimate(); // Roman
	void updateEstimate();

};

} /* namespace tracking */

#endif /* FORWARD_CALC_H_ */
