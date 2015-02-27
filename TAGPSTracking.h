/*
 * TAGPSTracking.h
 *
 *  Created on: Jun 18, 2013
 *      Author: thomas
 */

#ifndef TAGPSTRACKING_H_
#define TAGPSTRACKING_H_

#include "TATrackingMode.h"

namespace tracking {

class TAGPSTracking: public tracking::TATrackingMode {
public:
	TAGPSTracking();

	void setNorthOffset(float panAngleNorth);

	void setMagneticDeclination(float magneticDeclination_);

	void setMapping(float panOffset, float tiltOffset);

//	void setAntennaPos(GPSPos antennaPOS);

	void update(GPSPos& antennaPos, GPSPos& targetPos);

private:
	float panOffset_, tiltOffset_;
	float magneticDeclination_;

	// GPSPos antennaPOS;
};

} /* namespace tracking */
#endif /* TAGPSTRACKING_H_ */
