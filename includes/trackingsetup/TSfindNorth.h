/*
 * TSfindNorth.h
 *
 *  Created on: Feb 3, 2015
 *      Author: asl
 */

#ifndef TSFINDNORTH_H_
#define TSFINDNORTH_H_

#include "TATrackingMode.h"

namespace tracking {

class TSfindNorth: public TATrackingMode {
public:
	TSfindNorth();
	virtual ~TSfindNorth();

	void init (float panSpeed, float tiltAngle);

	void update(int curPanAngle, int curTiltAngle, MagReading* magn, bool panPositionReached, bool tiltPositionReached);

	findNorthState getLocateState();

	void reset();

	float northPanAngleFound();

	float magneticDeclination(GPSPos& localPos);

	friend std::ostream& operator<<(std::ostream& out, const TSfindNorth& findNorth);

private:
	float evalFunction(float, float);

	findNorthState curState_;
	float panSpeed_;
	int tiltAngle_;

    float panAngleInt_;

	float northFound_;


	std::list<float> panAngles_;
	std::list<float> fctValues_;
	std::list<float> magX_;
	std::list<float> magY_;


};

} /* namespace tracking */

#endif /* TSFINDNORTH_H_ */
