/*
 * TAGPSTracking.cpp
 *
 *  Created on: Jun 18, 2013
 *      Author: thomas
 */

//#include "TAGPSTracking.h"
#include "TSincludes.h"

namespace tracking {

TAGPSTracking::TAGPSTracking() :
		panOffset_(0), tiltOffset_(0), magneticDeclination_(0) {

}

void TAGPSTracking::setMapping(float panOffset, float tiltOffset) {
	panOffset_ = panOffset;
	tiltOffset_ = tiltOffset;
}

void TAGPSTracking::setMagneticDeclination(float magneticDeclination) {
	magneticDeclination_ = magneticDeclination;
}

void TAGPSTracking::update(GPSPos& antennaPos, GPSPos& targetPos) {
	double bearing = TAMappingEstimation::getBearing(&antennaPos, &targetPos);
	double azimuth = TAMappingEstimation::getAzimuth(&antennaPos, &targetPos);

	/*
	 * tiltOffset was wrongly calculated, and is omitted for the time being.
	 *
	 * setNewSetpoints(ct_abspos,bearing-panOffset,azimuth-tiltOffset);
	 */
	setNewSetpoints(ct_abspos, bearing - panOffset_ - magneticDeclination_, azimuth);
}

} /* namespace tracking */
