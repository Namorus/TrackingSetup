/*
 * TSmavlinkMag.h
 *
 *  Created on: Feb 4, 2015
 *      Author: asl
 */

#ifndef TSMAVLINKMAG_H_
#define TSMAVLINKMAG_H_

#include <trackingsetup/mavlink_reader.h>

namespace tracking {

class MavlinkMagnetometer: public Input {
public:
    MavlinkMagnetometer(MavlinkMessages* mavlinkMessages_);
	bool getMag(MagReading* mag);

	virtual ~MavlinkMagnetometer();

	friend std::ostream& operator<<(std::ostream& out,
				const MavlinkReader& mavlinkGPS);

private:
    MavlinkMessages* mavlinkMessages_;

	uint64_t lastTimestamp_;

};

} /* namespace tracking */

#endif /* TSMAVLINKMAG_H_ */
