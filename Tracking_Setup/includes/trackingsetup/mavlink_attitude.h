/*
 * mavlink_attitude.h
 *
 *  Created on: May 8, 2015
 *      Author: Maverick
 */

#ifndef MAVLINK_ATTITUDE_H_
#define MAVLINK_ATTITUDE_H_

#include <trackingsetup/mavlink_reader.h>

namespace tracking {

class MavlinkAttitude {
public:
    MavlinkAttitude(MavlinkMessages* mavlinkMessages);
	bool getAttitude(Attitude* att);

	virtual ~MavlinkAttitude();

private:
    MavlinkMessages* mavlinkMessages_;

	uint64_t lastTimestamp_;

};

} /* namespace tracking */

#endif /* MAVLINK_ATTITUDE_H_ */
