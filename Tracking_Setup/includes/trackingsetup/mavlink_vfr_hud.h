/*
 * mavlink_vfr_hud.h
 *
 *  Created on: May 17, 2015
 *      Author: Roman Meier
 */

#ifndef MAVLINK_AIRSPEEDS_H_
#define MAVLINK_AIRSPEEDS_H_

#include <trackingsetup/mavlink_reader.h>

namespace tracking {

class MavlinkVfrHud {
public:
	MavlinkVfrHud(MavlinkMessages* mavlinkMessages);
	bool getVfrHud(VfrHud* vfr_hud);

	virtual ~MavlinkVfrHud();

private:
	MavlinkMessages* mavlinkMessages_;

	uint64_t lastTimestamp_;

};

} /* namespace tracking */

#endif /* MAVLINK_AIRSPEEDS_H_ */
