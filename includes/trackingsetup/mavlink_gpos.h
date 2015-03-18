/*
 * mavlink_gpos.h
 *
 *  Created on: March 11, 2015
 *      Author: Thomas Mantel
 */

#ifndef MAVLINKGPOS_H_
#define MAVLINKGPOS_H_

#include <trackingsetup/mavlink_reader.h>
#include <trackingsetup/position_input.h>

namespace tracking {

class MavlinkGpos: public PositionInput {
public:
    MavlinkGpos(MavlinkMessages* mavlinkMessages);
	bool getPos(GPSPos* gpos);
	bool getVel(GPSPos* vel);
	bool getGpos(GlobalPos* gpos);

	virtual ~MavlinkGpos();

	friend std::ostream& operator<<(std::ostream& out,
			const MavlinkGpos& mavlinkGPOS);

private:
    MavlinkMessages* mavlinkMessages_;

	uint64_t lastPosTimestamp_;
	uint64_t lastVelTimestamp_;

};

} /* namespace tracking */

#endif /* MAVLINKGPOS_H_ */
