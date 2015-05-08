/*
 * mavlink_attitude.h
 *
 *  Created on: May 8, 2015
 *      Author: Maverick
 */

#ifndef MAVLINK_ATTITUDE_H_
#define MAVLINK_ATTITUDE_H_

#include <trackingsetup/mavlink_reader.h>
#include <trackingsetup/attitude_input.h>

namespace tracking {

class MavlinkAttitude: public AttitudeInput {
public:
    MavlinkAttitude(MavlinkMessages* mavlinkMessages);
	bool getAtt(Att* att);
	bool getAttitude(Attitude* attitude);

	virtual ~MavlinkAttitude();

	friend std::ostream& operator<<(std::ostream& out,
			const MavlinkAttitude& mavlinkATT);

private:
    MavlinkMessages* mavlinkMessages_;

	uint64_t lastAttTimestamp_;

};

} /* namespace tracking */

#endif /* MAVLINK_ATTITUDE_H_ */
