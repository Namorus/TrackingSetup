/*
 * mavlink_att.h
 *
 *  Created on: May 8, 2015
 *      Author: Maverick
 */

#ifndef MAVLINK_ATT_H_
#define MAVLINK_ATT_H_

#include <trackingsetup/mavlink_reader.h>
#include <trackingsetup/attitude_input.h>

namespace tracking {

class MavlinkAtt: public AttitudeInput {
public:
    MavlinkAtt(MavlinkMessages* mavlinkMessages, std::string label);
	bool getAtt(Att* att);

	virtual ~MavlinkAtt();

	friend std::ostream& operator<<(std::ostream& out,
			const MavlinkAtt& mavlinkATT);

private:
    MavlinkMessages* mavlinkMessages_;

	uint64_t lastTimestamp_;

};

} /* namespace tracking */

#endif /* MAVLINK_ATT_H_ */
