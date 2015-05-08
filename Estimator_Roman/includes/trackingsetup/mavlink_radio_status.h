/*
 * TSmavlinkMag.h
 *
 *  Created on: Feb 4, 2015
 *      Author: asl
 */

#ifndef TSMAVLINKRADIO_H_
#define TSMAVLINKRADIO_H_

#include <trackingsetup/input.h>
#include <trackingsetup/mavlink_reader.h>
#include <trackingsetup/types.h>

namespace tracking {

class MavlinkRadioStatus: public Input {
public:
    MavlinkRadioStatus(MavlinkMessages* mavlinkMessages);
    bool getRSSI(RadioRSSI* radioRSSI);

    virtual ~MavlinkRadioStatus();

//	friend std::ostream& operator<<(std::ostream& out,	const TSmavlinkReader& mavlinkGPS);

private:
    MavlinkMessages* mavlinkMessages_;
    uint64_t lastTimestamp_;
};

} /* namespace tracking */

#endif /* TSMAVLINKRADIO_H_ */
