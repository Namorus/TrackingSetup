/*
 * TSmavlinkMag.h
 *
 *  Created on: Feb 4, 2015
 *      Author: asl
 */

#ifndef TSMAVLINKRADIO_H_
#define TSMAVLINKRADIO_H_

#include "TSmavlinkReader.h"
#include "TAinput.h"
#include "TAtypes.h"

namespace tracking {

class TSmavlinkRadioStatus: public TAinput {
public:
    TSmavlinkRadioStatus(MavlinkMessages* mavlinkMessages);
    bool getRSSI(RadioRSSI* radioRSSI);

    virtual ~TSmavlinkRadioStatus();

//	friend std::ostream& operator<<(std::ostream& out,	const TSmavlinkReader& mavlinkGPS);

private:
    MavlinkMessages* mavlinkMessages_;
    uint64_t lastTimestamp_;
};

} /* namespace tracking */

#endif /* TSMAVLINKRADIO_H_ */
