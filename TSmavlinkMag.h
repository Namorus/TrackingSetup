/*
 * TSmavlinkMag.h
 *
 *  Created on: Feb 4, 2015
 *      Author: asl
 */

#ifndef TSMAVLINKMAG_H_
#define TSMAVLINKMAG_H_

#include "TSmavlinkReader.h"
#include "TAinput.h"

namespace tracking {

class TSmavlinkMag: public TAinput {
public:
    TSmavlinkMag(MavlinkMessages* mavlinkMessages_);
	bool getMag(MagReading* mag);

	virtual ~TSmavlinkMag();

	friend std::ostream& operator<<(std::ostream& out,
				const TSmavlinkReader& mavlinkGPS);

private:
    MavlinkMessages* mavlinkMessages_;

	uint64_t lastTimestamp_;

};

} /* namespace tracking */

#endif /* TSMAVLINKMAG_H_ */
