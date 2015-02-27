/*
 * TAmode.h
 *
 *  Created on: Jun 17, 2013
 *      Author: thomas
 */

#ifndef TAMODE_H_
#define TAMODE_H_

#include <trackingsetup/trackingsetup.h>

namespace tracking {

class TAmode: public tracking::TrackingSetup {
public:

	TAmode();

	trackingMode get();
	void set(trackingMode newmode);
	std::string getModeName(trackingMode mode);

	friend std::ostream& operator<<(std::ostream& out, const TAmode& curMode);

protected:

	trackingMode curmode;
	pthread_mutex_t* pMutexCurmode;
};

} /* namespace tracking */
#endif /* TAMODE_H_ */
