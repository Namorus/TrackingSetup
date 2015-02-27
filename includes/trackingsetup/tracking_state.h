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

class State: public tracking::TrackingSetup {
public:

	State();

	trackingState get();
	void set(trackingState newmode);
	std::string getModeName(trackingState mode);

	friend std::ostream& operator<<(std::ostream& out, const State& curMode);

protected:

	trackingState curmode;
	pthread_mutex_t* pMutexCurmode;
};

} /* namespace tracking */
#endif /* TAMODE_H_ */
