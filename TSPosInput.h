/*
 * TSGpsInput.h
 *
 *  Created on: Feb 3, 2015
 *      Author: asl
 */

#ifndef TSPOSINPUT_H_
#define TSPOSINPUT_H_

#include "TAinput.h"
#include "TAtypes.h"
#include "TSmavlinkReader.h"

namespace tracking {

class TSPosInput: public TAinput {
public:
	void getPos(GPSPos* pos);

	friend std::ostream& operator<<(std::ostream& out,
			const TSPosInput& posInput);

protected:
	std::string label_;
	GPSPos pos_;

	void setLabel(std::string);
	void getLabel(std::string&);
	std::string getLabel();

};

} /* namespace tracking */

#endif /* TSPOSINPUT_H_ */
