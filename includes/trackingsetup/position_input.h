/*
 * TSGpsInput.h
 *
 *  Created on: Feb 3, 2015
 *      Author: asl
 */

#ifndef TSPOSINPUT_H_
#define TSPOSINPUT_H_

#include <trackingsetup/input.h>
#include <trackingsetup/mavlink_reader.h>
#include <trackingsetup/types.h>

namespace tracking {

class PositionInput: public Input {
public:
	void getPos(GPSPos* pos);

	friend std::ostream& operator<<(std::ostream& out,
			const PositionInput& posInput);

protected:
	std::string label_;
	GPSPos pos_;

	void setLabel(std::string);
	void getLabel(std::string&);
	std::string getLabel();

};

} /* namespace tracking */

#endif /* TSPOSINPUT_H_ */
