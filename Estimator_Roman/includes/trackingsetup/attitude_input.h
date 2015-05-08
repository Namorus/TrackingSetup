/*
 * attitude_input.h
 *
 *  Created on: May 8, 2015
 *      Author: Maverick
 */

#ifndef ATTITUDE_INPUT_H_
#define ATTITUDE_INPUT_H_

#include <trackingsetup/input.h>
#include <trackingsetup/mavlink_reader.h>
#include <trackingsetup/types.h>

namespace tracking {

class AttitudeInput: public Input {
public:
	void getAtt(Att* att);

protected:
	std::string label_;
	Att att_;

	void setLabel(std::string);
	void getLabel(std::string&);
	std::string getLabel();

};

} /* namespace tracking */

#endif /* ATTITUDE_INPUT_H_ */
