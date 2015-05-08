/*
 * attitude_input.cpp
 *
 *  Created on: May 8, 2015
 *      Author: Maverick
 */

#include <trackingsetup/attitude_input.h>

namespace tracking {
void AttitudeInput::setLabel(std::string label) {
	label_ = label;
}

std::string AttitudeInput::getLabel() {
	return label_;
}
void AttitudeInput::getLabel(std::string& label) {
	label = label_;
}

} /* namespace tracking */



