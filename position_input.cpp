/*
 * TSGpsInput.cpp
 *
 *  Created on: Feb 3, 2015
 *      Author: asl
 */

#include <trackingsetup/position_input.h>

namespace tracking {
void PositionInput::setLabel(std::string label) {
	label_ = label;
}

std::string PositionInput::getLabel() {
	return label_;
}
void PositionInput::getLabel(std::string& label) {
	label = label_;
}

std::ostream& operator<<(std::ostream& out, const PositionInput& posInput) {
	out << "$" << posInput.label_ << " ";
	out.precision(15);
	out << posInput.pos_.lat << " " << posInput.pos_.lon << " "
			<< posInput.pos_.elev << " ";
	return out;
}

} /* namespace tracking */
