/*
 * TSGpsInput.cpp
 *
 *  Created on: Feb 3, 2015
 *      Author: asl
 */

#include "TSPosInput.h"

namespace tracking {
void TSPosInput::setLabel(std::string label) {
	label_ = label;
}

std::string TSPosInput::getLabel() {
	return label_;
}
void TSPosInput::getLabel(std::string& label) {
	label = label_;
}

std::ostream& operator<<(std::ostream& out, const TSPosInput& posInput) {
	out << "$" << posInput.label_ << " ";
	out.precision(15);
	out << posInput.pos_.lat << " " << posInput.pos_.lon << " "
			<< posInput.pos_.elev << " ";
	return out;
}

} /* namespace tracking */
