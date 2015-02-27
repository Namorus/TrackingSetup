/*
 * TSfindNorth.cpp
 *
 *  Created on: Feb 3, 2015
 *      Author: asl
 */
#include <cmath>
#include <sstream>
#include "TSfindNorth.h"
#include <GeographicLib/MagneticModel.hpp>

namespace tracking {

TSfindNorth::TSfindNorth() :
	curState_(fn_NOTREADY),
	panSpeed_(0),
    tiltAngle_(0),
    panAngleInt_(0.0),
    northFound_(0) {

}

TSfindNorth::~TSfindNorth() {
	// TODO Auto-generated destructor stub
}

void TSfindNorth::init(float panSpeed, float tiltAngle) {
	panSpeed_ = panSpeed;
	tiltAngle_ = tiltAngle;

	northFound_ = 0;

    panAngleInt_ = 0.0;

	panAngles_.clear();
	fctValues_.clear();
	magX_.clear();
	magY_.clear();

	setNewTiltSetpoint(ct_abspos,tiltAngle_);
	setNewPanSetpoint(ct_velocity,panSpeed_);

	curState_ = fn_INITIALIZING;
}

void TSfindNorth::update(int curPanAngle, int curTiltAngle,
		MagReading* magn,
		bool panPositionReached, bool tiltPositionReached) {

    float maxMagY = magY_.front();
    std::list<float>::iterator magYit = magY_.begin();

    std::stringstream debugMessage("findNorth: ");

	switch (curState_) {
	case fn_NOTREADY:
		// this case should not be reached
		addLogMessage(vl_WARNING,"findNorth: update method called without initializing beforehand");
		curState_ = fn_FINISHED;
		break;

	case fn_INITIALIZING:
		if (tiltPositionReached && panPositionReached) {
			addLogMessage(vl_DEBUG,"findNorth: Recording magnetometer readings...");
			curState_ = fn_RECORDING;
            panAngles_.push_back(curPanAngle);
            magX_.push_back(magn->magX);
            magY_.push_back(magn->magY);
        }
		break;

	case fn_RECORDING:
        panAngleInt_ += fmod(curPanAngle-panAngles_.back()+360,360);
//        debugMessage << "integrated pan angle: " << panAngleInt_;
//        addLogMessage(vl_DEBUG,debugMessage.str());
		panAngles_.push_back(curPanAngle);
		magX_.push_back(magn->magX);
		magY_.push_back(magn->magY);

        if(panAngleInt_ > 360.0) {
            setNewPanSetpoint(ct_velocity,0.0);
			curState_ = fn_EVALUATING;
		}
		break;

	case fn_EVALUATING:
		addLogMessage(vl_INFO,"findNorth: Evaluating...");

		for(std::list<float>::iterator panIt = panAngles_.begin(); panIt != panAngles_.end(); panIt++) {
            if(*magYit > maxMagY) {
                maxMagY = *magYit;
				northFound_ = *panIt;
			}
            magYit++;
		}
        debugMessage << "North found @ " << northFound_ << "deg";
        addLogMessage(vl_INFO,debugMessage.str());
		curState_ = fn_FINISHED;
		addLogMessage(vl_INFO,"findNorth: finished");
		break;

	case fn_FINISHED:
		break;

	}

}

findNorthState TSfindNorth::getLocateState() {
	return curState_;
}

void TSfindNorth::reset() {
	panAngles_.clear();
	fctValues_.clear();
	magX_.clear();
	magY_.clear();
}

float TSfindNorth::northPanAngleFound() {
	return northFound_;
}

float TSfindNorth::magneticDeclination(GPSPos& localPos) {
	GeographicLib::MagneticModel mag("wmm2015","/usr/share/geographiclib/magnetic");
	double Bx, By, Bz;
	mag(2015, localPos.lat, localPos.lon, localPos.elev, Bx, By, Bz); // TODO: use function for value of year
	double H, F, D, I;
	GeographicLib::MagneticModel::FieldComponents(Bx, By, Bz, H, F, D, I);

	return D;
}

} /* namespace tracking */
