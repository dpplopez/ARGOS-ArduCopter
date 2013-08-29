// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// == MODIFIED FOR ARGOS ==========================================================================
// AP_RangeFinder_SRF08.cpp - Arduino Library for SRF08 I2C sonar
// Author:      David Perez-Pinar
// Rev.:        1.0.1
// AC Rev.:     AC 3.0.1
// Date:        2013.08.15
// Status:      Beta testing
// Change Log:  1.0.0 - First functional code, tested in flight. Long range measurement problems.
//              1.0.1 - Added test code to measure read time and to retreive several echoes.
//
// License:     This library is free software; you can redistribute it and/or
//              modify it under the terms of the GNU Lesser General Public
//              License as published by the Free Software Foundation; either
//              version 2.1 of the License, or (at your option) any later version.
//
// Datasheet:   http://www.cs.york.ac.uk/micromouse/Docs/SRF08UltraSonicRanger.pdf
//
// Description: This class implements the SRF08 I2C sonar interface. In order to make it compatible
//              with the current ArduCopter sonar implementation, only the read() function is
//              maintained as public, and implements an internal state machine to take care of
//              commanding updates to the sonar hardware and returning the last good reading 
//              obtained from it, all of this within timing limits imposed by the sonar hardware
//              itself. Sensor measurements are reported after scaling and filtering. Scale factor
//              is hardcoded to be 1.0, as the sensor already reports distances in cm. The state
//              machine always starts a new measurement as soon as processing has timed out in
//              order to have new measurements as soon as possible.
//              Although firmware samples the sonar at 20 Hz, the sensor is read at a rate of about
//              10 Hz, because it needs a processing time of more than 65 ms to get the range.
//              Reported distance is always filtered, even if new measurements are not available.
//              Therefore, reported distance changes at a rate of 20 Hz, alternating filtered
//              new samples and filtered old samples.
//
// Hardware:    Sensor must be connected to the APM I2C port. SRF08 must be powered with 5 volts,
//              and will need 250 mA for generating ultrasonic signals used for ranging. Thus,
//              depending on devices connected to APM, it could happen that the sensor logic works
//              when powered through USB, but ranging doesn't. Also, I2C lines require a level
//              converter to take care of the 5v/3.3v conversion.
//
// Variables:   bool healthy: Indicates whether last communication with sensor was successful
//
// Methods:     read(): Read last distance measured (in cm)
//
// ================================================================================================

// AVR LibC Includes
#include "AP_RangeFinder_SRF08.h"
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

// Constructor //////////////////////////////////////////////////////////////

AP_RangeFinder_SRF08::AP_RangeFinder_SRF08( FilterInt16 *filter ) :
    RangeFinder(NULL, filter),
    healthy(true),
    _addr(AP_RANGE_FINDER_SRF08_DEFAULT_ADDR),
	_state(AP_RANGE_FINDER_SRF08_STSTDBY),
	_lastDistance(0),
	_lastTime(0)
{
    min_distance = AP_RANGE_FINDER_SRF08_MIN_DISTANCE;
    max_distance = AP_RANGE_FINDER_SRF08_MAX_DISTANCE;
}

// Public Methods //////////////////////////////////////////////////////////////

// read - return last value measured by sensor
int AP_RangeFinder_SRF08::read() {
	// Check current sensor state
	switch (_state) {
	case AP_RANGE_FINDER_SRF08_STSTDBY:
		// Sensor is probed for the first time: issue a ranging command
		take_reading();
		_lastTime = hal.scheduler->micros();
		_state = AP_RANGE_FINDER_SRF08_STMEASURING;
		break;
	case AP_RANGE_FINDER_SRF08_STMEASURING:
		uint32_t timer = hal.scheduler->micros();
		if (timer-_lastTime>=AP_RANGE_FINDER_SRF08_MEASUREDELAY) {
			// Measuring has finished: get measured range from sensor and update reported distance
#ifdef SRF08_GETECHOES
			get_measurement(SRF08_GETECHOES);
#else
			get_measurement();
#endif
			// Start a new measurement
			take_reading();
#ifdef SRF08_GETTIME
			readTime = hal.scheduler->micros() - timer;
#endif
			_lastTime = timer;
		}
		// Even if no new measurement is available, keep distance into specified limits and filter result
		_lastDistance = constrain_float(_lastDistance, min_distance, max_distance);
		_lastDistance = _mode_filter->apply(_lastDistance);
		break;
	}
    return _lastDistance;
}

// Protected Methods ///////////////////////////////////////////////////////////

// take_reading - ask sensor to make a range reading
bool AP_RangeFinder_SRF08::take_reading()
{
    // Send I2C range measurement command to SRF08
    if (hal.i2c->writeRegister(_addr, AP_RANGE_FINDER_SRF08_REGCOMMAND, AP_RANGE_FINDER_SRF08_CMDMEASURE) != 0)
        healthy = false;
	else
        healthy = true;
	return healthy;
}

// get_measurement - get the measured distance from sensor
#ifdef SRF08_GETECHOES

bool AP_RangeFinder_SRF08::get_measurement(uint8_t echoes) {
	uint8_t i;
    // Read sensor distance registers and update last measured distance
	// If an I2C error is found, last distance is not updated
	for (i=0; i<SRF08_GETECHOES; i++) {
		if (hal.i2c->readRegisters(_addr, AP_RANGE_FINDER_SRF08_REGDISTANCE+i*2, 2, &(buff[2*i])) != 0) {
			healthy = false;
			break;
		}
		else {
			// Combine results into distance (only for first echo)
			if (i==0)
				_lastDistance = (((uint16_t) buff[2*i]) << 8) | ((uint16_t) buff[2*i+1]);
			healthy = true;
		}
	}
	return healthy;
}

#else

bool AP_RangeFinder_SRF08::get_measurement() {
	uint8_t buff[2];
    // Read sensor distance registers and update last measured distance
	// If an I2C error is found, last distance is not updated
    if (hal.i2c->readRegisters(_addr, AP_RANGE_FINDER_SRF08_REGDISTANCE, 2, buff) != 0)
        healthy = false;
	else {
		// Combine results into distance
		_lastDistance = (((uint16_t) buff[0]) << 8) | ((uint16_t) buff[1]);
		healthy = true;
    }
	return healthy;
}

#endif
