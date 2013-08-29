// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// == MODIFIED FOR ARGOS ==========================================================================
// AP_RangeFinder_SRF08.cpp - Arduino Library for SRF08 I2C sonar
// Author:     David Perez-Pinar
// Rev.:       1.0.1
// AC Rev.:    AC 3.0.1
// Date:       2013.08.15
// Status:     Beta testing
// ================================================================================================

#ifndef __AP_RANGEFINDER_SRF08_H__
#define __AP_RANGEFINDER_SRF08_H__

#include "RangeFinder.h"

//#define SRF08_GETTIME           // Define (uncomment) to include code for measuring duration of read function
//#define SRF08_GETECHOES 4       // Define (uncomment) to include code for retrieving several echoes (as defined by number; max. 16)

#define AP_RANGE_FINDER_SRF08_DEFAULT_ADDR   (0xE0 >> 1)	// Default sensor address (I2C driver requires right shift for 7 bit address)

#define AP_RANGEFINDER_SRF08                 0			// Sonar hardware identifier for Mission Planner
#define AP_RANGE_FINDER_SRF08_SCALER         1.0		// Sensor measures distances in cm, no need to scale
#define AP_RANGE_FINDER_SRF08_MIN_DISTANCE   20			// SRF08 minimum specified distance is 3 cm
#define AP_RANGE_FINDER_SRF08_MAX_DISTANCE   600		// SRF08 maximum specified distance is 6 m
#define AP_RANGE_FINDER_SRF08_MEASUREDELAY   70000		// SRF08 required delay from ranging command for reading results

#define AP_RANGE_FINDER_SRF08_CMDMEASURE     0x51		// Command for asking the sensor to take a range reading in cm
#define AP_RANGE_FINDER_SRF08_REGCOMMAND     0x00		// I2C registry address for commands
#define AP_RANGE_FINDER_SRF08_REGDISTANCE    0x02		// I2C registry address for distance results (two bytes, high first)
#define AP_RANGE_FINDER_SRF08_REGLIGHT       0X01		// I2C registry address for light measurement results

#define AP_RANGE_FINDER_SRF08_STSTDBY        0
#define AP_RANGE_FINDER_SRF08_STMEASURING    1

class AP_RangeFinder_SRF08 : public RangeFinder {
public:
    // Member variables
    bool healthy;			// Health indicator
    // Construction
    AP_RangeFinder_SRF08(FilterInt16 *filter);
	// Public methods
    // init - simply sets the i2c address
    void init(uint8_t address = AP_RANGE_FINDER_SRF08_DEFAULT_ADDR) { _addr = address; }
	// getAddress - get I2C address
	uint8_t getAddress() { return _addr; }
	// calculate scaler (not needed for SRF08: kept here for compatibility)
	float calculate_scaler(int sonar_type, float adc_refence_voltage) { return 1.0f; }
    // read value from sensor and return distance in cm
    int read();
	// TESTING WITH SEVERAL ECHOES
#ifdef SRF08_GETECHOES
	uint8_t buff[SRF08_GETECHOES*2];
#endif
	// TESTING READ TIME
#ifdef SRF08_GETTIME
	uint32_t readTime;
#endif
protected:
	// Member variables
    uint8_t  _addr; 			// Sonar I2C hardware address
	uint8_t  _state;			// Ranging internal state to control sensor timing
	uint16_t _lastDistance;		// Last measured distance (after scaling and filtering)
	uint32_t _lastTime;         // Last ranging command time
	// Protected methods
    // take_reading - ask sensor to make a range reading
    bool take_reading();
	// get_measurement - get the measured distance from sensor
#ifdef SRF08_GETECHOES
	bool get_measurement(uint8_t echoes);
#else
	bool get_measurement();
#endif
};

#endif  // __AP_RANGEFINDER_SRF08_H__
