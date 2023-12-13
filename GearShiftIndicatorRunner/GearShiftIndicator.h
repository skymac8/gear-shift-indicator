#ifndef GEARSHIFTINDICATOR_H
#define GEARSHIFTINDICATOR_H

#include <cmath>
#include <vector>

// Constant, Beetle-specific values.
// Note that since the Beetle is FWD, the relevant tire diameter
// for calculations is the front tire diameter. Since the tire
// diameters vary slighty between the front and right tire,
// use the average front tire diameter for calculations.
// NB: These values were measured while cold.
const float kFrontLeftTireDiameterIn = 23.75; // in
const float kFrontRightTireDiameterIn = 23.5; // in
// Average front tire diameter.
const float kTireDiameterIn =
    (kFrontLeftTireDiameterIn + kFrontRightTireDiameterIn) / 2.0; // in.
// For use in tire tolerance method.
const float kTireTolerance = 0.05; // %

// Gear ratios. Dimensionless.
const float k1st = 3.300;
const float k2nd = 1.944;
const float k3rd = 1.308;
const float k4th = 1.029;
const float k5th = 0.837;
const std::vector<float> forwardGears{k1st, k2nd, k3rd, k4th, k5th};
const float kFinalDriveRatio = 3.938;
// Highest gear that the car is equipped with.
const int kMaxGear = 5;

// RPM at which an upshift is recommended. Selected arbitrarily. Can be updated.
const float kUpshiftRPM = 1800.0; // RPM.

// Tolerance to apply to the computed gear ratio value.
// Selected to be a small enough percentage so as not to encounter overlap
// between the value ranges of neighboring gears. Can be updated if needed.
const float kTolerance = 0.05; // %.

// Mathematical constants.
const float kMinPerHr = 60.0;    // min/hr
const float kInPerMi = 63360.0;  // in/mi
const float kPi = std::acos(-1); // Dimensionless

// For computing RPM as a function of speed.
const float beetleConstants =
    (kTireDiameterIn * kMinPerHr * kPi) / (kFinalDriveRatio * kInPerMi);
// Same as above, but calculated using upper bound of tire diameter.
const float beetleConstants_UpperTireTolerance =
    (kTireDiameterIn * (1 + kTireTolerance) * kMinPerHr * kPi) /
    (kFinalDriveRatio * kInPerMi);
// Same as above, but calculated using lower bound of tire diameter.
const float beetleConstants_LowerTireTolerance =
    (kTireDiameterIn * (1 - kTireTolerance) * kMinPerHr * kPi) /
    (kFinalDriveRatio * kInPerMi);

// Serial constants.
const unsigned long bps = 115200; // Serial data rate. bits/s.
const uint16_t timeout = 2000;    // Timeout. ms.

// Returns true if the value is within spec, applying kTolerance to the spec.
bool isMatch(float value, float spec);

// Returns true if spec sits between the lower and upper values (inclusive).
bool isWithin(float lower, float upper, float spec);

// Returns computed current gear ratio, based on speed and RPM.
float gearRatio(float rpm, float speedMph);

// For use with tire tolerance method: given tolerance on tire diameter,
// returns upper bound of computed gear ratio.
float gearRatio_UpperTireTolerance(float rpm, float speedMph);

// For use with tire tolerance method: given tolerance on tire diameter,
// returns lower bound of computed gear ratio.
float gearRatio_LowerTireTolerance(float rpm, float speedMph);

// Returns expected RPM, given vehicle speed and gear ratio.
float expectedRpm(float speedMph, float gearRatio);

// Different gear calculation methods. See comment in .ino file that describes
// each mode.
uint8_t matchedGearBasedOnTireTolerance(float computedGear,
                                        float computedGearUpper);
uint8_t matchedGearBasedOnOverallTolerance(float computedGear);
uint8_t matchedGearBasedOnClosest(float computedGear);

// Returns recommended gear, based on fuel economy estimates, represented as an
// int. If not in a forward drive gear (e.g., if in Reverse or Neutral),
// recommends the current gear.
uint8_t recommendation(int matchedGear, float rpm);

#endif
