#include "GearShiftIndicator.h"
#include <algorithm>
#include <iostream>
#include <iterator>
#include <vector>

bool isMatch(float value, float spec) {
  return (value >= spec * (1 - kTolerance)) &&
         (value <= spec * (1 + kTolerance));
}

bool isWithin(float lower, float upper, float spec) {
  return (spec >= lower && spec <= upper);
}

float gearRatio(float rpm, float speedMph) {
  if (speedMph <= 0)
    return -1;
  return (rpm / speedMph) * beetleConstants;
}

float gearRatio_UpperTireTolerance(float rpm, float speedMph) {
  if (speedMph <= 0)
    return -1;
  return (rpm / speedMph) * beetleConstants_UpperTireTolerance;
}

float gearRatio_LowerTireTolerance(float rpm, float speedMph) {
  if (speedMph <= 0)
    return -1;
  return (rpm / speedMph) * beetleConstants_LowerTireTolerance;
}

uint8_t matchedGearBasedOnTireTolerance(float computedGear,
                                        float computedGearUpper) {
  for (int i = 0; i < forwardGears.size(); i++) {
    if (isWithin(computedGear, computedGearUpper, forwardGears[i])) {
      // 1st gear occurs at 0th index, so offset by 1.
      return i + 1;
    }
  }
  // If we don't match any of these ratios, assume that we must be in Reverse,
  // Neutral, or have the clutch in.
  return 0;
}

uint8_t matchedGearBasedOnOverallTolerance(float computedGear) {
  for (int i = 0; i < forwardGears.size(); i++) {
    if (isMatch(computedGear, forwardGears[i])) {
      // 1st gear occurs at 0th index, so offset by 1.
      return i + 1;
    }
  }
  // If we don't match any of these ratios, assume that we must be in Reverse,
  // Neutral, or have the clutch in.
  return 0;
}

uint8_t matchedGearBasedOnClosest(float computedGear) {
  // Start out with a vector containing k1st ... k5th.
  std::vector<float> distanceToForwardGears(forwardGears);
  for (int i = 0; i < distanceToForwardGears.size(); i++) {
    distanceToForwardGears[i] = abs(distanceToForwardGears[i] - computedGear);
  }
  std::vector<float>::iterator minDist = std::min_element(
      distanceToForwardGears.begin(), distanceToForwardGears.end());
  int nearestGearIndex = std::distance(distanceToForwardGears.begin(), minDist);
  // 1st gear occurs at 0th index, so offset by 1.
  uint8_t nearestGear = nearestGearIndex + 1;
  return nearestGear;
}

uint8_t recommendation(int matchedGear, float rpm) {
  // Not possible to upshift if we're already in the highest gear.
  if (matchedGear == kMaxGear)
    return matchedGear;
  if (rpm > kUpshiftRPM)
    return matchedGear + 1;
  return matchedGear;
}
