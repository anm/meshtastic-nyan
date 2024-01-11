#pragma once

#include <cstdint>

constexpr uint16_t METERS_PER_NM = 1852;
constexpr double MPS_TO_KNOTS = (60 * 60) / METERS_PER_NM;

/*
  Want Ground wind speed and direction.
  Best derive from apparent wind, compass, and GNSS SOG/COG.

  AWA + true heading -> AWD

  Water referenced true wind messages are ambigious and probably wrong,
  because it can't really be done without a leeway sensor. We need GPS anyway
  for report location, so might as well work from apparent to start with.
*/

struct NyanVessel {
  double HDT;
  uint32_t HDT_ts;
  double AWS;
  uint32_t AWS_ts;
  double AWA;
  uint32_t AWA_ts;

  double COG;
  double SOG;

  double water_temp;
  uint32_t water_temp_ts;

  double meshtastic_COG;
  double meshtastic_SOG;
  uint32_t meshtastic_position_ts;

  double AWD();
};

/*
  Need running average and max over last reporting period - 10 mins seems to be a standard.

  First do a three second running average, then take the max and mean of
  that. This was the storage is bounded. Don't need more detail than three
  seconds.
*/

/* Weather station reporting period in seconds. */
constexpr uint16_t met_reporting_period = 600;
