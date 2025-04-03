#pragma once

#include <iostream>
#include <cmath>
#include "constants.h"
#include "NyanVessel.h"

// For logging
#include "configuration.h"

using namespace std;

/*
  Want Ground wind speed and direction.
  Best derive from apparent wind, compass, and GNSS SOG/COG.

  AWA + true heading -> AWD

  Water referenced true wind messages are ambigious and probably wrong,
  because it can't really be done without a leeway sensor. We need GPS anyway
  for report location, so might as well work from apparent to start with.
*/

/* Vectors. Angles in radians. */
class vec {
 public:

  double x,y;

  vec() {};
  vec(double magnitude, double angle);
  vec operator+(const vec& v);
  vec operator-(const vec& v);

  double magnitude();
  double angle();
};

/* Angles in degrees. */
class Wind {
public:
  static double derive_AWD(double AWA, double HDT) {
    double awd = AWA + HDT;
    if (awd >= 360) awd -= 360;
    return awd;
  }


  static bool derive_ground_wind(NyanVessel& v, double& GWS, double& GWD) {
    Position pos;
    bool havePosition = v.getPosition(&pos);

    if (!(v.AWS.valid() && v.AWA.valid() && v.HDT.valid() &&
          havePosition)) {
      LOG_DEBUG("Can't derive_ground_wind: invalid data.");
      return false;
    }

    LOG_DEBUG("Wind using averages: AWS: %f AWA: %f HDT: %f",
              v.AWS.get(), v.AWA.get(), v.HDT.get());
    vec apparent_wind {v.AWS.get(),
                       derive_AWD(v.AWA.get() * deg_to_rad, v.HDT.get()) * deg_to_rad};
    vec course {pos.SOG, pos.COG * deg_to_rad};
    vec gw = apparent_wind + course;

    GWS = gw.magnitude();
    GWD = gw.angle() * rad_to_deg;
    return true;
  }
};
