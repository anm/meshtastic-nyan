#pragma once

#include <iostream>
#include <cmath>
#include "constants.h"
#include "NyanVessel.h"

// For logging
#include "configuration.h"

using namespace std;

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
    if (!(v.AWS.valid() && v.AWA.valid() && v.HDT.valid() &&
          v.SOG.valid() && v.COG.valid())) {
      LOG_DEBUG("Can't derive_ground_wind: invalid data.\n");
      return false;
    }

    LOG_DEBUG("Wind using averages: AWS: %f AWA: %f HDT: %f\n", v.AWS.average(), v.AWA.average(), v.HDT.average());
    vec apparent_wind {v.AWS.average(), derive_AWD(v.AWA.average() * deg_to_rad, v.HDT.average()) * deg_to_rad};
    vec course {v.SOG.get(), v.COG.get() * deg_to_rad};
    vec gw = apparent_wind + course;

    GWS = gw.magnitude();
    GWD = gw.angle() * rad_to_deg;
    return true;
  }
};
