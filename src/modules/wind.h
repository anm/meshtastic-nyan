#pragma once

#include <iostream>
#include <cmath>
using namespace std;

constexpr double pi = 3.141592653589793;
constexpr double tau = pi * 2;
constexpr double deg_to_rad = tau / 360;
constexpr double rad_to_deg = 360 / tau;

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

class wind {
public:
  static vec ground_wind(double AWD, double AWS, double COG, double SOG);
};
