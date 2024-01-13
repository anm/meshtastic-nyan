#include "wind.h"

double vec::magnitude() {
  return sqrt(x*x + y*y);
}

double vec::angle() {
  double a;
    if (x>0) {
      a = atan(y/x);
    } else if (x<0 && y>=0) {
      a = atan(y/x) + pi;
    } else if (x<0 && y<0) {
      a = atan(y/x) - pi;
    } else if (x==0 && y>0) {
      a = pi/2;
    } else if (x==0 && y<0) {
      a = -pi/2;
    } else if (x==0 && y==0) {
      a = 0;
    } else {
      // should not happen
      a = NAN;
    }

    if (a<0) {
      a+=tau;
    }

    return a * rad_to_deg;
}

vec::vec(const double magnitude, const double angle) {
  const double r = angle * deg_to_rad;
  x = magnitude * cos(r);
  y = magnitude * sin(r);
}

vec vec::operator+(const vec& v) {
  vec sum;
  sum.x = this->x + v.x;
  sum.y = this->y + v.y;
  return sum;
}

vec vec::operator-(const vec& v) {
  vec sum;
  sum.x = this->x - v.x;
  sum.y = this->y - v.y;
  return sum;
}

