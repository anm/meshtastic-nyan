#include "NyanVessel.h"

double NyanVessel::AWD() {
  double awd = AWA + HDT;
  if (awd >= 360) awd -= 360;
  return awd;
}
