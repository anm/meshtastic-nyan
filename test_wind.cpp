/*
  Run:
  g++ -lm wind.cpp && ./a.out
*/


#include "wind.h"

void test(double AWD, double AWS, double COG, double SOG) {
  vec GW = wind::ground_wind(AWD, AWS, COG, SOG);
  cout << "GW.x: " << GW.x << " GW.y: " << GW.y <<
    " mag: " << GW.magnitude() << " angle: " << GW.angle() << "\n";
}

int main() {
  cout << "Vector / wind test\n";
  test(0,0,0,0);
  test(200,20,0,10);
  test(180,30,0,10);
  test(180,30,90,10);

  cout << "precedence: " << (1<2 && 1==1);
}
