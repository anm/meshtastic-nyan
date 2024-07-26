#include <Arduino.h>

#include "NMEA0183.h"
#include "NMEA0183Msg.h"
#include "NMEA0183Messages.h"

// A function that is defined in the NMEA0183 library that I want to use directly.
double NMEA0183GetDouble(const char *data);

bool parse_sentence(const char *buf);

void NMEA_serial_setup(void);
void NMEA_serial_loop(void);
