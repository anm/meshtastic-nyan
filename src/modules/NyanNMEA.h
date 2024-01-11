#include <Arduino.h>

#include "NMEA0183.h"
#include "NMEA0183Msg.h"
#include "NMEA0183Messages.h"

bool parse_sentence(const char *buf);
