/* Based on example code */

#include "NyanNMEA.h"
#include "NyanVessel.h"

#include "gps/RTC.h"

extern NyanVessel v;

// From meshtastic, used for debugging output
#include "configuration.h"

void handle_MTW(const tNMEA0183Msg &msg) {
  if ((msg.FieldCount() == 2) && (msg.Field(1)[0] == 'C')) {
    v.water_temp  = NMEA0183GetDouble(msg.Field(0));
    LOG_INFO("Parsed MTW: %.2fC", v.water_temp);
  }
}

void handle_HDT(const tNMEA0183Msg &msg) {
  if ((msg.FieldCount() == 2) && (msg.Field(1)[0] == 'T')) {
    double heading = NMEA0183GetDouble(msg.Field(0));
    v.HDT = heading;
    v.HDT_ts = getValidTime(RTCQuality::RTCQualityDevice);
    LOG_INFO("Parsed HDT: %f", v.HDT);
  }
}

void handle_MWV(const tNMEA0183Msg &msg) {
  // Reference is: NMEA0183Wind_Apparent or _True
  // Speed in m/s
  // Angle in radians?
  double wind_angle;
  tNMEA0183WindReference wind_reference;
  double wind_speed;

  if (NMEA0183ParseMWV_nc(msg, wind_angle, wind_reference, wind_speed)) {
    if (wind_reference != NMEA0183Wind_Apparent) {
      return;
    }
    v.AWA = wind_angle;
    v.AWS = wind_speed;
    v.AWA_ts = v.AWS_ts = getValidTime(RTCQuality::RTCQualityDevice);
    LOG_INFO("Parsed MWV");
  }
}

 struct tNMEA0183Handler {
  const char *Code;
  void (*Handler)(const tNMEA0183Msg &NMEA0183Msg);
};

tNMEA0183Handler NMEA0183Handlers[]={
  {"HDT",&handle_HDT},
  {"MTW",&handle_MTW},
  {"MWV",&handle_MWV},
  {0,0}
};

void HandleNMEA0183Msg(const tNMEA0183Msg &NMEA0183Msg) {
  int iHandler;
  // Find handler
  for (iHandler=0; NMEA0183Handlers[iHandler].Code!=0 &&
         !NMEA0183Msg.IsMessageCode(NMEA0183Handlers[iHandler].Code);
       iHandler++);

  if (NMEA0183Handlers[iHandler].Code!=0) {
    NMEA0183Handlers[iHandler].Handler(NMEA0183Msg);
  }
}

/*
char lc(char c) {
  if ( (c  >= 'A') && (c <= 'Z') ) {
  return c - 'A' + 'a';
}

byte hex_byte_to_int(char *hex) {
  char c1 = lc(*hex);
  char c2 = lc(*(hex+1));
  byte val1, val2;
  byte sum;

  if (c1 >= 'a') {
    val1 = c1 - 'a' + 16;
  }
  if (c2 >= 'a') {
    val2 = c2 - 'a' + 16;
  }
  sum = val1*16 + val2;

 return sum;
}
*/

/* Parse an NMEA sentence from a C string. */
bool parse_sentence(const char *buf) {
  tNMEA0183Msg msg = tNMEA0183Msg();
  if (msg.SetMessage(buf)) {
    LOG_DEBUG("Handling NMEA %s", msg.MessageCode());
    HandleNMEA0183Msg(msg);
    return true;
  } else {
    LOG_DEBUG("NMEA bad checksum");
    return false;
  }
}

  /*
  if (len < NMEA_MIN_SENTENCE_LENGTH) return false;
  if (! ((buf[0] == '$') || (buf[0] == '!')) ) return false;
  if ((buf[len-1 - 2] != '*')) return false;

  byte given_sum = hex_byte_to_int(buf+len-1-1);
  byte calced_sum = 0;
  for (int i = 1; i < (len-3); i++) {
    calced_sum ^= buf[i];
  }
  */


    /* Example data from my boat:
     $IIMTW,22,C*0D
     $IIMWD,,,,,01.16,N,00.60,M*5D
     $IIMWV,291,R,01.16,N,A*2F
     $GPGLL,5312.324,N,00547.752,E,194510,A*24
     !AIVDM,1,1,,A,13`h@GPP00PJLKtNL:Kf4?vD2865,0*67
     !AIVDM,1,1,,A,13aOAr0P00PJR;bNLMA>4?vD2D0V,0*51
     $GPRMC,194510,A,5312.324,N,00547.752,E,0.02,000.00,,,*11
     $IIVDR,,,,,,N*0E
     $IIVHW,,T,,M,00.00,N,00.00,K*55
     $IIVLW,1603.64,N,,*2B
     !AIVDM,1,1,,B,13aJH>@P2;PHFh0NKsdTTgvJ2`6K,0*03
     $IIVWT,022,L,01.34,N,00.69,M,,*23
     $GPRMB,A,,R,,,,,,,,,,V*23
     $GPZDA,194511,,,,00,*41
     $IIDPT,001.16,0.40*42
     !AIVDM,1,1,,A,13aI8U?P00PJel>NKbwN4?vF2<0d,0*1C
     $GPGLL,5312.324,N,00547.752,E,194511,A*25
     $GPGSA,A,3,02,23,07,05,09,06,03,29,30,26,,,1.4,,0.7*1F
     !AIVDM,1,1,,A,13aGu8gP000JOP0NLBi@0?vF06Jl,0*5B
     $GPGSV,3,3,12,40,14,124,28,26,12,034,25,03,10,118,21,29,09,342,25*78
     $IIMTW,22,C*0D
  */
