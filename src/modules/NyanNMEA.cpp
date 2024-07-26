/* Based on example code */

#include "NyanNMEA.h"
#include "NyanVessel.h"

#include "gps/RTC.h"

extern NyanVessel v;

// From meshtastic, used for debugging output
#include "configuration.h"

void handle_MTW(const tNMEA0183Msg &msg) {
  if ((msg.FieldCount() >= 2) && (msg.Field(1)[0] == 'C')) {
    double temp = NMEA0183GetDouble(msg.Field(0));
    v.water_temperature.set(temp);
    LOG_INFO("Parsed MTW: %.2fC\n", temp);
  }
}

void handle_HDT(const tNMEA0183Msg &msg) {
  if ((msg.FieldCount() >= 2) && (msg.Field(1)[0] == 'T')) {
    double heading = NMEA0183GetDouble(msg.Field(0));
    v.HDT.set(heading);
    LOG_INFO("Parsed HDT: %f\n", heading);
  }
}

void handle_MWV(const tNMEA0183Msg &msg) {
  // I don't know if this message will always have the status field or if it
  // might be missing in older versions. Will be more tolerant.
  if ((msg.FieldCount() >= 5) && (msg.Field(4)[0] != 'A')) {
    // Status flag present but not indicating data valid
    LOG_DEBUG("MWV: Flagged invalid.\n");
    return;
  }

  if (msg.FieldCount() < 4) {
    return;
  }

  /* Only accept relative wind angle, because True/Theoretical must have been
     calculated, and I can't be sure how that was done. Better DIY by known
     method if it is wanted. */
  char reference = msg.Field(1)[0];
  if (reference != 'R') {
    LOG_DEBUG("MWV: skipping non 'R' reference.\n");
    return;
  }

  double wind_speed = NMEA0183GetDouble(msg.Field(2));
  char unit = msg.Field(3)[0];
  if (unit == 'M') {
    wind_speed *= MPS_TO_KNOTS;
  } else if (unit == 'K') {
    wind_speed *= KPH_TO_KNOTS;
  } else if (unit == 'N') {
    // in knots already
  } else {
    // Unknown / invalid unit
    LOG_WARN("MWV: Unknown unit\n");
    return;
  }

  v.AWS.set(wind_speed);

  // Angle should be in degrees already
  double wind_angle = NMEA0183GetDouble(msg.Field(0));
  v.AWA.set(wind_angle);

  LOG_INFO("MWV - AWS: %f AWA: %f\n", wind_speed, wind_angle);
}

// Always in meters
//
// The data from ww's YD box gives a DBS message when offset is negative, of
// what is actually DBK. I suspect just using DPT will be most trustworthy.
// YD also says that DBK / DBS / DBT are obsolete.
//
// My boat gave DPT with correct positive offset.
void handle_DPT(const tNMEA0183Msg &msg) {
  double dbt, offset, range, dbs;
  NMEA0183ParseDPT_nc(msg, dbt, offset, range);
  if ((dbt == NMEA0183DoubleNA) || (offset == NMEA0183DoubleNA)) {
    LOG_WARN("Failed to parse double from DPT message.\n");
    return;
  }

  if (offset >= 0) {
    dbs = dbt + offset;
  } else {
    // Don't care about depth below keel. Need depth below surface.
    // dbk = dbt + offset
    LOG_DEBUG("Ignoring DPT message with negative offset (depth below keel).\n");
    return;
  }
  v.water_depth.set(dbs);
  LOG_DEBUG("Set water depth to %f from DPT message\n", dbs);
}

void handle_RMC(const tNMEA0183Msg &msg) {
  double GPSTime;
  char status;
  double latitude;
  double longitude;
  double COG;
  double SOG;
  unsigned long int DaysSince1970;
  double variation;
  time_t datetime;
  char FAAModeIndicator;
  char NavStatus;

  NMEA0183ParseRMC_nc(msg, GPSTime, status, latitude, longitude,
                      COG, SOG, DaysSince1970, variation,
                      FAAModeIndicator, NavStatus, &datetime);

  if ((status != 'A') || (NavStatus == 'V')) {
    LOG_DEBUG("Invalid position from NMEA RMC\n");
    return;
  }

  v.position_nmea.set_valid();

  LOG_DEBUG("Setting data from RMC: COG: %f SOG: %f lat: %f lon: %f\n",
            COG, SOG, latitude, longitude);

  v.position_nmea.COG = COG;
  v.position_nmea.SOG = SOG;
  v.position_nmea.latitude  = latitude;
  v.position_nmea.longitude = longitude;
}

struct tNMEA0183Handler {
  const char *Code;
  void (*Handler)(const tNMEA0183Msg &NMEA0183Msg);
};

tNMEA0183Handler NMEA0183Handlers[]={
  {"DPT",&handle_DPT},
  {"RMC",&handle_RMC},
  {"HDT",&handle_HDT},
  {"MTW",&handle_MTW},
  {"MWV",&handle_MWV},
  {0,0}
};

void HandleNMEA0183Msg(const tNMEA0183Msg &NMEA0183Msg) {
  LOG_DEBUG("HandleNMEA0183Msg() running");

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
    //LOG_DEBUG("Handling NMEA %s\n", msg.MessageCode());
    HandleNMEA0183Msg(msg);
    return true;
  } else {
    LOG_WARN("NMEA bad checksum\n");
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


tNMEA0183 NMEA0183_Serial;

void NMEA_serial_setup(void) {
  LOG_INFO("Starting NMEA serial");
  Serial2.begin(4800, SERIAL_8N1);
#ifdef ARCH_RP2040
  Serial2.setPinout(NMEA0183_UART_TX, NMEA0183_UART_RX);
#else
  Serial2.setPins(NMEA0183_UART_RX, NMEA0183_UART_TX);
#endif

  NMEA0183_Serial.SetMessageStream(&Serial2);
  NMEA0183_Serial.SetMsgHandler(HandleNMEA0183Msg);
  bool opened = NMEA0183_Serial.Open();
  if (!opened) LOG_ERROR("NMEA0183_Serial.Open() failed");
}

void NMEA_serial_loop(void) {
  LOG_DEBUG("NMEA serial loop");
  NMEA0183_Serial.ParseMessages();
}
