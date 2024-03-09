/* Code based on example from NMEA2000 library. */

#include <Arduino.h>

// From meshtastic, used for debugging output
#include "configuration.h"

#include <NMEA2000_mcp.h>
#include <N2kMessages.h>
#include <N2kMessagesEnumToStr.h>

#ifdef HELTEC_V3
/*
const uint8_t CAN_SPI_INTERUPT_PIN = 2;
const uint8_t CAN_SPI_CLOCK_PIN = 5;
const uint8_t CAN_SPI_MISO_PIN = 7;
const uint8_t CAN_SPI_MOSI_PIN = 4;
const uint8_t CAN_SPI_SS_PIN = 6;
*/

const uint8_t CAN_SPI_INTERUPT_PIN = 19; // D2
const uint8_t CAN_SPI_CLOCK_PIN = 38; // D4
const uint8_t CAN_SPI_MISO_PIN = 33; // D3
const uint8_t CAN_SPI_MOSI_PIN = 34; // D6
const uint8_t CAN_SPI_SS_PIN = 20; // D5
#endif

#ifdef HELTEC_WSL_V3
// TODO, just copied from other heltec for now

const uint8_t CAN_SPI_INTERUPT_PIN = 19; // D2
const uint8_t CAN_SPI_CLOCK_PIN = 38; // D4
const uint8_t CAN_SPI_MISO_PIN = 33; // D3
const uint8_t CAN_SPI_MOSI_PIN = 34; // D6
const uint8_t CAN_SPI_SS_PIN = 20; // D5
#endif

// (it's actually a V1.2 but this is what meshtastic defines)
#ifdef TBEAM_V10
const uint8_t CAN_SPI_INTERUPT_PIN = 2; // D2
const uint8_t CAN_SPI_CLOCK_PIN = 13; // D4
const uint8_t CAN_SPI_MISO_PIN = 25; // D3
const uint8_t CAN_SPI_MOSI_PIN = 33; // D6
const uint8_t CAN_SPI_SS_PIN = 14; // D5
#endif

const unsigned char MCP_CLOCK_SPEED = MCP_16MHz;
const uint16_t RX_BUFFER_SIZE = 256;

typedef struct {
  unsigned long PGN;
  void (*Handler)(const tN2kMsg &N2kMsg);
} tNMEA2000Handler;

void SystemTime(const tN2kMsg &N2kMsg);

tNMEA2000Handler NMEA2000Handlers[]={
  {126992L,&SystemTime},
  {0,0}
};

tNMEA2000_mcp *n2k;

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);

template<typename T>
void PrintLabelValWithConversionCheckUnDef(const char* label,
                                           T val,
                                           double (*ConvFunc)(double val)=0,
                                           bool AddLf=false, int8_t Desim=-1 ) {
  LOG_DEBUG(label);
  if (!N2kIsNA(val)) {
    if (ConvFunc) { LOG_DEBUG("%d", ConvFunc(val)); } else { LOG_DEBUG("%d", val); }
  } else {
    LOG_DEBUG("not available");
  }
}

void SystemTime(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    uint16_t SystemDate;
    double SystemTime;
    tN2kTimeSource TimeSource;

    if (ParseN2kSystemTime(N2kMsg,SID,SystemDate,SystemTime,TimeSource) ) {
                      LOG_DEBUG("System time:");
      PrintLabelValWithConversionCheckUnDef("  SID: ",SID,0,true);
      PrintLabelValWithConversionCheckUnDef("  days since 1.1.1970: ",SystemDate,0,true);
      PrintLabelValWithConversionCheckUnDef("  seconds since midnight: ",SystemTime,0,true);
      //                        LOG_DEBUG("  time source: "); PrintN2kEnumType(TimeSource,OutputStream);
    } else {
      LOG_DEBUG("Failed to parse PGN: %u\n", N2kMsg.PGN);
    }
}

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  int iHandler;

  LOG_DEBUG("Handling PGN %u\n", N2kMsg.PGN);

  // Find handler
  for (iHandler=0; NMEA2000Handlers[iHandler].PGN!=0 &&
         !(N2kMsg.PGN==NMEA2000Handlers[iHandler].PGN); iHandler++);

  if (NMEA2000Handlers[iHandler].PGN != 0) {
    NMEA2000Handlers[iHandler].Handler(N2kMsg);
  }
}

void nyan_N2K_setup() {
  /* TODO: Remember ID and settings as required by standard
   *  See NMEA2000/src/NMEA2000.h
   */

  n2k = new tNMEA2000_mcp(CAN_SPI_SS_PIN,
                          MCP_CLOCK_SPEED,
                          CAN_SPI_INTERUPT_PIN,
                          RX_BUFFER_SIZE);

  /* SPI numbers in Arduino / ESP32 are a total mess.
   * On *ESP32-S3* it looks like:
   *
   * FSPI =  0, and means SPI2
   * HSPI  = 1, and means SPI3

   * SPI0 and SPI1 are not accessible through Arduino.
   *
   * This nonsense cost be about three days.

   On *ESP32*:
   HSPI SPI2
   VSPI SPI3
   0 or 1: SPI01

   Arduino defaults, on including SPI.h:
   #if CONFIG_IDF_TARGET_ESP32
   SPIClass SPI(VSPI);
   #else
   SPIClass SPI(FSPI);
   #endif

   Meshtastic might use HSPI for SDCard or LoRa if config options defined, else
   arduino default.
  */

  // HSIP seems to be the non-default one on both ESP32 and ESP32-S3
  constexpr uint32_t CAN_SPI_SPEED = 1000000;
  SPIClass *spi = new SPIClass(HSPI);

  pinMode(CAN_SPI_CLOCK_PIN, OUTPUT);
  pinMode(CAN_SPI_SS_PIN, OUTPUT);
  pinMode(CAN_SPI_MOSI_PIN, OUTPUT);

  spi->begin(CAN_SPI_CLOCK_PIN, CAN_SPI_MISO_PIN, CAN_SPI_MOSI_PIN, CAN_SPI_SS_PIN);
  spi->setFrequency(CAN_SPI_SPEED);

  // Use hardware chip select
  spi->setHwCs(true);

  n2k->SetSPI(spi);

  /*
  for (int t = 100; t > 0; --t) {
    spi->beginTransaction(SPISettings(CAN_SPI_SPEED, MSBFIRST, SPI_MODE0));
    digitalWrite(spi->pinSS(), LOW); //pull SS slow to prep other end for transfer
    spi->transfer(42);
    digitalWrite(spi->pinSS(), HIGH); //pull ss high to signify end of data transfer
    spi->endTransaction();

    delay(10);
  }
  */

    /*
  while (1) {
    digitalWrite(CAN_SPI_CLOCK_PIN, 1);
    digitalWrite(CAN_SPI_SS_PIN, 0);
    delay(1);
    digitalWrite(CAN_SPI_CLOCK_PIN, 0);
    digitalWrite(CAN_SPI_SS_PIN, 1);
    delay(1);

    //    spi->writeBytes((uint8_t*) "meow meow test", 15);
  }
  */

  //    void InterruptHandler();
  n2k->SetProductInformation("1", // Model serial code
                             1, // Product code
                             "NYAN", // Model ID
                             "0.0.1", // SW version
                             "0.0.1", // Model Version
                             3 // Load Equivalence number (50mA units) (TODO)
                             );

  // Device function codes from:
  // https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf

  n2k->SetDeviceInformation(4,    // Unique number (Selected by fair dice roll) (FIXME)
                            160,  // Device function "160: Data Receiver/Transceiver"
                            70,   // Device class
                            2046 // Manufacturer code
                            );

  n2k->SetForwardType(tNMEA2000::fwdt_Text);
  //  n2k->SetForwardStream();
  n2k->EnableForward(false);
  n2k->SetMsgHandler(HandleNMEA2000Msg);
  //  nk2->SetN2kCANReceiveFrameBufSize(50);
  //  n2k->SetN2kCANMsgBufSize(2);

  if (n2k->Open()){
    LOG_DEBUG("Opened CAN bus\n");
    n2k->ParseMessages(); // Docs says needs to be called promptly after open.
  } else {
    LOG_ERROR("Failed to open CAN bus\n");
  }

  LOG_DEBUG("CAN: nyan_N2K_setup done\n");
}

void nyan_N2K_loop() {
  n2k->ParseMessages();
}
