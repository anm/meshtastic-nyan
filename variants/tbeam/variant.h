//#define USE_NYAN_TEST_DATA
#define DISABLE_INA_CHARGING_DETECTION

// Which I2C bus: Wire, Wire1, ...
#define INA3221_BUS Wire1

/*
  INA3221 Address is set by jumping it's select pin as follows:
  GND 1000000 64
  VS  1000001 65
  SDA 1000010 66
  SCL 1000011 67
*/
#define INA3221_ADDR 67

#define HAS_SCREEN 1
#define HAS_TELEMETRY 0
#define HAS_SENSOR 0

//#define USE_NMEA_SERIAL
//#define NMEA0183_UART_TX 15
//#define NMEA0183_UART_RX 35

// Enable secondary bus for external peripherals
#define SDA 32
#define SCL 33
//#define I2C_SDA1 SDA
//#define I2C_SCL1 SCL

// Used for screen
#define I2C_SDA 21
#define I2C_SCL 22

#define BUTTON_PIN 38 // The middle button GPIO on the T-Beam
#define EXT_NOTIFY_OUT 13 // Default pin to use for Ext Notify Module.

#define LED_STATE_ON 0 // State when LED is lit
#define LED_PIN 4      // Newer tbeams (1.1) have an extra led on GPIO4

// TTGO uses a common pinout for their SX1262 vs RF95 modules - both can be enabled and we will probe at runtime for RF95 and if
// not found then probe for SX1262
#define USE_RF95 // RFM95/SX127x
#define USE_SX1262
#define USE_SX1268

#define LORA_DIO0 26 // a No connect on the SX1262 module
#define LORA_RESET 23
#define LORA_DIO1 33 // SX1262 IRQ
#define LORA_DIO2 32 // SX1262 BUSY
#define LORA_DIO3    // Not connected on PCB, but internally on the TTGO SX1262, if DIO3 is high the TXCO is enabled

#ifdef USE_SX1262
#define SX126X_CS LORA_CS // FIXME - we really should define LORA_CS instead
#define SX126X_DIO1 LORA_DIO1
#define SX126X_BUSY LORA_DIO2
#define SX126X_RESET LORA_RESET
// Not really an E22 but TTGO seems to be trying to clone that
#define SX126X_DIO2_AS_RF_SWITCH
#define SX126X_DIO3_TCXO_VOLTAGE 1.8
// Internally the TTGO module hooks the SX1262-DIO2 in to control the TX/RX switch (which is the default for the sx1262interface
// code)
#endif

// Leave undefined to disable our PMU IRQ handler.  DO NOT ENABLE THIS because the pmuirq can cause sperious interrupts
// and waking from light sleep
// #define PMU_IRQ 35
#define HAS_AXP192
#define GPS_UBLOX
#define GPS_RX_PIN 34
#define GPS_TX_PIN 12
// #define GPS_DEBUG
