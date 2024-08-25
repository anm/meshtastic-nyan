/*
  Pins needed to allocate:
  JTAG
  I2C
  Lightning sensor interrupt pin
  SPI, for CAN, or CAN directly.
  UART for NMEA0183


  GPIO39 	MTCK
  GPIO40 	MTDO
  GPIO41 	MTDI
  GPIO42 	MTMS
  GPIO3 	Switch: LOW for JTAG on pins, HIGH for USB JTAG. Function fuse dependent.
*/

#define I2C_SCL 47
#define I2C_SDA 48

// Lightning sensor
#define USE_AS3935

#define USE_INA3221
// Which I2C bus: Wire, Wire1, ...
#define INA3221_BUS Wire

/*
GND 1000000 64
VS  1000001 65
SDA 1000010 66
SCL 1000011 67
*/
#define INA3221_ADDR 67

#define USE_NMEA_SERIAL
#define NMEA0183_UART_TX 6
#define NMEA0183_UART_RX 7

#define LED_PIN LED

#define HAS_SCREEN 0
#define HAS_TELEMETRY 0
#define HAS_SENSOR 0

#define VEXT_ENABLE Vext // active low, powers the oled display and the lora antenna boost
#define VEXT_ON_VALUE LOW
#define BUTTON_PIN 0

#define ADC_CTRL 37
#define ADC_CTRL_ENABLED LOW
#define BATTERY_PIN 1 // A battery voltage measurement pin, voltage divider connected here to measure battery voltage
#define ADC_CHANNEL ADC1_GPIO1_CHANNEL
#define ADC_ATTENUATION ADC_ATTEN_DB_2_5 // lower dB for high resistance voltage divider
#define ADC_MULTIPLIER 4.9 * 1.045

#define USE_SX1262

#define LORA_DIO0 -1 // a No connect on the SX1262 module
#define LORA_RESET 12
#define LORA_DIO1 14 // SX1262 IRQ
#define LORA_DIO2 13 // SX1262 BUSY
#define LORA_DIO3    // Not connected on PCB, but internally on the TTGO SX1262, if DIO3 is high the TXCO is enabled

#define LORA_SCK 9
#define LORA_MISO 11
#define LORA_MOSI 10
#define LORA_CS 8

#define SX126X_CS LORA_CS
#define SX126X_DIO1 LORA_DIO1
#define SX126X_BUSY LORA_DIO2
#define SX126X_RESET LORA_RESET

#define SX126X_DIO2_AS_RF_SWITCH
#define SX126X_DIO3_TCXO_VOLTAGE 1.8
