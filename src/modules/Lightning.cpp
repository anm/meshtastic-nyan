// Author: River MacLeod

#include "Lightning.h"

#include "configuration.h"
#include "SparkFun_AS3935.h"

// I2C address is set by DIP switches on the sensor board
const uint8_t AS3935_ADDR    = 0x03;
const uint8_t AS3935_INDOOR  = 0x12;
const uint8_t AS3935_OUTDOOR = 0xE;
const uint8_t LIGHTNING_INT  = 0x08;
const uint8_t DISTURBER_INT  = 0x04;
const uint8_t NOISE_INT      = 0x01;

SparkFun_AS3935 lightning(AS3935_ADDR);

// Interrupt pin for lightning detection
const uint8_t LIGHTNING_IRQ_PIN = 15;

// Used for oscillator measurement
volatile uint32_t AS3935_pulse_count = 0;

void IRAM_ATTR AS3935_count_pulse(void) {
  ++AS3935_pulse_count;
}

uint32_t AS3935_measure_frequency(void) {
  uint32_t start_time, stop_time;

  attachInterrupt(digitalPinToInterrupt(LIGHTNING_IRQ_PIN), AS3935_count_pulse, RISING);
  AS3935_pulse_count = 0;
  start_time = micros();
  delay(1000);
  detachInterrupt(digitalPinToInterrupt(LIGHTNING_IRQ_PIN));
  stop_time = micros();

  uint32_t measurement_period = stop_time - start_time;

  // depends on setting
  const uint16_t f_output_divisor = 16;

  uint32_t f = f_output_divisor * (double) AS3935_pulse_count * 1E6 /
               (double) measurement_period;

  LOG_DEBUG("AS3935_measure_frequency(): counts: %u, time: %u, frequency: %i Hz\n",
        AS3935_pulse_count, measurement_period, f);

  return f;
}

bool AS3935_calibrate(void) {
  uint32_t frequency;
  int32_t f_error;
  int32_t min_f_error;
  uint32_t min_f_error_cap;

  lightning.changeDivRatio(0); // div 16
  lightning.displayOscillator(true, 3);

  constexpr uint32_t F_NOMINAL = 500000;
  constexpr uint32_t F_MAX = F_NOMINAL * 1.035;
  constexpr uint32_t F_MIN = F_NOMINAL * 0.965;

  LOG_INFO("Calibrating AS3935 Lightning Sensor\n");
  LOG_DEBUG("Lightning: Calibrating LCO. Acceptable range %lu kHz to %lu kHz\n",
        F_MIN, F_MAX);

  uint8_t cap;
  for (cap = 0; cap <= 120; cap +=8) {
    lightning.tuneCap(cap);

    frequency = AS3935_measure_frequency();
    f_error = frequency - F_NOMINAL;

    LOG_DEBUG("Trimcap: %upF, Error: %li Hz, %.1f%%\n",
          cap, f_error, (100.0 * f_error/(double)F_NOMINAL));
    if (cap == 0) {
      min_f_error = f_error;
      min_f_error_cap = 0;
    } else {
      if (labs(f_error) < labs(min_f_error)) {
        min_f_error = f_error;
        min_f_error_cap = cap;
      }
    }
  }

  lightning.displayOscillator(false, 3);

  LOG_DEBUG("Lowest error: %.1f%%. Using %upF.\n",
        (100.0 * min_f_error / (double)F_NOMINAL),
        min_f_error_cap);

  lightning.tuneCap(min_f_error_cap);

  bool f_error_acceptable = labs(min_f_error) < (F_NOMINAL * 0.035);
  if (!f_error_acceptable) {
    LOG_ERROR("Lightning LC Osc calibration failed - best f out of range.\n");
    return false;
  }

  // Calibrate the other two internal oscillators from the LC one just done.
  bool int_cal_ok = lightning.calibrateOsc();
  if (!int_cal_ok) {
    LOG_ERROR("Lightning: Calibrating internal oscillators failed.\n");
    return false;
  }

  return true; // Calibration succedded.
}

void AS3935_setup(void) {
  // When lightning is detected the interrupt pin goes HIGH.
  pinMode(LIGHTNING_IRQ_PIN, INPUT);

  // Needed, but expected to be done already by meshtastic
  Wire.begin();

  LOG_INFO("AS3935 Lightning sensor starting.\n");

  if (!lightning.begin()) {
    LOG_ERROR("Lightning sensor begin() failed.\n");
    return;
  }

  AS3935_calibrate();

  lightning.setIndoorOutdoor(OUTDOOR);
  //lightning.setIndoorOutdoor(INDOOR);

  // 1-7. Higher gives greater noise rejection.
  lightning.setNoiseLevel(2); // 5 is high enough in the hacklab

  // 1-10. Squelch level.
  //lightning.watchdogThreshold(threshVal);

  // 0 to 15, higher rejects more. Default is 2.
  // no disturbances in my lab when set to 10. Many at 2.
  lightning.spikeRejection(2);

  int int_reg = lightning.readInterruptReg();
  LOG_DEBUG("Initial read of int reg: %u\n", int_reg);
}

void AS3935_check_lightning(void) {
  uint8_t distance;

  if (digitalRead(LIGHTNING_IRQ_PIN)) {
    int intVal = lightning.readInterruptReg();

    switch (intVal) {
      // TODO: Increment counters for this reporting period.

    case NOISE_INT:
      LOG_INFO("Lightning Sensor: Noise floor too high\n");
      break;

    case DISTURBER_INT:
      LOG_INFO("Lightning Sensor: Disturber (QRM) detected\n");
      break;

    case LIGHTNING_INT:
      LOG_INFO("Lightning Sensor: Lightning detected!\n");

      distance = lightning.distanceToStorm();
      LOG_INFO("Lightning distance approx %u km\n", distance);

      // TODO: Send report immediatly, I think.
      break;

    default:
      LOG_ERROR("Lightning Sensor: Unknown value in interrupt register: %u\n", intVal);
      break;
    }
  }
}
