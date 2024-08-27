#pragma once

#include <cstddef>
#include <cstdint>

// For millis()
#include <Arduino.h>
#include "constants.h"

// From meshtastic, used for debugging output
#include "configuration.h"

template<typename T>
class Statistics;

template<typename T>
class Sensor {
 public:
  Sensor(uint32_t _valid_period = 10000);

  Statistics<T> stats{this};

  bool valid();
  void set(T value);
  T get();

 protected:
  T val;
  uint32_t timestamp;
  uint32_t valid_period;
};

template<typename T> class Statistics {
public:
  Statistics(Sensor<T> *_sensor);

  /* Read the sensor by calling sensor.get() and use the value for
     statistics. Sample will only be taken if sensor is .valid().
  */
  void sample();

  /* Set the sensor and immediatly sample it. */
  void sample(T v);

  /* Returns the proportion attempted samplings that had valid data, as a
     value between 0 and 1. */
  double quality();

  /* Clear samples and prepare for a new reporting period. */
  void reset();

  T max();
  T mean();

protected:
  Sensor<T> *sensor;
  size_t read_attempts = 0;
  size_t count = 0;
  T sum = 0;
  T _max;
};

template<typename T>
Sensor<T>::Sensor(uint32_t _valid_period) : valid_period{_valid_period} {};

template<typename T>
bool Sensor<T>::valid() {
  return millis() < (timestamp + valid_period);
}

template<typename T>
void Sensor<T>::set(T value) {
  timestamp = millis();
  val = value;
}

template<typename T>
T Sensor<T>::get() {
  return val;
}

/* Provides a running average of a sensor value. */
template<typename T, size_t length>
class SensorAveraging : public Sensor<T> {
public:
 SensorAveraging(uint32_t _valid_period = 5000) : Sensor<T>{_valid_period} {};

  void sample() {
    T d = Sensor<T>::val / length;

    if (!Sensor<T>::valid()) {
      reset();
      return;
    }

    if (started) {
      for (size_t i = 0; i < length; i++) {
        data[i] = d;
      }
      started = false;
    } else {
      tail = (tail + 1) % length;
      data[tail] = d;
    }
  }

  /* Return the average value. */
  /* "overrides" Sensor get(), but not using virtual functions. */
  T get() {
    T out = data[0];
    for (size_t i = 1; i < length; i++) {
      out += data[i];
    }
    return out;
  }

  /* Returns latest unfiltered sensor value. */
  T raw() {
    return Sensor<T>::val;
  }

 protected:
  T data[length];
  size_t tail = 0;
  bool started = true;

  void reset() {
    started = false;
  }
};

/*
 * Collect statistics on Sensor values.
 *
 * N.B.: Sampled values are summed, and could overflow. Check the total
 * does not exceed the max value of the used type.
 */
template<typename T>
Statistics<T>::Statistics(Sensor<T> *_sensor) : sensor{_sensor} {};

/* Read the sensor by calling sensor.get() and use the value for
   statistics. Sample will only be taken if sensor is .valid().
*/
template<typename T>
void Statistics<T>::sample() {
  read_attempts++;
  // FIXME maybe: potential race condition
  if (!sensor->valid()) return;
  T v = sensor->get();
  count++;
  sum += v;
  if (count == 1) {
    _max = v;
  } else {
    if (v > _max) {
      _max = v;
    }
  }
}

template<typename T>
void Statistics<T>::sample(T v) {
  sensor->set(v);
  sample();
}

/* Returns the proportion of attempted samplings that had valid data, as a
   value between 0 and 1. */
template<typename T>
double Statistics<T>::quality() {
  if (read_attempts == 0) {
    return 0;
  }

  return count / (double) read_attempts;
}

/* Clear samples and prepare for a new reporting period. */
template<typename T>
void Statistics<T>::reset() {
  read_attempts = 0;
  count = 0;
  sum = 0;
}

template<typename T>
T Statistics<T>::max() {
  return _max;
}

template<typename T>
T Statistics<T>::mean() {
  return sum / count;
}

struct Position {
  double latitude;
  double longitude;

  double COG;
  double SOG;

  // Note that the stored data is valid at the time this function is called.
  void set_valid() {
    LOG_DEBUG("Setting Position timestamp.\n");
    timestamp = millis();
  }

  bool valid() {
    // TODO: Also check fix quality from GNSS data
    return timestamp_valid();
  }

  bool timestamp_valid() {
    return millis() < (timestamp + valid_period);
  }

protected:
  // uC mS counter at time message saved. Not time of fix (but probably close).
  uint32_t timestamp;
  uint32_t valid_period = 20000;
};

struct NyanVessel {
  Position position_gnss_builtin;
  Position position_nmea;

  SensorAveraging<double, 10> HDT;
  SensorAveraging<double, 10> AWS;
  SensorAveraging<double, 10> AWA;

  // Derived values
  Sensor<double> GWS;
  Sensor<double> GWD;

  Sensor<double> water_temperature;
  Sensor<double> water_depth;
  Sensor<double> water_depth_below_keel;

  bool getPosition(Position *p) {
    /* I would like to choose the best position source, but for now will just
       decide to use internal GNSS, then NMEA one if the first isn't valid. */

    if (this->position_gnss_builtin.valid()) {
      *p = this->position_gnss_builtin;
      LOG_DEBUG("Using gnss_builtin\n");
      return true;
    }

    if (this->position_nmea.valid()) {
      *p = this->position_nmea;
      LOG_DEBUG("Using position_nmea\n");
      return true;
    }

    LOG_DEBUG("No position available\n");
    return false;
  }
};
