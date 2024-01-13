#pragma once

#include <cstddef>
#include <cstdint>

// For millis()
#include <Arduino.h>
#include "constants.h"

/*
  Want Ground wind speed and direction.
  Best derive from apparent wind, compass, and GNSS SOG/COG.

  AWA + true heading -> AWD

  Water referenced true wind messages are ambigious and probably wrong,
  because it can't really be done without a leeway sensor. We need GPS anyway
  for report location, so might as well work from apparent to start with.
*/

template<typename T>
class Sensor {
 public:
 Sensor(uint32_t _valid_period = 5000) : valid_period{_valid_period} {};

  bool valid() {
    return millis() < (timestamp + valid_period);
  }

  void set(T value) {
    timestamp = millis();
    val = value;
  }

  bool get(T& value) {
    value = val;
    return valid();
  }

  T get() {
    return val;
  }

 protected:
  T val;
  uint32_t timestamp;
  uint32_t valid_period;
};

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

  T average() {
    T out = data[0];
    for (size_t i = 1; i < length; i++) {
      out += data[i];
    }
    return out;
  }

 protected:
  T data[length];
  size_t tail = 0;
  bool started = true;

  void reset() {
    started = false;
  }
};

struct NyanVessel {
  SensorAveraging<double, 10> HDT;
  SensorAveraging<double, 10> AWS;
  SensorAveraging<double, 10> AWA;

  Sensor<double> COG;
  Sensor<double> SOG;

  Sensor<double> water_temp;
};
