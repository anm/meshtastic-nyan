// Author: River MacLeod

#pragma once

void AS3935_setup(void);
void AS3935_check_lightning(void);

struct LightningCounter {
  uint32_t strokes_in_period;
  uint32_t strokes_since_start;
  uint32_t qrm_in_period;
  uint32_t qrm_since_start;
  uint32_t noise_in_period;
  uint32_t noise_since_start;

  uint8_t distance;

  void note_distance(uint8_t d) {
    distance = d;
  }

  void get_distance() {
    return distance;
  }

  void inc_strokes() {
    strokes_in_period++;
    strokes_since_start++;
  }

  void inc_qrm() {
    qrm_in_period++;
    qrm_since_start++;
  }

  void inc_noise() {
    noise_in_period++;
    noise_since_start++;
  }

  uint32_t get_strokes() {
    return strokes_in_period;
  }

  uint32_t get_strokes_ever() {
    return strokes_since_start;
  }

  uint32_t get_qrm() {
    return qrm_in_period;
  }

  uint32_t get_noise() {
    return noise_in_period;
  }

  void reset() {
    strokes_in_period = 0;
    qrm_in_period = 0;
    noise_in_period = 0;
    distance = 0;
  }
};
