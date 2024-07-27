#include <string.h>
#include <assert.h>

#include <WiFi.h>
#include "os_status.h"
#include "mesh/NodeDB.h"
#include "gps/RTC.h"

#include "configuration.h"
#include "main.h"
#include "MeshService.h"

#include "INA3221.h"
#include "Lightning.h"

#include "NyanModule.h"
#include "NyanNMEA.h"

#ifdef USE_N2K
#include "NyanN2K.h"
#endif

#include "NyanVessel.h"
#include "wind.h"

NyanVessel v;

/*
  Wind Reports

  Need running average and max over last reporting period - 10 mins seems to
  be a standard.

  First do a three second running average, then take the max and mean of
  that. This was the storage is bounded. Don't need more detail than three
  seconds.
*/

/* Weather station reporting period in milliseconds. */
constexpr uint32_t met_reporting_period = 600000;

/* Import NMEA0183 data from a tcp server.
   If not connected, connect.
   Read and parse sentences.
   Add data to ship data model.
*/
void NMEA_read() {
  tNMEA0183Msg NMEA0183Msg;
  tNMEA0183 NMEA0183;
  static WiFiClient tcp;

  const uint8_t NMEA_BUFFER_LENGTH = 85;
  // Including terminating charaters (CR LF)
  const uint8_t NMEA_MIN_SENTENCE_LENGTH = 11;

  static char nmea_buffer[NMEA_BUFFER_LENGTH];
  static uint8_t nmea_index = 0;

  const uint16_t port = 10110;
  const char *host = "shore.halekai.uk";

  if (!WiFi.isConnected()) {
    LOG_INFO("WiFi not connected\n");
    return;
  }

  if (! tcp.connected()) {
    LOG_WARN("TCP not connected\n");
    if (tcp.connect(host, port)) {
      LOG_INFO("TCP connected\n");
    } else {
      LOG_WARN("Connecting TCP NMEA failed.\n");
      return;
    }
  }

  /* Read and parse NMEA sentences. */
  constexpr uint32_t time_limit = 500; // mS
  uint32_t end_time = millis() + time_limit;
  //  while (tcp.available() && millis() < end_time) {
  while (tcp.available()) {
    nmea_buffer[nmea_index] = static_cast<char>(tcp.read());

    // There should be no null chars
    if (nmea_buffer[nmea_index] == 0) goto err;

    // Buffer is full and string not finished.
    if (nmea_index == NMEA_BUFFER_LENGTH - 1) goto err;

    // When LF encountered, string is finished.
    if (nmea_buffer[nmea_index] == 0x0a) {
      if ((nmea_index >= NMEA_MIN_SENTENCE_LENGTH) &&
          ((nmea_buffer[0] == '$') || (nmea_buffer[0] == '!')) &&
          // Check for CR
          (nmea_buffer[nmea_index-1] == 0x0d)) {

        // null terminate, to treat as string
        nmea_buffer[nmea_index-1] = 0;
        //LOG_DEBUG("Parsing %.6s\n", nmea_buffer);
        parse_sentence(nmea_buffer);
      }
      goto done;
    }

    // Move on and loop for next char
    nmea_index++;
    continue;

  err:
    LOG_DEBUG("NMEA parse error\n");
  done:
    nmea_index = 0;
  }
}

bool get_local_GPS(NyanVessel& v) {
  // FIXME: validity period will be longer than specified. Should call this
  // funcion when GPS supplies a fix.

  // Seconds
  uint32_t validity_period = 10;

  // FIXME: Timeformats
  // Fix timestamp is "in integer epoch seconds" (from GPS)
  // getValidTime gives what?

  if ((localPosition.timestamp > 0) &&
      ((localPosition.timestamp + validity_period) > getValidTime(RTCQualityFromNet)) &&
      (localPosition.fix_quality > 0)) {

    v.position_gnss_builtin.set_valid();

    v.position_gnss_builtin.COG = localPosition.ground_track * 100;
    v.position_gnss_builtin.SOG = localPosition.ground_speed * MPS_TO_KNOTS;

    v.position_gnss_builtin.latitude  = localPosition.latitude_i  * 1e-7;
    v.position_gnss_builtin.longitude = localPosition.longitude_i * 1e-7;

    LOG_DEBUG("Got fix from builtin GNSS. SOG %f COG %f\n",
              v.position_gnss_builtin.SOG, v.position_gnss_builtin.COG);
    return true;
  }
  return false;
}

void sample_NMEA_sensors(NyanVessel& v) {
  v.HDT.sample();
  v.AWA.sample();
  v.AWS.sample();
  LOG_DEBUG("Sensors after sampling: HDT: %f AWA (av): %f (%f) AWS (av) : %f %f\n",
            v.HDT.get(), v.AWA.raw(), v.AWA.get(),
            v.AWS.raw(), v.AWS.get());

  double GWS, GWD;
  if (Wind::derive_ground_wind(v, GWS, GWD)) {
    v.GWS.stats.sample(GWS);
    v.GWD.stats.sample(GWD);
  }
}

void signalk_test(NyanVessel v) {
  /*
   * Connect to SignalK by tcp and send delta message.
   *
   * Qs: will OpenCPN accept these directly?
   * UDP?
  */

  int node_id = 0;

  static WiFiClient tcp;

  const char *host = "nyan-host-0.river.cat";
  const uint16_t port = 8375;

  // SignalK security token
  // Generate with: signalk-generate-token -u nyan -e 10y -s /home/signalk/.signalk/security.json
  String token = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpZCI6Im55YW4iLCJpYXQiOjE3MTAwMDQ2NDMsImV4cCI6MjAyNTU4MDY0M30.HjqsMoUgywAL9oHmEnF3ZmhPqY7fOdgPgsOjeten8CI";

  if (!WiFi.isConnected()) {
    LOG_INFO("WiFi not connected\n");
    return;
  }

  if (! tcp.connected()) {
    if (tcp.connect(host, port)) {
      LOG_INFO("SignalK TCP connected\n");
    } else {
      LOG_WARN("Connecting SignalK TCP failed.\n");
      return;
    }
  }

  String s =
    R"({"context": "vessels.urn:mrn:nyan:)" + String(node_id) + R"(",)"

    // Sending token over tcp seems to be unimplemented by SignalK server
    //    R"("token": ")" + token + R"(",)"

    R"("updates": [{"source": {"label": "nyan", "type": "nyan"},)"

    // Timestamp for local data probably no better than letting SignalK do it on arrival
    // Timestamp for LoRa received data should probably come from the source
    //    R"("timestamp": ")" + String(timestamp) + R"(",)" // e.g. 2015-01-07T07:18:44Z
    R"("values": [)";

  // Path reference: http://signalk.org/specification/

  Position pos;
  bool havePosition;
  havePosition = v.getPosition(&pos);
  if (havePosition) {
    s +=
      R"({"path": "navigation.position", "value": {"latitude": )" +
      String(pos.latitude) + R"(, "longitude": )" + String(pos.longitude) + "}},";

    LOG_DEBUG("pos.latitude is %s by arduino, %f by printf\n", String(pos.latitude), pos.latitude);
  }

  LOG_DEBUG("v.GWS.stats.quality(): %f v.GWS.stats.mean(): %f\n",
            v.GWS.stats.quality(), v.GWS.stats.mean());

  if (v.GWS.stats.quality() > 0.1 && v.GWD.stats.quality() > 0.1) {
    LOG_DEBUG("SignalK: sending ground wind\n");

    R"({"path": "environment.wind.speedOverGround", "meta" : {"units": "C"}, "value": )"
        + String(v.GWS.stats.mean()) + "}"
      ",";
      R"({"path": "environment.wind.directionTrue", "meta" : {"units": "C"}, "value": )"
        + String(v.GWD.stats.mean()) + "}"
      ",";
  }

  /*
    environment/wind/directionTrue
    environment/wind/angleTrueGround
    environment/outside/pressure (Pa)
    environment/outside/relativeHumidity
  */

  if (v.water_temperature.valid()) {
    s +=
      R"({"path": "environment.water.temperature", "meta" : {"units": "C"}, "value": )"
      + String(v.water_temperature.get()) + "}"
      ",";
  }

  if (v.water_depth.valid()) {
    s +=
      R"({"path": "environment.depth.belowSurface", "value": )" + String(v.water_depth.get()) + "}"
      ",";
  }

  s +=
    // This is mostly here to have a guaranteed item at the end, that doesn't
    // have a comma after it, because JSON :(
    R"({"path": "nyan.uptime", "value": )" + String(millis()) + "}"
    "]}]}\r\n";

  LOG_INFO(s.c_str());
  tcp.print(s);
}

void NyanModule::send_report() {
  /* A struct generated by protobuf definition. */
  nyan_telemetry telemetry;

  // We have data to send
  bool send = false;

  // TODO check protobuf optionalness for what sending when value not available

  if (v.GWS.stats.quality() > 0) {
    send = true;
    telemetry.GWS_mean = (uint8_t) v.GWS.stats.mean();
    telemetry.GWS_gust = v.GWS.stats.max();

    telemetry.GWD_mean = (uint16_t) v.GWD.stats.mean();

    LOG_DEBUG("Sending Ground Wind %i kts, %i°T Gust: %i kts\n",
              telemetry.GWS_mean, telemetry.GWD_mean, telemetry.GWS_gust);
  }

  if (v.water_temperature.valid()) {
    send = true;
    telemetry.water_temperature = v.water_temperature.get();
    LOG_DEBUG("Water temperature %f°C\n", telemetry.water_temperature);
  }

  if (v.water_depth.valid()) {
    send = true;
    telemetry.water_depth = v.water_depth.get();
    LOG_DEBUG("Sending water depth %fm\n", telemetry.water_depth);
  }

  if (send) {
    LOG_INFO("Sending Nyan telemetry\n");

    meshtastic_MeshPacket *p = allocDataProtobuf(telemetry);
    p->decoded.want_response = false;
    p->priority = meshtastic_MeshPacket_Priority_RELIABLE;
    service.sendToMesh(p);
  } else {
    LOG_INFO("No valid data to report\n");
  }
}

bool NyanModule::handleReceivedProtobuf(const meshtastic_MeshPacket &mp,
                                        nyan_telemetry *telemetry) {
  if (telemetry == NULL) {
    LOG_WARN("handleReceivedProtobuf() got null protobuf decode\n");
    return false;
  }

  screen->print("Nyan RXed ");

  LOG_DEBUG("handleReceivedProtobuf() called. "
            "Got GWS_mean: %u, GWS_gust: %u, GWD_mean: %u\n",
            telemetry->GWS_mean,
            telemetry->GWS_gust,
            telemetry->GWD_mean);

  return true;
}

/* Periodically send ship data over mesh. */

/* Can run every time the thread is scheduled.
   Delayed to meet desired period, which is the return value.
*/
int32_t NyanModule::runOnce() {
  // NMEA tcp
  NMEA_read();

#ifdef USE_N2K
  nyan_N2K_loop();
#endif

#ifdef USE_NMEA_SERIAL
  NMEA_serial_loop();
#endif

  get_local_GPS(v);

  signalk_test(v);
  sample_onboard_sensors();

  AS3935_check_lightning();

  LOG_DEBUG("No of meshtastic tasks: %u\n", task_count());

  return 3000; // period in milliseconds
}

INA3221 ina3221 = INA3221(&INA3221_BUS, (ina3221_addr_t) INA3221_ADDR);
//INA3221 ina3221 = INA3221((ina3221_addr_t) INA3221_ADDR);

void INA3221_setup(void) {
  LOG_INFO("INA3221_setup\n");

  ina3221.setShuntRes(50, 100, 100); // In milliOhms
  ina3221.setFilterRes(10, 10, 10); // In Ohms

  delay(10);
  ina3221.reset();
  delay(10);

  if (ina3221.getManufID() == 0x5449) {
    LOG_DEBUG("Read INA3221 Maufacturer ID OK.\n");
  } else {
    LOG_ERROR("Read INA3221 Maufacturer ID Failed.\n");
  }

  delay(10);
}

/* Sample I2C Sensors */
void NyanModule::sample_onboard_sensors(void) {
  /*
  Wire1.end();

  bool set_ok;
  set_ok =  Wire1.setSDA(I2C_SDA1);
  set_ok &= Wire1.setSCL(I2C_SCL1);
  if (!set_ok) {
    LOG_ERROR("Could not set I2C pins.");
  }

  Wire1.begin();
  */

  float V_in = ina3221.getVoltage(INA3221_CH1);
  float I_in = ina3221.getCurrentCompensated(INA3221_CH1) / 1000.0;
  float V_5V = ina3221.getVoltage(INA3221_CH2);

  LOG_DEBUG("INA3221 V_in: %.3fV\t I_in: %.3fA\t V_5V: %.3fV\n", V_in, I_in, V_5V);
}

/* A FreeRTOS task to read / filter / store sensor data. */
void NyanModule::sensor_sampler_task(void *params) {
  while(true) {
    LOG_DEBUG("NYAN Sampling sensors\n");
    LOG_DEBUG("NYAN sampler stack high water mark: %u\n",
              uxTaskGetStackHighWaterMark(NULL));
    sample_NMEA_sensors(v);
    //    sample_onboard_sensors();
    delay(5000);
  }
}

/* A FreeRTOS task to periodically compile metob reports */
void NyanModule::report_sender_task(void *params) {
  NyanModule *nm = (NyanModule *) params;
  LOG_DEBUG("NYAN reporter stack high water mark: %u\n",
            uxTaskGetStackHighWaterMark(NULL));

  while(true) {
    delay(met_reporting_period);
    LOG_DEBUG("NYAN sending report\n");
    nm->send_report();
  }
}

void debug_memory(void) {
#ifdef ESP32
  LOG_DEBUG("ESP.getHeapSize(): %u\n", ESP.getHeapSize());
  LOG_DEBUG("ESP.getFreeHeap(): %u\n", ESP.getFreeHeap());

  LOG_DEBUG("Free heap: %u\n", esp_get_free_heap_size());
  LOG_DEBUG("Lowest free heap seen: %u\n", esp_get_minimum_free_heap_size());
#endif
}

NyanModule::NyanModule() : ProtobufModule("nyan", meshtastic_PortNum_NYAN, &nyan_telemetry_msg),
                           concurrency::OSThread("NyanModule") {
  LOG_INFO("Starting Nyan Module\n");
  LOG_DEBUG("WiFi enabled?: %u\n", config.network.wifi_enabled);

  TaskHandle_t sensor_task_handle;
  TaskHandle_t reporter_task_handle;

  debug_memory();

  INA3221_setup();
  AS3935_setup();

#ifdef USE_N2K
  nyan_N2K_setup();
#endif

#ifdef USE_NMEA_SERIAL
  NMEA_serial_setup();
#endif

  auto create_return_val =
    xTaskCreate(NyanModule::sensor_sampler_task,
                "NYAN sensor sampler",
                // Stack size, in bytes, not words,
                // contrary to rtos docs, because espressif...
                10000,
                NULL, // task paramaters
                5, // priority
                &sensor_task_handle);
  if( create_return_val == pdPASS ) {
    LOG_DEBUG("NYAN sensor sampler task created\n");
  } else {
    LOG_ERROR("NYAN sensor sampler task create FAILED\n");
    // Probably not enough memory
  }
  LOG_DEBUG("After create task\n");
  debug_memory();

  create_return_val =
    xTaskCreate(NyanModule::report_sender_task,
                "NYAN report sender",
                // Stack size, in bytes, not words,
                // contrary to rtos docs, because espressif...
                // TODO ifdef to change for other platforms
                5000,
                this, // task paramaters
                5, // priority
                &reporter_task_handle);
  if (create_return_val == pdPASS) {
    LOG_DEBUG("NYAN reporter task created\n");
  } else {
    LOG_ERROR("NYAN reporter task create FAILED\n");
    // Probably not enough memory
  }
  LOG_DEBUG("After create task\n");
  debug_memory();
}
