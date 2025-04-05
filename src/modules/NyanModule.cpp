#include <string.h>
#include <assert.h>

#include <WiFi.h>
#include "os_status.h"
#include "mesh/NodeDB.h"

#include "gps/RTC.h"

#include "configuration.h"
#include "main.h"
#include "MeshService.h"

#include "NodeInfoModule.h"

#include "serialization/JSON.h"
#include "serialization/JSONValue.h"

#include "INA3221.h"

#ifdef USE_AS3935
#include "Lightning.h"
#endif

#include "NyanModule.h"
#include "NyanNMEA.h"

#ifdef USE_N2K
#include "NyanN2K.h"
#endif

// For unit conversions
#include "N2kMessages.h"

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
uint32_t met_reporting_period = 600000;

class ReportSender : concurrency::OSThread {
public:
  ReportSender(NyanModule *nm) : concurrency::OSThread("Nyan Report Sender"),
                                 nm(nm) {}
  int32_t runOnce() override;

private:
  NyanModule *nm;
};

int32_t ReportSender::runOnce() {
  NyanModule::report_sender_task(nm);
  return met_reporting_period;
}

void debug_memory(void) {
  LOG_DEBUG("Heap size: %u", memGet.getHeapSize());
  LOG_DEBUG("Free heap: %u", memGet.getFreeHeap());

#ifdef ARCH_ESP32
  LOG_DEBUG("Lowest free heap seen: %u", esp_get_minimum_free_heap_size());
#endif
}

/* Import NMEA0183 data from a tcp server.
   If not connected, connect.
   Read and parse sentences.
   Add data to ship data model.
*/
void NMEA_TCP_read() {
  tNMEA0183Msg NMEA0183Msg;
  tNMEA0183 NMEA0183;
  static WiFiClient tcp;

  const uint8_t NMEA_BUFFER_LENGTH = 85;
  // Including terminating charaters (CR LF)
  const uint8_t NMEA_MIN_SENTENCE_LENGTH = 11;

  static char nmea_buffer[NMEA_BUFFER_LENGTH];
  static uint8_t nmea_index = 0;

  // TODO: Move to config.
  const char *nmea_tcp_host = "shore.halekai.uk";
  const uint16_t nmea_tcp_port = 10110;

  if (!WiFi.isConnected()) {
    LOG_INFO("WiFi not connected");
    return;
  }

  if (! tcp.connected()) {
    if (tcp.connect(nmea_tcp_host, nmea_tcp_port)) {
      LOG_INFO("TCP connected");
    } else {
      LOG_WARN("Connecting TCP NMEA failed.");
      return;
    }
  }

  const uint32_t time_limit = 500; // mS
  uint32_t end_time = millis() + time_limit;

  while (tcp.available()) {
    if (millis() > end_time) {
      LOG_ERROR("NMEA TCP parsing taking too long.");
      return;
    }

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
        parse_sentence(nmea_buffer);
        goto done;
      }
      goto err;
    }

    // Move on and loop for next char
    nmea_index++;
    continue;

  err:
    LOG_ERROR("NMEA parse error");
  done:
    nmea_index = 0;
  }
}

bool get_position_fixed(NyanVessel& v) {
  // Consult meshtastic's manually set position system, and copy to nyan
  // vessel. Assume we are pointing North.

  if (config.position.fixed_position) {
    v.position_fixed.latitude = localPosition.latitude_i  * 1e-7;
    v.position_fixed.longitude = localPosition.longitude_i * 1e-7;
    v.position_fixed.COG = 0;
    v.position_fixed.SOG = 0;
    v.position_fixed.set_valid();

    // Assume any sensors are pointing north
    v.HDT.set(0);

    return true;
  }

  return false;
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
      ((localPosition.timestamp + validity_period)
       > getValidTime(RTCQualityFromNet))
      && (localPosition.fix_quality > 0)) {

    v.position_gnss_builtin.set_valid();

    v.position_gnss_builtin.COG = localPosition.ground_track * 100;
    v.position_gnss_builtin.SOG = localPosition.ground_speed * MPS_TO_KNOTS;

    v.position_gnss_builtin.latitude  = localPosition.latitude_i  * 1e-7;
    v.position_gnss_builtin.longitude = localPosition.longitude_i * 1e-7;

    LOG_DEBUG("Got fix from builtin GNSS. SOG %f COG %f",
              v.position_gnss_builtin.SOG, v.position_gnss_builtin.COG);
    return true;
  }
  return false;
}

void sample_NMEA_sensors(NyanVessel& v) {
  v.HDT.sample();
  v.AWA.sample();
  v.AWS.sample();
  LOG_DEBUG("Sensors after sampling: HDT: %f AWA (av): %f (%f) AWS (av) : %f %f",
            v.HDT.get(), v.AWA.raw(), v.AWA.get(),
            v.AWS.raw(), v.AWS.get());

  /* Ground wind */
  double GWS = 0;
  double GWD = 0;
  if (Wind::derive_ground_wind(v, GWS, GWD)) {
    LOG_INFO("Derived ground wind: %f kn, %f T", GWS, GWD);
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
  // Generate with: signalk-generate-token -u nyan -e 10y -s
  // /home/signalk/.signalk/security.json
  // Note: Not used in the end, because it turns out SignalK doesn't even
  // implement this over TCP, despite it being specified in the docs!
  //  String token = "...";

  if (!WiFi.isConnected()) {
    LOG_INFO("WiFi not connected");
    return;
  }

  if (! tcp.connected()) {
    if (tcp.connect(host, port)) {
      LOG_INFO("SignalK TCP connected");
    } else {
      LOG_WARN("Connecting SignalK TCP failed.");
      return;
    }
  }

  String s =
    R"({"context": "vessels.urn:mrn:nyan:)" + String(node_id) + R"(",)"

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

    LOG_DEBUG("pos.latitude is %s by arduino, %f by printf", String(pos.latitude), pos.latitude);
  }

  LOG_DEBUG("v.GWS.stats.quality(): %f v.GWS.stats.mean(): %f",
            v.GWS.stats.quality(), v.GWS.stats.mean());

  if (v.GWS.stats.quality() > 0.1 && v.GWD.stats.quality() > 0.1) {
    LOG_DEBUG("SignalK: sending ground wind");

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

  if (v.water_depth_below_keel.valid()) {
    s +=
      R"({"path": "environment.depth.belowKeel", "value": )" + String(v.water_depth.get()) + "}"
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

/*
 * Connect to SignalK by tcp and send the provided JSON string.
 */
void signalk_send(const char *json) {
  static WiFiClient tcp;

  const char *host = "nyan-host-0.river.cat";
  const uint16_t port = 8375;

  if (!WiFi.isConnected()) {
    LOG_WARN("WiFi not connected for SignalK send.");
    return;
  }

  if (! tcp.connected()) {
    if (tcp.connect(host, port)) {
      LOG_INFO("SignalK TCP connected.");
    } else {
      LOG_WARN("Connecting SignalK TCP failed.");
      return;
    }
  }

  LOG_INFO("Sending JSON to SignalK.");
  tcp.print(json);
}

void NyanModule::send_report() {
  /* A struct generated by protobuf definition. */
  nyan_telemetry telemetry = {};

  // We have data to send
  bool send = false;

  // TODO May want to replace with average position, to correspond with
  // average wind, etc.
  Position p;
  if (v.getPosition(&p)) {
    send = true;

    telemetry.latitude = p.latitude;
    telemetry.has_latitude = true;

    telemetry.longitude = p.longitude;
    telemetry.has_longitude = true;
  }

  if (v.GWS.stats.quality() > 0) {
    LOG_DEBUG("GWS quality: %f", v.GWS.stats.quality());

    send = true;
    telemetry.GWS_mean = (uint8_t) round(v.GWS.stats.mean());
    telemetry.has_GWS_mean = true;

    telemetry.GWS_gust = (uint8_t) round(v.GWS.stats.max());
    telemetry.has_GWS_gust = true;

    // FIXME: separate or check quality too
    telemetry.GWD_mean = (uint16_t) round(v.GWD.stats.mean());
    telemetry.has_GWD_mean = true;

    LOG_DEBUG("Sending Ground Wind %u kts, %u T Gust: %u kts",
              telemetry.GWS_mean, telemetry.GWD_mean, telemetry.GWS_gust);
  }

  // Clear statistics collection, ready for the next met reporting period.
  v.GWS.stats.reset();
  v.GWD.stats.reset();

  if (v.water_temperature.valid()) {
    send = true;
    telemetry.water_temperature = v.water_temperature.get();
    telemetry.has_water_temperature = true;
    LOG_DEBUG("Water temperature %f°C", telemetry.water_temperature);
  }

  if (v.water_depth.valid()) {
    send = true;
    telemetry.water_depth = v.water_depth.get();
    telemetry.has_water_depth = true;
    LOG_DEBUG("Sending water depth %fm", telemetry.water_depth);
  }

  if (v.water_depth_below_keel.valid()) {
    send = true;
    telemetry.water_depth_below_keel = v.water_depth_below_keel.get();
    telemetry.has_water_depth_below_keel = true;
    LOG_DEBUG("Sending water depth below keel%fm",
              telemetry.water_depth_below_keel);
  }

  if (v.nyan_supply_voltage.valid()) {
    send = true;
    telemetry.nyan_supply_decivolts = round(v.nyan_supply_voltage.get() * 10.0);
    telemetry.has_nyan_supply_decivolts = true;
    LOG_DEBUG("Sending supply voltage: %u dV",
              telemetry.nyan_supply_decivolts);
  }

  if (send) {
    // So people have our name
    nodeInfoModule->sendOurNodeInfo();

    LOG_INFO("Sending Nyan telemetry");
    meshtastic_MeshPacket *p = allocDataProtobuf(telemetry);
    p->decoded.want_response = false;
    p->priority = meshtastic_MeshPacket_Priority_RELIABLE;
    service->sendToMesh(p);

    NyanModule::NyanTelemetryToSignalK(*p, &telemetry);
  } else {
    LOG_INFO("Not sending telemetry - nothing to report");
  }
}

void NyanModule::printNyanProtobuf(const meshtastic_MeshPacket &mp,
                               nyan_telemetry *telemetry) {

  LOG_INFO("Print Nyan Protobuf");

  meshtastic_NodeInfoLite *node = nodeDB->getMeshNode(mp.from);
  if (node->has_user) {
    LOG_DEBUG("Long name:%s", node->user.long_name);
  }

  if (telemetry->has_latitude && telemetry->has_longitude) {
    LOG_INFO("Position: %f %f",
             telemetry->latitude,
             telemetry->longitude);
  }

  if (telemetry->has_GWS_mean) {
    LOG_INFO("GWS_mean: %u", telemetry->GWS_mean);
  }

  if (telemetry->has_GWS_gust) {
    LOG_INFO("GWS_gust: %u", telemetry->GWS_gust);
  }

  if (telemetry->has_GWD_mean) {
    LOG_INFO("GWD_mean: %u", telemetry->GWD_mean);
  }

  if (telemetry->has_water_temperature) {
    LOG_INFO("water_temperature: %fC", telemetry->water_temperature);
  }

  if (telemetry->has_nyan_supply_decivolts) {
    LOG_INFO("nyan_supply_decivolts: %udV",
             telemetry->nyan_supply_decivolts);
  }
}

void NyanModule::NyanTelemetryToSignalK(const meshtastic_MeshPacket &mp,
                                        nyan_telemetry *telemetry) {

#ifndef ARCH_ESP32
  LOG_INFO("Skipping SignalK send because not on ESP32.");
  // I think the pico is having memory issues with this.
  return;
#endif

  LOG_INFO("NyanTelemetryToSignalK()");
  debug_memory();

  // It looks like the JSON library should delete all the new objects here,
  // but I am a bit suspcious.

  JSONArray values; // SignalK values object

  meshtastic_NodeInfoLite *node = nodeDB->getMeshNode(mp.from);
  if (node->has_user) {
    LOG_DEBUG("SignalK: setting station name from NodeDB; name: %s",
              node->user.long_name);

    JSONObject station_name;
    JSONValue path {"name"};
    JSONValue value {node->user.long_name};
    station_name["path"] = &path;
    station_name["value"] = &value;
    JSONValue snv (station_name);
    values.push_back(&snv);
  }

  if (telemetry->has_latitude && telemetry->has_longitude) {
    LOG_INFO("telemetry Position: %f %f",
             telemetry->latitude,
             telemetry->longitude);

    JSONObject position;
    position["latitude"]  = new JSONValue(telemetry->latitude);
    position["longitude"] = new JSONValue(telemetry->longitude);

    JSONObject pos_value;
    pos_value["path"] = new JSONValue("navigation.position");
    pos_value["value"] = new JSONValue(position);

    values.push_back(new JSONValue(pos_value));
  }

  if (telemetry->has_GWS_mean) {
    LOG_INFO("telemetry GWS_mean: %u", telemetry->GWS_mean);

    JSONObject GWS_value;
    GWS_value["path"] = new JSONValue("environment.wind.speedOverGround");
    GWS_value["value"] = new JSONValue(KnotsToms(telemetry->GWS_mean));
    values.push_back(new JSONValue(GWS_value));

    // Also send speedTrue. This is wrong, but SignalK is generally wrong
    // about a lot of stuff so I may need to be wrong too.
    JSONObject GWS_value2;
    GWS_value2["path"] = new JSONValue("environment.wind.speedTrue");
    GWS_value2["value"] = new JSONValue(KnotsToms(telemetry->GWS_mean));
    values.push_back(new JSONValue(GWS_value2));
  }

  if (telemetry->has_GWD_mean) {
    LOG_INFO("telemetry GWD_mean: %u", telemetry->GWD_mean);

    JSONObject GWD_value;
    // SignalK seems to lack a ground wind direction key, even though it has
    // ground wind speed!!!
    GWD_value["path"] = new JSONValue("environment.wind.directionTrue");
    GWD_value["value"] = new JSONValue(DegToRad(telemetry->GWD_mean));
    values.push_back(new JSONValue(GWD_value));

    JSONObject GWD_value2;
    // I'll just make this key up.
    GWD_value["path"] = new JSONValue("environment.wind.directionOverGround");
    GWD_value["value"] = new JSONValue(DegToRad(telemetry->GWD_mean));

    values.push_back(new JSONValue(GWD_value));
  }

  if (telemetry->has_water_temperature) {
    LOG_INFO("telemetry water_temperature: %fC", telemetry->water_temperature);

    JSONObject water_temperature_value;
    water_temperature_value["path"]  =
      new JSONValue("environment.water.temperature");
    water_temperature_value["value"] =
      new JSONValue(CToKelvin(telemetry->water_temperature));

    values.push_back(new JSONValue(water_temperature_value));
  }

  if (telemetry->has_nyan_supply_decivolts) {
    LOG_INFO("telemetry nyan_supply_decivolts: %udV",
             telemetry->nyan_supply_decivolts);

    JSONObject nyan_supply_voltage_value;

    // This is not correct - nyan supply is not a battery. I think it's the
    // closest SignalK has.
    nyan_supply_voltage_value["path"]  =
      new JSONValue("electrical.batteries.nyan.voltage");
    nyan_supply_voltage_value["value"] =
      new JSONValue(telemetry->nyan_supply_decivolts / 10.0);

    values.push_back(new JSONValue(nyan_supply_voltage_value));
  }

  // TODO: Water depths

  String urn = "vessels.urn:mrn:nyan:" + String(mp.from);
  // I wondered if base stations would show better as atons, but that doesn't
  //  show the wind, so not really.
  // String urn = "atons.urn:mrn:nyan:" + String(mp.from);

  JSONObject SignalK;
  SignalK["context"] = new JSONValue(urn.c_str());

  LOG_DEBUG("After context");
  debug_memory();

  JSONObject source;
  source["label"] = new JSONValue("nyan");
  source["type"] = new JSONValue("nyan");

  JSONObject update;
  update["source"] = new JSONValue(source);
  update["values"] = new JSONValue(values);

  JSONArray updates;
  updates.push_back(new JSONValue(update));
  SignalK["updates"] = new JSONValue(updates);

  JSONValue JSONvalue(SignalK);
  string json_s = JSONvalue.Stringify() + "\n";

  LOG_DEBUG("Memory after json creating, before delete.");
  debug_memory();

  delete JSONvalue;

  LOG_DEBUG("JSON string built from Nyan protobuf: ");
  LOG_DEBUG(json_s.c_str());

  signalk_send(json_s.c_str());
}

bool NyanModule::handleReceivedProtobuf(const meshtastic_MeshPacket &mp,
                                        nyan_telemetry *telemetry) {
  if (telemetry == NULL) {
    LOG_WARN("handleReceivedProtobuf() got null protobuf decode");
    return false;
  }

  screen->print("Nyan RXed");
  LOG_INFO("Received Nyan telemetry from node 0x%0x (Packet ID: 0x%x)",
           mp.from, mp.id);

  printNyanProtobuf(mp, telemetry);

  NyanModule::NyanTelemetryToSignalK(mp, telemetry);

  // Pass to other modules too. Maybe needed for the router to rebroadcast it?
  return false;
}

/* Can run every time the thread is scheduled.
   Delayed to meet desired period, which is the return value.
*/
int32_t NyanModule::runOnce() {

#ifdef USE_NMEA_TCP
  NMEA_TCP_read();
#endif

#ifdef USE_N2K
  nyan_N2K_loop();
#endif

#ifdef USE_NMEA_SERIAL
  NMEA_serial_loop();
#endif

  get_position_fixed(v);
  get_local_GPS(v);

  sample_onboard_sensors();

#ifdef USE_AS3935
  AS3935_check_lightning();
#endif

  return 3000; // period in milliseconds
}

#ifdef USE_INA3221
INA3221 ina3221 = INA3221((ina3221_addr_t) INA3221_ADDR);
#endif

bool INA3221_setup_ok = false;

void INA3221_setup(void) {
#ifdef USE_INA3221
  LOG_INFO("INA3221_setup");

  ina3221.begin(&INA3221_BUS);

  // TODO Configuration
  ina3221.setShuntRes(50, 100, 100); // In milliOhms
  ina3221.setFilterRes(10, 10, 10); // In Ohms

  delay(10);
  ina3221.reset();
  delay(10);

  if (ina3221.getManufID() == 0x5449) {
    LOG_DEBUG("Read INA3221 Maufacturer ID OK.");
    INA3221_setup_ok = true;
  } else {
    LOG_ERROR("Read INA3221 Maufacturer ID Failed.");
  }

  delay(10);
#endif
}

void read_INA3221() {
#ifdef USE_INA3221
  if (! INA3221_setup_ok) return;

  float V_in = ina3221.getVoltage(INA3221_CH1);
  float I_in = ina3221.getCurrentCompensated(INA3221_CH1) / 1000.0;
  float V_5V = ina3221.getVoltage(INA3221_CH2);

  v.nyan_supply_voltage.set(V_in);
  v.nyan_supply_voltage.sample();

  LOG_DEBUG("INA3221 V_in: %.3fV I_in: %.3fA V_5V: %.3fV", V_in, I_in, V_5V);
#endif
}

void NyanModule::sample_onboard_sensors(void) {
  read_INA3221();
}

void set_test_sensor_data(void) {
  // Add fake data
  v.position_nmea.latitude = 56.020;
  v.position_nmea.longitude = -3.197;
  v.position_nmea.COG = 88;
  v.position_nmea.SOG = 5;
  v.position_nmea.set_valid();

  v.AWS.set(15);
  v.AWA.set(160);
  v.HDT.set(90);
  v.water_temperature.set(15);
}

/* A FreeRTOS task to read / filter / store sensor data. */
void NyanModule::sensor_sampler_task(void *params) {
  while(true) {
    LOG_DEBUG("NYAN Sampling sensors");
    LOG_DEBUG("NYAN sampler stack high water mark: %u",
              uxTaskGetStackHighWaterMark(NULL));

    // Inject mock data if in test mode
    if (config.nyan.test_send) {
      if (config.nyan.test_reporting_period < 10000) {
        config.nyan.test_reporting_period = 10000;
      }
      met_reporting_period = config.nyan.test_reporting_period;

      set_test_sensor_data();
    }

    sample_NMEA_sensors(v);
    sample_onboard_sensors();
    delay(5000);
  }
}

/* A FreeRTOS task to periodically send metob reports */
void NyanModule::report_sender_task(void *params) {
  NyanModule *nm = (NyanModule *) params;
  while(true) {
    LOG_DEBUG("NYAN reporter stack high water mark: %u",
              uxTaskGetStackHighWaterMark(NULL));

    // Don't delay if running on main loop task
    //    delay(met_reporting_period);

    nm->send_report();

    // Also print some useful info every so often.
    LOG_INFO("Uptime %us", millis() / 1000);
    LOG_INFO("TX time last hour: %f%%", airTime->utilizationTXPercent());
  }
}

NyanModule::NyanModule() : ProtobufModule("nyan",
                                          meshtastic_PortNum_NYAN,
                                          &nyan_telemetry_msg),
                           concurrency::OSThread("NyanModule") {
  LOG_INFO("Starting Nyan Module");
  LOG_INFO("WiFi enabled?: %u", config.network.wifi_enabled);

  TaskHandle_t sensor_task_handle;
  TaskHandle_t reporter_task_handle;

  debug_memory();

#ifdef USE_INA3221
  INA3221_setup();
#endif

#ifdef USE_AS3935
  AS3935_setup();
#endif

#ifdef USE_N2K
  nyan_N2K_setup();
#endif

#ifdef USE_NMEA_SERIAL
  NMEA_serial_setup();
#endif

#ifdef USE_NYAN_TEST_DATA
  // TODO: build the cli with ability to set this
  config.nyan.test_send = true;
  config.nyan.test_reporting_period = 600000;
#endif

  // In bytes
  size_t stack_size = 2200;
#ifndef ARCH_ESP32
  // FreeRTOS standard is words, but espressif made it bytes.
  stack_size /= 4;
#endif

  auto create_return_val =
    xTaskCreate(NyanModule::sensor_sampler_task,
                "NYAN sensor sampler",
                // contrary to rtos docs, because espressif...
                stack_size,
                NULL, // task paramaters
                5, // priority
                &sensor_task_handle);
  if( create_return_val == pdPASS ) {
    LOG_DEBUG("NYAN sensor sampler task created");
  } else {
    LOG_ERROR("NYAN sensor sampler task create FAILED");
    // Probably not enough memory
  }
  LOG_DEBUG("After create task");
  debug_memory();

  // In bytes
  stack_size = 8000;

#ifndef ARCH_ESP32
  // FreeRTOS standard is words, but espressif made it bytes.
  stack_size /= 4;
#endif

  /*
  create_return_val =
    xTaskCreate(NyanModule::report_sender_task,
                "NYAN report sender",
                // Stack size, in bytes, not words,
                // contrary to rtos docs, because espressif...
                // TODO ifdef to change for other platforms
                stack_size,
                this, // task paramaters
                5, // priority
                &reporter_task_handle);
  if (create_return_val == pdPASS) {
    LOG_DEBUG("NYAN reporter task created");
  } else {
    LOG_ERROR("NYAN reporter task create FAILED");
    // Probably not enough memory
  }

  */

  /* Try using meshtastic scheduler instead of FreeRTOS task for this.
     It will not have so good timing, but avoids allocating a new stack.
  */
  new ReportSender(this);

  LOG_DEBUG("After create task");
  debug_memory();
}

