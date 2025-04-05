#pragma once
#include "ProtobufModule.h"
#include "../mesh/generated/meshtastic/nyan.pb.h"

class NyanModule : public ProtobufModule<nyan_telemetry>,
                   private concurrency::OSThread {
  public:
  NyanModule();

  static void report_sender_task(void *params);
  static void sensor_sampler_task(void *params);

  protected:

  virtual int32_t runOnce() override;
  virtual bool handleReceivedProtobuf(const meshtastic_MeshPacket &mp,
                                      nyan_telemetry *telemetry) override;
  void NyanTelemetryToSignalK(const meshtastic_MeshPacket &mp,
                              nyan_telemetry *telemetry);
  void printNyanProtobuf(const meshtastic_MeshPacket &mp,
                         nyan_telemetry *telemetry);
  static void sample_onboard_sensors(void);

  void send_report();
};
