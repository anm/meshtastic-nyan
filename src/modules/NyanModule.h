#pragma once
#include "ProtobufModule.h"
#include "../mesh/generated/meshtastic/nyan.pb.h"

/**
 */
class NyanModule : public ProtobufModule<nyan_telemetry>,
                   private concurrency::OSThread {
  public:
  /* Constructor name is for debugging output */
  NyanModule();

  protected:

  virtual int32_t runOnce() override;
  virtual bool handleReceivedProtobuf(const meshtastic_MeshPacket &mp, nyan_telemetry *telemetry) override;

  void report_compilation_task(void *params);
  static void sensor_sampler_task(void *params);
  void send_report();
};
