#pragma once
#include "ProtobufModule.h"
#include "../mesh/generated/meshtastic/nyan.pb.h"

/**
 */
class NyanModule : public ProtobufModule<nyan_telemetry>,
                   private concurrency::OSThread {
  public:
  /* Constructor name is for debugging output */
  NyanModule() :
    ProtobufModule("nyan", meshtastic_PortNum_NYAN, &nyan_telemetry_msg),
    concurrency::OSThread("NyanModule") {}

  protected:

  virtual int32_t runOnce() override;
  virtual bool handleReceivedProtobuf(const meshtastic_MeshPacket &mp, nyan_telemetry *telemetry) override;

  void send_report();
};
