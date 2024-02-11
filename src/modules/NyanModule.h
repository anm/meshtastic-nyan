#pragma once
#include "SinglePortModule.h"

/**
 */
class NyanModule : public SinglePortModule, private concurrency::OSThread
{
  public:
    /** Constructor
     * name is for debugging output
     */
  NyanModule() : SinglePortModule("nyan", meshtastic_PortNum_NYAN), concurrency::OSThread("NyanModule") {}

  protected:
    /** Run as part of want_replies handling ..?
     */
  virtual meshtastic_MeshPacket *allocReply() override;
  virtual int32_t runOnce() override;

  void send_report();
};
