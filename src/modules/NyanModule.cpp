#include "NyanModule.h"
#include "MeshService.h"
#include "configuration.h"
#include "main.h"

#include <assert.h>

/* Connect by tcp to an NMEA0183 data source. */

/* If not connected, connect.
   Read sentence.
   NMEA Parse.
   Add data to ship data model.
*/

/* Periodically send ship data over mesh. */

/* Can run every time the thread is scheduled.
   Delayed to meet desired period, which is the return value.
*/

int32_t NyanModule::runOnce()
{
  LOG_INFO("Meow\n");
  screen->print("Meow ");

  return 1000; // period
}

meshtastic_MeshPacket *NyanModule::allocReply()
{
    assert(currentRequest); // should always be !NULL
#ifdef DEBUG_PORT
    auto req = *currentRequest;
    auto &p = req.decoded;
    // The incoming message is in p.payload
    LOG_INFO("Nyan received message from=0x%0x, id=%d, msg=%.*s\n",
             req.from, req.id, p.payload.size, p.payload.bytes);
#endif

    screen->print("Nyan replying\n");

    const char *replyStr = "Nyan module received message";
    auto reply = allocDataPacket();                 // Allocate a packet for sending
    reply->decoded.payload.size = strlen(replyStr); // You must specify how many bytes are in the reply
    memcpy(reply->decoded.payload.bytes, replyStr, reply->decoded.payload.size);

    return reply;
}

void NyanModule::send_report() {
  meshtastic_MeshPacket *p = allocReply();
  //  p->to = dest;

  service.sendToMesh(p);
}
