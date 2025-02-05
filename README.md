# Nyan - The Nearby Yacht Area Network

This repo contains a modification of Meshtastic designed for use on yachts,
boats, and shore stations. It forms a mesh network with LoRa radios which can
be used to exchange text messages and boat telemetry, particularly weather
observations.

More information about the Nyan project on the [wiki](https://mews.river.cat/start).

## Modifications made on Meshtastic:

### Added network interfaces:

Data input from:
- NMEA2000 / CAN bus
- NMEA0183 (UART / serial lines)
- NMEA0183 over TCP

Data out to:
- SignalK over TCP

### Weather Reports

The software collates data from connected NMEA networks and compiles
meteorological reports which are sent over LoRa every 10 minutes.

### Sensors

A sensor class is added, which is used to store values from NMEA networks or
any other source.

Support is added for the AS3935 lightning detector IC.

### Telemetry

The Meshtastic Telemetry module is removed. It conflicted with my use of the
INA3221, and Nyan adds its own sensor framework and message types.

## Status

The code is functional, but of experimental hack quality, aka. "research
grade".

It has been tested connected to a boat's NMEA2000 network.

## TODOs / Caveats

- Configuring network interfaces is hardcoded. I started to add configuration
  parameters to the Meshtastic system, but would also need to modify the
  Meshtastic clients / UI to add these.

- Do wind averaging properly (it is currently an expeditious hack that can give
  wrong results).

- Add output over NMEA2000. Possibly by making fake AIS targets for what are
  Nyan sources. Possibly sending protobufs over a custom PGN.

- Lightning sensor needs reporting added.

- SignalK requires you to disable its "security" feature to accept data over
  TCP. I would recommend you disable this anyway, for reliability reasons, and
  secure access to your boat's network on a more appropriate level.

## Hardware

I have developed this software on Heltec ESP32 boards and Raspberry Pi Picos,
and there are pin configurations for some of these boards added to the
repo. The Pico is a good choice if you want an easy way to connect to
NMEA2000, because you can use Waveshare CAN and LoRa boards with it (these are
a bit expensive though). The Heltecs you need to wire up a CAN interface
manually.
