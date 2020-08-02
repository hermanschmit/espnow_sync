# ESPNOW_SYNC

This repository includes a library and an example of code to synchronize multiple ESP32 devices. It is intended for use with the esp_idf and PlatformIO.

Currently tested on Sparkfun ESP32 Thing boards.

## Requirements

## Operation

Like Ethernet 1588.

TIMER0 only
Conventional WiFi must be turned off.

## Usage

## Example

The example application espnow_sync_main.c sets up a blinking LED based on TIMER0. The synchronization of TIMER0 will result in the visible synchronization of the LEDs.

## Accuracy Measurements

TODO

### Possible causes of Accuracy Loss

TODO

## TODOs

Add a scheme to check that the synchronization completed successfully.

