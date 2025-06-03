# SR-JV80 Wi-Fi Expansion Card ESP32-S3 Firmware

This repo holds the firmware source file for the SR-JV80 Wi-Fi Expansion Card. It uses Arduino IDE with the standard ESP32 package and some other dependencies. You can download ready-to-flash binary files from release section.

## Required Dependencies

### Required Board Package
- esp32

### Required Library
- ESP Async Web Server
- Async TCP

## How to Flash

The card have a production header which is unpopulated if you buy one. For development you can solder a pin header (facing top side to avoid interference).

The card itself does not include the standard ESP32 auto-flash circuitry so you need to manually put the card in bootloader mode:

1. Short "BOOT" pin to ground
2. Power the board up, or if the board is already powered up short reset ("R") pin to ground then release
3. Release "BOOT" pin
4. Flash using esptool or your prefered flashing method

## Changelog

### 1.11

- Add production test function to eliminate need for separate production test firmware 

### 1.10

- Added AP channel selection function, changed default channel to 11
- Added AP hide SSID function
- Added show current value function for configuration dropdowns in HTML

### 1.00

- Fixed an issue where Apple devices are not supported in Bluetooth LE wakeup.
- Fixed an issue in HTML
- Fixed an issue where Bluetooth LE advertisement is sent once on boot, even if wireless mode is set to All Off

### 0.99

- Initial production version
- Bluetooth wakeup only supports Android phone.
