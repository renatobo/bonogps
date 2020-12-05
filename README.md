# BonoGPS

BonoGPS is a firmware for ESP32 devices that reads NMEA sentences from a ublox GPS device (for example commonly available and cheap [BN220](https://www.amazon.com/Navigation-Raspberry-Betaflight-Geekstory-Shipping/dp/B07PRDY6DS) and [BN880](https://www.amazon.com/Geekstory-Navigation-Raspberry-Aircraft-Controller/dp/B078Y6323W)) and repeats them back to either

1. a TCP/IP socket
2. a Bluetooth Low Energy (BLE) service
3. a BT Classic (BT-SPP) stream


The main goal is making GPS data available to mobile apps that can record data, for example for track riding. Currently these apps are supported

1. Harry's Lap Timer
2. TrackAddict
3. RaceChrono

A web configuration (responsive HTML) panel allows changing select GPS parameters on the fly: access it via [http://bonogps.local]() (when in WiFi AP mode, this becomes [http://10.0.0.1]() on Android without mDNS resolution).

The settings required by each app are available as presets that you can load for your device (iOS, Android). More information on what version, features, and connections of each app are available below.

You can also interface your GPS to [uBlox u-center](https://www.u-blox.com/en/product/u-center) via TCP/IP, in this case be aware that the uart speed is fixed.

## Usage

Most runtime configurations are managed via the web-based interface

- Load presets for the supported/tested apps
- Change GPS messages, rate, etc.
- Enable BLE/BT-SPP connectivity
- Switch between WiFi client to a well-known WiFi network or built-in Access Point
- Collect information about current status of the device
- Save current configuration
- Put GPS in powersaving for some time
- Restart the ESP32

The BOOT button also allows you to switch between WiFi modes on the fly.

### Harry's LapTimer

More info at https://www.gps-laptimer.de/, where you can also find a very good user and developer forum. It offers the largest set of customizable options to build your own external device and it is the only supported platform for iOS phones via Bluetooth Low Energy

Features:

  - NMEA parsing offered by HLT directly. Enable or Disable GSV/GSA as needed
  - WiFi: TCP at port 8118
  - tested with v24
  - BLE and WiFi supported and tested on iPhone (tested with 7). BLE handles 5Hz and 10Hz  when GSV/GSA disabled
  - BT-SPP and WiFi supported and tested on Android

##### iOS Bluetooth Low Energy setup

Open your advanced configuration settings in the HTL app and enable BLE device for GPS. Settings are displayed in the "information" page in the web configuration panel

- the device name is a combination of "BonoGPS" and the last 4 chars of the EPS32 MAC: you can also look it up in the web interface as the main page title
- the service uuid is **1819** (standard for SERVICE_LOCATION_AND_NAVIGATION_UUID)
- the characteristic uuid is **2A67** (standard for CHARACTERISTIC_LOCATION_AND_SPEED_CHARACTERISTIC_UUID)

### TrackAddict

Another very popular app, [https://www.hptuners.com/product/trackaddict-app/]()

  - tested with v4.6.0 on Android
  - BT-SPP is the only option
  - it needs a specific set of NMEA messages configuration, available as option
  - NBP was coded, yet not going to matter for TrackAddict as reported in https://forum.hptuners.com/showthread.php?78403-GPS-over-NBP&highlight=gps
  - Notes for the Classic BT-SPP version to optimize transfer to TrackAddict on Android
    - Ideally, TrackAddict wants RMC, GGA, and GLL messages, with RMC and GGA being the recommended minimum.
    - Track Addict is similarly configured to only accept NMEA messages with a GPS talker ID (i.e., $GPRMC instead of $GNRMC) https://forum.hptuners.com/showthread.php?69123-Track-addict-external-GPS&highlight=gps

### RaceChrono

The cleanset lap timer supported: [https://racechrono.com/]()

  - tested with v7.0.10 free (thus satellites view untested) on Android
  - BT-SPP is the only option
  - GSA+GSV polling disabled as it's not used

## Hardware build instructions

You only need to connect your ublox GPS module to a Hardware serial port such as UART2/Serial2 and the GPS power to the Vin and Ground pins of ESP32.

### GPS Preconfiguration

To reduce complexity of this software, you need to save a baseline configuration of your ublox GPS module. Parameters include

- default port speed (it must match GPS_STANDARD_BAUD_RATE)
- what satellite systems to use (for US, GPS/Glonass/Galileo)
- what Wide Area Augmentation System to be used
- disable all unnecessary NMEA messages
- set motion defaults

... etc.


## Software build instructions

This code is developed specifically for ESP32, and tested with [PlatformIO](https://platformio.org/) (main development platform) and the [Arduino IDE (1.8.13)](https://www.arduino.cc/en/software)

### Libraries
  - [Nimble-Arduino](https://github.com/h2zero/NimBLE-Arduino) 
  - [Uptime Library](https://github.com/YiannisBourkelis/Uptime-Library) 
  - [EasyButton](https://easybtn.earias.me/) 

### Optional libraries depending on #define options
  - [Task Scheduler](https://github.com/arkhipenko/TaskScheduler)  [included by default]
  - [NeoGPS](https://github.com/SlashDevin/NeoGPS)  [not included right now, but coded and available for some additional cases]

### Built-in libraries used by this code
  - WebServer
  - FS
  - Preferences
  - WiFi
  - DNSServer
  - ESPmDNS
  - ArduinoOTA
  - Update
  - BluetoothSerial 

### Partition size
  - you have to select a partitioning schema with 1.7 Mb of programming space (e.g. Minimal SPIFF with 1.9Mb), as the app with its libraries tend to be pretty large due to BT stacks

```
esp32.menu.PartitionScheme.min_spiffs=Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)
esp32.menu.PartitionScheme.min_spiffs.build.partitions=min_spiffs
esp32.menu.PartitionScheme.min_spiffs.upload.maximum_size=1966080
```

## Possible enhancements and ideas
   
  - add display: https://randomnerdtutorials.com/esp32-ssd1306-oled-display-arduino-ide/
  - add battery and expose battery level via BLE and/or web interface
  - log to SD  https://randomnerdtutorials.com/esp32-data-logging-temperature-to-microsd-card/
  - test https://apps.apple.com/us/app/espressif-esptouch/id1071176700 or https://apps.apple.com/in/app/esp-ble-provisioning/id1473590141
  - OTA via browser or from internet location https://lastminuteengineers.com/esp32-ota-web-updater-arduino-ide/ https://github.com/chrisjoyce911/esp32FOTA or https://github.com/platformio/bintray-secure-ota
  - use ootb library at https://os.mbed.com/teams/ublox/code/gnss/ or https://github.com/ldab/u-blox_GNSS
  - use packed binary format with custom binary parser for HLT
  - use https with https://github.com/fhessel/esp32_https_server and https://github.com/fhessel/esp32_https_server_compat
  - restore usage of https://github.com/khoih-prog/ESPAsync_WiFiManager and Async webserver (issues with ASyncTCP not being able to send info back outside of request/reply

