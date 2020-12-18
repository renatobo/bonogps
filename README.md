# BonoGPS

BonoGPS is a firmware for ESP32 devices that reads NMEA sentences from a ublox GPS device (for example commonly available and cheap [BN220](https://www.amazon.com/Navigation-Raspberry-Betaflight-Geekstory-Shipping/dp/B07PRDY6DS) and [BN880](https://www.amazon.com/Geekstory-Navigation-Raspberry-Aircraft-Controller/dp/B078Y6323W)) and repeats them back to either

1. a TCP/IP socket
2. a Bluetooth Low Energy (BLE) service
3. a BT Classic (BT-SPP) stream


The main goal is to **make GPS data available to mobile apps that can record data, for example for track riding**. Currently these apps are supported

1. [Harry's Lap Timer](https://www.gps-laptimer.de)
2. [TrackAddict](https://www.hptuners.com/product/trackaddict-app/)
3. [RaceChrono](https://racechrono.com/)
4. [Racetime](https://www.racetimeapp.com/en/)

A web configuration (responsive HTML) panel allows changing select GPS parameters on the fly: access it via [http://bonogps.local]() (when in WiFi AP mode, this becomes [http://10.0.0.1]() on Android without mDNS resolution).

The settings required by each app are available as presets that you can load for your device (iOS, Android). More information on what version, features, and connections of each app are available below.

You can also interface your GPS to [uBlox u-center](https://www.u-blox.com/en/product/u-center) via TCP/IP, in this case be aware that the uart speed is fixed.

## Usage

1. Turn on the ESP32 and wait a few minutes for the GPS to get a fix on position (on BN devices, the red led will blink every second)
2. Open your preferred mobile app and connect 
3. Enjoy your ride!

Most runtime configurations are managed via the web-based interface

![Main page of web configuration panel](software/using/webinterface_root.png)

- **GPS runtime settings:** GPS messages, rate of updates, serial port speed
- **Connections:** List clients, enable BLE/BT-SPP connectivity, enable/disable the builtin TCP-IP messages repeater, switch between WiFi client to a well-known WiFi network or built-in Access Point
- **Device:** Put GPS in powersaving for some time, Disable WiFi, load preset configurations, save current configuration, collect information about current status of the device, save WiFi credentials for client mode restart the ESP32

The BOOT button allows you to loop between WiFi modes on the fly: WiFi Access Point -> WiFi client -> No WiFi.

- WiFi Client (WiFi STA): the blue led light flashes rapidly (250 ms cycle)
- WiFi Access Point (WiFi AP): the blue led light flashes slowly (500 ms cycle)
- No WiFi: the blue led light is off

![Fast blinking - Client](hardware/esp32/wifi_client.webm)
![Slow blinking - AP](hardware/esp32/wifi_ap.webm)

### Saving configuration

If you load a preset or if you change a runtime settings, you can preserve it across restart of the device.

Go to *Device > Save Config* and you'll find these options

![Save Config page](software/using/webinterface_saveconfig.png)

For convenience, the WiFi status can be excluded or saved separately.

## Connecting to an app

There are many mobile apps to log lap times, few accept custom devices, in particular on iOS. The ones below are tested.

| | Harry Lap Timer| TrackAddict | RaceChrono | Racetime
| --- | --- | --- | --- | ---
| iOS | **BLE**, TCP-IP | | |
| Android | **BT-SPP**, TCP-IP | BT-SPP | BT-SPP | BT-SPP 

See more info in each subfolder of **connecting**.

You can load a preset configuration from the configuration page selecting *Device > Load Preset* and then choosing your mobile phone device and app combination (when there are alternatives, the recommended option is in bold)

![Loading presets](software/using/webinterface_loadpreset.png)

## Hardware build instructions

The minimum build is a ublox GPS module connected to an ESP32:

- TX/RX from the GPS to a Hardware serial port such as UART2/Serial2 on ESP32
- the GPS power pins to ESP32: VCC to 3v3, GND to GND

BN220 comes with a 4 pin adapter cable for GND TX RX VCC, while BN880 includes 2 additionals pins for SDA and SCL of the IMU (which is not used)

Schematics are relatively simple

* power the GPS module
* connect GPS RX to ESP32 UART2 TX and GPS TX to ESP32 UART2 RX

![Schematics](hardware/esp32/esp32_to_gps_schem.png)

Examples are in [hardware/assembled](hardware/assembled)

### GPS Preconfiguration

To reduce complexity of this software, you need to save a baseline configuration of your ublox GPS module. Parameters include

- default port speed (it must match GPS_STANDARD_BAUD_RATE)
- what satellite systems to use (for US, GPS/Glonass/Galileo)
- what Wide Area Augmentation System to be used
- disable all unnecessary NMEA messages
- set motion defaults

... etc.

A backup of the options is in the [hardware/GPS folder](hardware/GPS/gps-bn220-config.txt), stored for a BN-220 but it's identical to a BN-880


## Software build instructions

This code is developed specifically for ESP32, and tested with [PlatformIO](https://platformio.org/) (main development platform) and the [Arduino IDE (1.8.13)](https://www.arduino.cc/en/software)

### Libraries

  - [Nimble-Arduino](https://github.com/h2zero/NimBLE-Arduino) 
  - [Uptime Library](https://github.com/YiannisBourkelis/Uptime-Library) 
  - [EasyButton](https://easybtn.earias.me/) 

You can reduce flash size by ~30kb of Nimble-Arduino removing role roles `CONFIG_BT_NIMBLE_ROLE_CENTRAL` and `CONFIG_BT_NIMBLE_ROLE_OBSERVER` in `nimconfig.h` by simply commenting the two defines.

### Optional libraries depending on #define options

  - [Task Scheduler](https://github.com/arkhipenko/TaskScheduler)  [included by default and recommended as most usefule features depend on it]
  - [NeoGPS](https://github.com/SlashDevin/NeoGPS)  [not included right now, but coded and available for some additional cases]

### Built-in libraries used by this code

  - WebServer
  - FS
  - Preferences
  - WiFi
  - DNSServer
  - ESPmDNS
  - Update
  - BluetoothSerial

### Optional builtin libraries disabled by default
  - ArduinoOTA: this is useful during development only, so it's undefined by default in the current code

### Important: Partition size

You have to select a partitioning schema with 1.7 Mb of programming space (e.g. Minimal SPIFF with 1.9Mb), as the app with its libraries tend to be pretty large due to BT stacks.

Within PlatformIO, use the [platformio.ini](platformio.ini) available configuration

```
board_build.partitions = min_spiffs.csv
```

Within the Arduino IDE, from `Tools > Partition Scheme`

![Partition settings](software/building/partition_setting.png)


## Possible enhancements and ideas
   
See [issues with label enhancement](https://github.com/renatobo/bonogps/issues?q=is%3Aissue+is%3Aopen+label%3Aenhancement) on the github project