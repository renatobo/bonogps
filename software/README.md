# Software development and build

- [IDE options/suggestions: Arduino IDE or VS Code+Platformio](#ide-optionssuggestions-arduino-ide-or-vs-codeplatformio)
  - [If you just starting I recommmend Arduino IDE](#if-you-just-starting-i-recommmend-arduino-ide)
  - [PlatformIO](#platformio)
- [External Libraries](#external-libraries)
- [Optional external libraries](#optional-external-libraries)
- [Built-in libraries](#built-in-libraries)
- [Important: Partition size](#important-partition-size)

## IDE options/suggestions: Arduino IDE or VS Code+Platformio

Development is active on the [VS Code + Platformio](https://platformio.org/install/ide?install=vscode) combination: you can download the complete repository and work directly.

### If you just starting I recommmend Arduino IDE

Code is written to be compatible with the Arduino IDE, there are a couple of steps required, starting from the assumption you have already installed and setup the Arduino IDE for ESP32 (if you have not, [here a nice tutorial](https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/))

- install 'External libraries' listed below
- create an empty project
- copy the content of `src/bonogps.cpp` in the `.ino` file just created
- copy `include/bonogps_board_settings.h` in the same folder
- choose your board: "ESP32 Dev Module" (the generic board that everyone has, often tagged DOIT) or "LOLIN D32 PRO" are supported, otherwise you might have to redefine your pins in `bonogps_board_settings.h`
- choose a partition schema with enough space (e.g. the Minimal SPIFSS with 1.9Mb of flash space)

The rest is common to any other build on the Arduino IDE.

IF you are unsure of what board you are running, [check this introductory tutorial](https://randomnerdtutorials.com/getting-started-with-esp32/).

### PlatformIO

Beside install PlatformIO (on VS Code as a recommendation), the build system uses a custom **python** script to determine the current software release version: `git_rev_macro.py` and it expects the project folder to be downloaded from github directly to build up the `GIT_REV` and `GIT_REPO` macro variables correctly.

If you are having issues, you can remove the line that invokes `git_rev_macro.py` and optionally set the 2 macro `GIT_REV` and `GIT_REPO` manually.

## External Libraries

- [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino): 1.0.2 or better latest github version directly (default in platformio from v1.1 of bonogps)
- [Uptime Library](https://github.com/YiannisBourkelis/Uptime-Library) ^1.0.0
- [EasyButton](https://easybtn.earias.me/) ^2.0.1
- [Task Scheduler](https://github.com/arkhipenko/TaskScheduler) ^3.2.0

From v1.1 of this code and using the latest code of NimBLE-Arduino (after this [commit](https://github.com/h2zero/NimBLE-Arduino/commit/569eb8a188c78fe780f4c2a24cf9247532cf55ea)), unnecessary BLE code is not compiled in. This reduces flash size by ~30kb of Nimble-Arduino disabling roles `CONFIG_BT_NIMBLE_ROLE_CENTRAL` and `CONFIG_BT_NIMBLE_ROLE_OBSERVER`. Up to version 1.0.2 of NimBLE-Arduino, you can manually uncomment those roles in `nimconfig.h` .

## Optional external libraries

Included via `#define` options

- [NeoGPS](https://github.com/SlashDevin/NeoGPS)  [not included right now, but coded and available for some additional cases]

## Built-in libraries

Always included

- WebServer
- FS
- Preferences
- WiFi
- DNSServer
- ESPmDNS
- Update
- BluetoothSerial

Included via `#define`

- ArduinoOTA: this really depends on how you prefer to update software on your device. It adds size to the flash and it uses some memory, so if you don't plan on using it, don't include it.

## Important: Partition size

You have to select a partitioning schema with 1.7 Mb of programming space (e.g. Minimal SPIFF with 1.9Mb), as the app with its libraries tend to be pretty large due to BT stacks.

Within PlatformIO, use the [platformio.ini](platformio.ini) available configuration

```text
board_build.partitions = min_spiffs.csv
```

Within the Arduino IDE, from `Tools > Partition Scheme`

![Partition settings](software/building/partition_setting.png
