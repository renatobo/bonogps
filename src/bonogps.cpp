/******************************************************************************

  Repeat NMEA sentences read on UART to TCP, BLE, BT-SPP 
  You can interface uBloxCenter via TCP/IP, only issue in this case is the fixed GPS_BAUD_RATE
  A web configuration panel allows changing select uBlox parameters on the fly: access it via http://bonogps.local (when in WiFi AP mode, this becomes http://10.0.0.1)

  Renato Bonomini https://github.com/renatobo

  The main goal of this is interfacing with apps running on your phone.

  Harry's LapTimer:
  - NMEA parsing offered by HLT directly. Enable or Disable GSV/GSA as needed
  - WiFi: TCP at port 8118
  - tested with v24
  - BLE and WiFi supported and tested on iPhone (tested with 7). BLE handles 5Hz and 10Hz  when GSV/GSA disabled
  - BT-SPP and WiFi supported and tested on Android

  TrackAddict
  - tested with v4.6.0 on Android
  - BT-SPP is the only option
  - it needs a specific set of NMEA messages configuration, available as option
  - NBP was coded, yet not going to matter for TrackAddict as reported in https://forum.hptuners.com/showthread.php?78403-GPS-over-NBP&highlight=gps
  - Notes for the Classic BT-SPP version to optimize transfer to TrackAddict on Android
    - Ideally, TrackAddict wants RMC, GGA, and GLL messages, with RMC and GGA being the recommended minimum.
    - Track Addict is similarly configured to only accept NMEA messages with a GPS talker ID (i.e., $GPRMC instead of $GNRMC) https://forum.hptuners.com/showthread.php?69123-Track-addict-external-GPS&highlight=gps

  RaceChrono
  - tested with v7.0.10 free (thus satellites view untested) on Android
  - BT-SPP is the only option

  Compiling notices
  - you have to select a partitioning schema with 1.6 Mb of programming space (e.g. Minimal SPIFF with 1.9Mb), as the app with its libraries tend to be pretty large due to BT stacks

  esp32.menu.PartitionScheme.min_spiffs=Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)
  esp32.menu.PartitionScheme.min_spiffs.build.partitions=min_spiffs
  esp32.menu.PartitionScheme.min_spiffs.upload.maximum_size=1966080

  Enhancements:
   
  - add display: https://randomnerdtutorials.com/esp32-ssd1306-oled-display-arduino-ide/
  - add battery and expose battery level via BLE and/or web interface
  - log to SD  https://randomnerdtutorials.com/esp32-data-logging-temperature-to-microsd-card/
  - test https://apps.apple.com/us/app/espressif-esptouch/id1071176700 or https://apps.apple.com/in/app/esp-ble-provisioning/id1473590141
  - OTA via browser or from internet location https://lastminuteengineers.com/esp32-ota-web-updater-arduino-ide/ https://github.com/chrisjoyce911/esp32FOTA or https://github.com/platformio/bintray-secure-ota
  - use ootb library at https://os.mbed.com/teams/ublox/code/gnss/ or https://github.com/ldab/u-blox_GNSS
  - use packed binary format with custom binary parser for HLT
  - use https with https://github.com/fhessel/esp32_https_server and https://github.com/fhessel/esp32_https_server_compat
  - restore usage of https://github.com/khoih-prog/ESPAsync_WiFiManager and Async webserver (issues with ASyncTCP not being able to send info back outside of request/reply

  Additional Libraries
  - Nimble-Arduino https://github.com/h2zero/NimBLE-Arduino
  - Uptime Library https://github.com/YiannisBourkelis/Uptime-Library
  - EasyButton https://easybtn.earias.me/
  - WiFiManager https://github.com/tzapu/WiFiManager 

  Optional libraries depending on #define options
  - Task Scheduler https://github.com/arkhipenko/TaskScheduler [included by default]
  - NeoGPS https://github.com/SlashDevin/NeoGPS [not included right now, but coded and available for some additional cases]

  Built-in libraries
  - WebServer
  - FS
  - Preferences
  - WiFi
  - DNSServer
  - ESPmDNS
  - ArduinoOTA
  - Update
  - BluetoothSerial 

******************************************************************************/

// For PlatformIO or VS Code IDE
#include <Arduino.h>

// Enable or disable compiling features
#define WIFIMANAGER     // library to manage credentials and connectivity to an existing WiFi
#define UPTIME          // add library to display for how long the device has been running
#define BTSPPENABLED    // add BT-SPP stack, remove if unnecessary as it uses quite a bit of flash space
#define BLEENABLED      // add BLE stack, remove if unnecessary as it uses quite a bit of flash space
#define ENABLE_OTA      // add code for OTA - decent memory save
#define MDNS_ENABLE     // Enable or disable mDNS - currently not working in all conditions. Small memory save to be worth removing
#define TASK_SCHEDULER  // enable adv task scheduler. Small memory save to be worth removing
#define BUTTON          // Enable changing WIFI_STA and WIFI_AP via Boot button. Small memory save to be worth removing - Which button is defined below

// Configure names and PIN's used for interfacing with external world
#define DEVICE_MANUFACTURER "https://github.com/renatobo"
#define DEVICE_NAME "Bono GPS"
// BONOGPS_FIRMWARE_VER is used in BLE "Firmware" data and in the /status page of web configuration
#define BONOGPS_BUILD_DATE __TIMESTAMP__
#ifdef GIT_REV
#define BONOGPS_FIRMWARE_VER GIT_REV
#else
// the following define is needed to display version when building this with the Arduino IDE 
#define BONOGPS_FIRMWARE_VER "v0.1.1"
#endif
// Bonjour DNS name, access the GPS configuration page by appending .local as DNS
#define BONOGPS_MDNS "bonogps"
// Prefix for the AP name, followed by 4 digits device id
#define BONOGPS_AP "BonoGPS"
#define BONOGPS_PWD "bono5678"
// the AP name is generated on the fly, this is its maximum size
#define MAX_AP_NAME_SIZE 20

// How large should the RX buffer for the GPS port - more than 512 creates issues
#define UART_BUFFER_SIZE_RX  512
// Size of the intermediate buffer where we are storing the current sentence
#define MAX_UART_BUFFER_SIZE 512

// TCP Port for NMEA sentences repeater, used for Harry's LapTimer mostly, but also for proxying with uBlox
#define NMEA_TCP_PORT 1818
// Disabling GPS over NBP as it does not override phone data within TrackAddict right now, so kind of useless mostly
// #define NUMERICAL_BROADCAST_PROTOCOL
#ifdef NUMERICAL_BROADCAST_PROTOCOL
#define NEED_NEOGPS
#define NBP_TCP_PORT 35000
#endif

// Define for how long the WiFi access manager is available for
#define WIFI_MGT_TIMEOUT 300
// Which pin controls the button to switch to STA mode
#define WIFI_MODE_BUTTON 0

// Define configuration of Bluetooth Low Energy
#define BLE_DEVICE_ID           "BonoGPS"
#define BLE_SERVICE_UUID        (uint16_t)0x1819 // SERVICE_LOCATION_AND_NAVIGATION_UUID  
#define BLE_CHARACTERISTIC_UUID (uint16_t)0x2A67 // CHARACTERISTIC_LOCATION_AND_SPEED_CHARACTERISTIC_UUID
#define BLE_MTU 185

// GPS port on UART2
#define gpsPort Serial2
#define GPS_STANDARD_BAUD_RATE 115200
// Min size of a NMEA message
#define MIN_NMEA_MESSAGE_SIZE 6

// Define the serial monitor port
#define SerialMonitor Serial
#define LOG_BAUD_RATE 115200
// LED_BUILTIN is 2 on ESP32
#define LED_BUILTIN 2

/********************************
 * 
 * nothing you should need to configure from here below
 * 
********************************/

#if !( defined(ESP32) )
#error This code is designed to run only on the ESP32 board
#endif

// Library to store desidered preferences in NVM
#include <Preferences.h>
Preferences prefs;
// data to be stored
#include "esp_wifi.h"
#include <WiFi.h>
typedef struct {
  unsigned long gps_baud_rate = GPS_STANDARD_BAUD_RATE; // initial or current baud rate
  uint8_t gps_rate = 5;
  WiFiMode_t wifi_mode = WIFI_AP;
  bool nmeaGSA = false;
  bool nmeaGSV = false;
  bool nmeaGBS = true;
  bool ble_active = true;
  bool btspp_active = false;
  bool trackaddict = false;
  uint8_t nmeaGSAGSVpolling = 0; 
} stored_preference_t;

stored_preference_t stored_preferences;

// esp32 and GPS runtime variables
uint16_t chip; // hold the device id to be used in broadcasting unit identifier strings
// WiFi runtime variables
char ap_ssid[MAX_AP_NAME_SIZE];
const char ap_password[] = BONOGPS_PWD;
int max_buffer = 0;

// for status
#ifdef UPTIME
#include "uptime_formatter.h"
#endif
bool gps_powersave = false;

// The next section is used to parse the content of messages locally instead of proxying them
// This can be used for
// - NBP: unused right now
// - Creating a custom binary packed format
#ifdef NEED_NEOGPS
// add libraries to parse GPS responses
//#define NMEAGPS_KEEP_NEWEST_FIXES true
//#define NMEAGPS_PARSING_SCRATCHPAD
//#define NMEAGPS_TIMESTAMP_FROM_INTERVAL
//#define NMEAGPS_DERIVED_TYPES
//#define NMEAGPS_PARSE_PROPRIETARY
//#define NMEAGPS_PARSE_MFR_ID
#define TIME_EPOCH_MODIFIABLE
#undef NEOGPS_PACKED_DATA
#include <GPSfix_cfg.h>
#include <NMEAGPS.h>
#if !defined( GPS_FIX_TIME )
#error You must define GPS_FIX_TIME in GPSfix_cfg.h!
#endif
#if !defined( GPS_FIX_LOCATION )
#error You must uncomment GPS_FIX_LOCATION in GPSfix_cfg.h!
#endif
#if !defined( GPS_FIX_SPEED )
#error You must uncomment GPS_FIX_SPEED in GPSfix_cfg.h!
#endif
#if !defined( GPS_FIX_ALTITUDE )
#error You must uncomment GPS_FIX_ALTITUDE in GPSfix_cfg.h!
#endif
NMEAGPS gps;
gps_fix my_fix;
#endif

// WiFi Manager to set up AP credentials
#ifdef WIFIMANAGER
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#else

#endif
#ifdef MDNS_ENABLE
#include <ESPmDNS.h>
#endif

// OTA
#ifdef ENABLE_OTA
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#endif

//for LED status and OTA updates
#ifdef TASK_SCHEDULER
#include <TaskScheduler.h>
// Scheduler for periodic tasks
Scheduler ts;
#endif

// BLueTooth classic stack (Android)
#ifdef BTSPPENABLED
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;
bool bt_deviceConnected = false;
void bt_spp_stop();
void bt_spp_start();
void bt_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
#endif

#ifdef BLEENABLED
// BLE stack
#include <NimBLEDevice.h>
// ble runtime
String ble_client = "";
bool ble_deviceConnected = false;
void ble_start();
void ble_stop();
#endif

#if defined(BLEENABLED) || defined(BTSPPENABLED)
char ble_device_id[MAX_AP_NAME_SIZE];
#endif

// Respond to button
#ifdef BUTTON
#include <EasyButton.h>
// Instance of the button to switch wifi mode
EasyButton button(WIFI_MODE_BUTTON);
#endif

/********************************
 * 
 * Declarations of prototypes required by VS Code or PlatformIO
 * 
********************************/

void WebConfig_start();
void gps_initialize_settings();
void restart_after_sleep();
void wifi_AP();
#ifdef ENABLE_OTA
void handle_OTA();
void OTA_start();
#endif

/********************************

   Utilities

 * ******************************/
String generate_version() {
#ifdef BUILD_ENV_NAME
  return String(BONOGPS_FIRMWARE_VER) +String(" [") + String(BUILD_ENV_NAME) +String("]");
#else
  return String(BONOGPS_FIRMWARE_VER);
#endif
}

/********************************

   GPS Settings
   These sentences are used to configure a uBlox series 8 device

 * ******************************/
// set rate of GPS to 5hz
const char UBLOX_INIT_5HZ[]  PROGMEM = { 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A };
// set rate of GPS to 10Hz
const char UBLOX_INIT_10HZ[] PROGMEM = { 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12 };
// set rate of GPS to 1Hz
const char UBLOX_INIT_1HZ[]  PROGMEM = { 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xE8, 0x03, 0x01, 0x00, 0x01, 0x00, 0x01, 0x39 };
// set rate of GPS to 1Hz
const char UBLOX_INIT_16HZ[]  PROGMEM = { 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xE8, 0x03, 0x01, 0x00, 0x01, 0x00, 0x01, 0x39 };

// Enable GxGSA (GNSS DOP and Active Satellites)
const char UBLOX_GxGSA_ON[]  PROGMEM = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00, 0x05, 0x41};
// Disable GxGSA
const char UBLOX_GxGSA_OFF[] PROGMEM = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31};
// Enable GxGSV (Sat View)
const char UBLOX_GxGSV_ON[]  PROGMEM = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00, 0x06, 0x48};
// Disable GxGSV
const char UBLOX_GxGSV_OFF[] PROGMEM = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38};
// set Max SVs per Talker id to 8
const char UBLOX_INIT_CHANNEL_8[] PROGMEM = {  0xB5, 0x62, 0x06, 0x17, 0x14, 0x00, 0x00, 0x41, 0x08, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7D, 0xE7 };
// standard max SVs and extended digits for unsupported SVs
const char UBLOX_INIT_CHANNEL_ALL[] PROGMEM =   {0xB5, 0x62, 0x06, 0x17, 0x14, 0x00, 0x00, 0x41, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x76, 0x63 };
// use GP as Main talker and GSV Talker ID - this is needed for some apps like TrackAddict
const char UBLOX_INIT_MAINTALKER_GP[] PROGMEM = {0xB5, 0x62, 0x06, 0x17, 0x14, 0x00, 0x00, 0x41, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x78 };
// set gps port BAUD rate
const char UBLOX_BAUD_57600[]  PROGMEM = { 0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xE1, 0x00, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDA, 0xA9  };
const char UBLOX_BAUD_38400[]  PROGMEM = { 0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x96, 0x00, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8F, 0x70  };
const char UBLOX_BAUD_115200[] PROGMEM = { 0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBC, 0x5E  };
// power saving
// enable power saving mode for 1800 or 3600 seconds
// UBX - RXM - PMREQ request 
const char UBLOX_PWR_SAVE_30MIN[] PROGMEM = { 0xB5, 0x62, 0x02, 0x41, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x77, 0x1B, 0x00, 0x02, 0x00, 0x00, 0x00, 0xE8, 0x00, 0x00, 0x00, 0x0F, 0xF6 };
const char UBLOX_PWR_SAVE_1HR[] PROGMEM =   { 0xB5, 0x62, 0x02, 0x41, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xEE, 0x36, 0x00, 0x02, 0x00, 0x00, 0x00, 0xE8, 0x00, 0x00, 0x00, 0xE1, 0x21 };
//, 0xrestart UBX-CFG-RST with controlled GNSS only software, hotstart (<4 hrs) so that ephemeris still valid
const char UBLOX_WARMSTART[] PROGMEM =    { 0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x02, 0x00, 0x10, 0x68 };
// Display precision in some apps
const char UBLOX_GxGBS_ON[] PROGMEM =    {  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x09, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00, 0x0C, 0x72};
const char UBLOX_GxGBS_OFF[] PROGMEM =    { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x62};
// Poll GSA
const char UBLOX_GNGSA_POLL[] PROGMEM = "$EIGNQ,GSA*2D\r\n";
// Poll GSV
const char UBLOX_GNGSV_POLL[] PROGMEM = "$EIGNQ,GSV*3A\r\n";
bool gsaorgsv_turn = true;

void push_gps_message(const char message[], int message_size = 0) {
  if (gps_powersave) {
    log_d("Disable powermode");
    gpsPort.end();
    delay(1000);
    // assumes stored_preferences global
    gps_powersave=false;
    gps_initialize_settings();
  }
  for (int i = 0; i < message_size; i++) {
    gpsPort.write(message[i]);
  }
}

#ifdef TASK_SCHEDULER
void poll_GSA_GSV_info() {
  if (gsaorgsv_turn) {
    gpsPort.write(UBLOX_GNGSA_POLL);
  } else {
    gpsPort.write(UBLOX_GNGSV_POLL);
  }
  gsaorgsv_turn=!gsaorgsv_turn;
}

Task tpoll_GSA_GSV_info ( 0, TASK_FOREVER , poll_GSA_GSV_info, &ts, false );
void control_poll_GSA_GSV(int frequency){
  if (frequency > 0) {
    push_gps_message(UBLOX_GxGSV_OFF, sizeof(UBLOX_GxGSV_OFF));
    push_gps_message(UBLOX_GxGSA_OFF, sizeof(UBLOX_GxGSA_OFF));
    tpoll_GSA_GSV_info.setInterval( frequency * 500 );
    tpoll_GSA_GSV_info.enable();
  } else {
    tpoll_GSA_GSV_info.disable();
  }
}

// handle restart when config is lost
Task trestart_after_sleep (0, 1, restart_after_sleep, &ts, false);
void restart_after_sleep(){
  gps_powersave = false;
  push_gps_message(UBLOX_WARMSTART, sizeof(UBLOX_WARMSTART));
  delay(1000);
  gpsPort.end();
  delay(1000);
  // assumes 'stored_preferences' is global
  gps_initialize_settings();
  trestart_after_sleep.disable();
}
#endif

void switch_baudrate(uint32_t newbaudrate) {
  // stored_preferences assumed global
  log_d("Send UBX-CFG for rate %d", newbaudrate);
  switch (newbaudrate) {
    case 38400:
      push_gps_message(UBLOX_BAUD_38400, sizeof(UBLOX_BAUD_38400));
      break;
    case 57600:
      push_gps_message(UBLOX_BAUD_57600, sizeof(UBLOX_BAUD_57600));
      break;
    case 115200:
      push_gps_message(UBLOX_BAUD_115200, sizeof(UBLOX_BAUD_115200));
      break;
    default:
      push_gps_message(UBLOX_BAUD_38400, sizeof(UBLOX_BAUD_38400));
      break;
  }
  log_d("Flush UART");
  gpsPort.flush();
  delay(500);
  log_d("end connection UART");
  gpsPort.end();
  delay(100);
  log_d("Start new connection UART");
  gpsPort.begin(newbaudrate);
  stored_preferences.gps_baud_rate = newbaudrate;
}

// read from NVM via preferences library
void ReadNVMPreferences() {
  // prefs assumed global
  // stored_preferences assumed global
  String string_wifi_mode;

  switch (stored_preferences.wifi_mode) {
    case WIFI_AP:
      string_wifi_mode = "WIFI_AP";
      break;
    case WIFI_STA:
      string_wifi_mode = "WIFI_STA";
      break;
    default:
      string_wifi_mode = "WIFI_STA";
      break;
  }

  stored_preferences.gps_baud_rate = prefs.getULong("gpsbaudrate", stored_preferences.gps_baud_rate);
  stored_preferences.gps_rate = prefs.getUChar("gpsrate", stored_preferences.gps_rate);
  stored_preferences.nmeaGSAGSVpolling = prefs.getUChar("gsagsvpoll", stored_preferences.nmeaGSAGSVpolling);
  // wifi mode TODO
  string_wifi_mode = prefs.getString("wifi", string_wifi_mode);
  if (string_wifi_mode == "WIFI_AP") {
    stored_preferences.wifi_mode = WIFI_AP;
  } else {
    stored_preferences.wifi_mode = WIFI_STA;
  }

  stored_preferences.nmeaGSA = prefs.getBool("nmeagsa", stored_preferences.nmeaGSA);
  stored_preferences.nmeaGSV = prefs.getBool("nmeagsv", stored_preferences.nmeaGSV);
  stored_preferences.nmeaGBS = prefs.getBool("nmeagbs", stored_preferences.nmeaGBS);
  stored_preferences.ble_active = prefs.getBool("ble", stored_preferences.ble_active);
  stored_preferences.btspp_active = prefs.getBool("btspp", stored_preferences.btspp_active);
  stored_preferences.trackaddict = prefs.getBool("trackaddict", stored_preferences.trackaddict);

  if (stored_preferences.ble_active && stored_preferences.btspp_active) {
    // prevents both settings to be ON
#ifdef BLEENABLED
    stored_preferences.btspp_active = false;
#else
    stored_preferences.ble_active = false;
#endif
  }
}

// write to preferences
void StoreNVMPreferences(bool savewifi = false) {
  // prefs assumed global
  // stored_preferences assumed global
  String string_wifi_mode;
  size_t size_t_written;

  if (savewifi) {
    switch (stored_preferences.wifi_mode) {
      case WIFI_AP:
        string_wifi_mode = "WIFI_AP";
        break;
      case WIFI_STA:
        string_wifi_mode = "WIFI_STA";
        break;
      default:
        string_wifi_mode = "WIFI_STA";
        break;
    }
    if (stored_preferences.wifi_mode == WIFI_AP) {
      string_wifi_mode == "WIFI_AP";
    } else {
      string_wifi_mode = "WIFI_STA";
    }
    size_t_written = prefs.putString("wifi", string_wifi_mode);
  }
  size_t_written = prefs.putULong("gpsbaudrate", stored_preferences.gps_baud_rate);
  size_t_written = prefs.putUChar("gpsrate", stored_preferences.gps_rate);
  size_t_written = prefs.putUChar("gsagsvpoll", stored_preferences.nmeaGSAGSVpolling);
  size_t_written = prefs.putBool("nmeagsa", stored_preferences.nmeaGSA);
  size_t_written = prefs.putBool("nmeagsv", stored_preferences.nmeaGSV);
  size_t_written = prefs.putBool("nmeagbs", stored_preferences.nmeaGBS);
  size_t_written = prefs.putBool("ble", stored_preferences.ble_active);
  size_t_written = prefs.putBool("btspp", stored_preferences.btspp_active);
  size_t_written = prefs.putBool("trackaddict", stored_preferences.trackaddict);
  if (size_t_written >0) {
    log_i("Preferences written");
  } else {
    log_e("Preferences NOT written");
  }
}

void StoreNVMPreferencesWiFi(String string_wifi_mode) {
  // prefs assumed global
  size_t size_t_written;
  size_t_written = prefs.putString("wifi", string_wifi_mode);
  if (size_t_written >0) {
    log_i("Preferences written");
  } else {
    log_e("Preferences NOT written");
  }
}

void gps_enable_trackaddict() {
  log_d("Setting GPS to specific Track Addict needs");
  push_gps_message(UBLOX_GxGSA_OFF, sizeof(UBLOX_GxGSA_OFF));
  stored_preferences.nmeaGSA=false;
  push_gps_message(UBLOX_GxGSV_OFF, sizeof(UBLOX_GxGSA_OFF));
  stored_preferences.nmeaGSV=false;
  push_gps_message(UBLOX_GxGBS_OFF, sizeof(UBLOX_GxGBS_OFF));
  stored_preferences.nmeaGBS=false;
  push_gps_message(UBLOX_INIT_MAINTALKER_GP, sizeof(UBLOX_INIT_MAINTALKER_GP));
  stored_preferences.trackaddict=true;
  push_gps_message(UBLOX_INIT_10HZ, sizeof(UBLOX_INIT_10HZ));
  stored_preferences.gps_rate = 10;
#ifdef TASK_SCHEDULER
  control_poll_GSA_GSV(0);
#endif  
  stored_preferences.nmeaGSAGSVpolling=0;
}

#ifdef NUMERICAL_BROADCAST_PROTOCOL
const uint NBPServerPort = NBP_TCP_PORT;
WiFiServer NBPServer(NBPServerPort);
WiFiClient NBPRemoteClient;
#endif

const uint NMEAServerPort = NMEA_TCP_PORT;
WiFiServer NMEAServer(NMEAServerPort);
WiFiClient NMEARemoteClient;

void led_blink() {
  //toggle state
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));     // set pin to the opposite state
}

#ifdef TASK_SCHEDULER
Task tLedBlink ( 0, TASK_FOREVER , &led_blink, &ts, true );
#endif

/********************************

    WiFi connection

 * ******************************/
const char html_text[] PROGMEM = "text/html";
bool wifi_connected = false;
IPAddress MyIP;

#ifdef WIFIMANAGER
//gets called when WiFiManager enters configuration mode
void configModeCallback (WiFiManager *myWiFiManager) {
  log_i("Entered config mode, connect to %s", ap_ssid);
  log_i("IP: %s", WiFi.softAPIP().toString());
  //if you used auto generated SSID, print it
  log_i("SSID: %s", myWiFiManager->getConfigPortalSSID());
  //entered config mode, make led toggle faster
#ifdef TASK_SCHEDULER
  tLedBlink.setInterval(100);
  tLedBlink.enableIfNot();
#endif
}
#endif

void NMEACheckForConnections() {
  if (NMEAServer.hasClient())
  {
    // If we are already connected to another computer, then reject the new connection. Otherwise accept the connection.
    if (NMEARemoteClient.connected())
    {
      log_w("NMEA TCP Connection rejected");
      NMEAServer.available().stop();
    } else {
      NMEARemoteClient = NMEAServer.available();
      // TODO not printing strings correctly
      log_d("NMEA TCP Connection accepted from client: %s", NMEARemoteClient.remoteIP().toString());
    }
  }
}
void start_NMEA_server() {
  log_i("Start GNSS TCP/IP Service");
  NMEAServer.begin();
  NMEAServer.setNoDelay(true);
}

#ifdef NUMERICAL_BROADCAST_PROTOCOL
unsigned long nbptimer = 0;
void NBPCheckForConnections() {
  if (NBPServer.hasClient())
  {
    // If we are already connected to another computer, then reject the new connection. Otherwise accept the connection.
    if (NBPRemoteClient.connected())
    {
      log_w("NBP TCP Connection rejected");
      NBPServer.available().stop();
    } else {
      NBPRemoteClient = NBPServer.available();
      log_i("NBP TCP Connection accepted from client: %s", NBPRemoteClient.remoteIP().toString());
    }
  }
}
void start_NBP_server() {
  log_i("Start NBP TCP/IP Service");
  NBPServer.begin();
  NBPServer.setNoDelay(true);
}
#endif

// Start STATION mode to connect to a well-known Access Point
void wifi_STA() {
  // WiFi Access
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  //set led pin as output
  pinMode(LED_BUILTIN, OUTPUT);
  // start ticker_wifi with 50 ms because we start in STA mode and try to connect
#ifdef TASK_SCHEDULER
  tLedBlink.setInterval(50);
  tLedBlink.enableIfNot();
#endif

#ifdef WIFIMANAGER
  //WiFiManager //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wm;
  // disable debug logging
  wm.setDebugOutput(false);
  // set dark theme
  wm.setClass("invert");
  // set configportal timeout
  wm.setConfigPortalTimeout(WIFI_MGT_TIMEOUT);
  //reset settings - for testing
  // wm.resetSettings();
  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wm.setAPCallback(configModeCallback);
  // set a static ip that we recognize
  wm.setAPStaticIPConfig(IPAddress(10, 0, 0, 1), IPAddress(10, 0, 0, 1), IPAddress(255, 255, 255, 0));
  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point named 'ap_ssid' with password 'ap_password' and goes into a blocking loop awaiting configuration
  if (!wm.autoConnect(ap_ssid, ap_password)) {
    log_e("failed to connect and hit timeout");
    // ESP.restart();
    // delay(1000);
    wifi_connected = false;
  } else {
    wifi_connected = true;
    //if you get here you have connected to the WiFi
    log_i("connected to wifi...yeey:)");
#ifdef TASK_SCHEDULER
    tLedBlink.setInterval(500);
    tLedBlink.enableIfNot();
#endif
    //keep LED on
    digitalWrite(LED_BUILTIN, LOW);

#ifdef MDNS_ENABLE
    if (!MDNS.begin(BONOGPS_MDNS)) {
      log_e("Error setting up MDNS responder!");
    } else {
      log_i("mDNS responder started");
    }
#endif

#ifdef ENABLE_OTA
    log_i("Start OTA service");
    OTA_start();
#endif
    log_i("Start Web Portal");
    WebConfig_start();
    // WLAN Server for GNSS data
    start_NMEA_server();
#ifdef NUMERICAL_BROADCAST_PROTOCOL
    // WLAN Server for GNSS data
    start_NBP_server();
#endif

  }
#endif // #ifdef WIFIMANAGER
  MyIP = WiFi.localIP();

#ifdef BUTTON
  button.onPressed(wifi_AP);
#endif
  stored_preferences.wifi_mode = WIFI_STA;

}

void wifi_AP() {
  // WiFi Access
  WiFi.mode(WIFI_AP); // explicitly set mode, esp defaults to STA+AP
  // Set WiFi options
  wifi_country_t country = {
    .cc = "US",
    .schan = 1,
    .nchan = 13,
    .max_tx_power = 20,
    .policy = WIFI_COUNTRY_POLICY_AUTO,
  };
  esp_wifi_set_country(&country);
  WiFi.softAP(ap_ssid, ap_password, 1, 0, 2);
  log_i("Wait 100 ms for AP_START...");
  delay(100);
  log_d("Power: %d", WiFi.getTxPower());

  log_i("Set softAPConfig");
  IPAddress Ip(10, 0, 0, 1);
  IPAddress NMask(255, 255, 255, 0);
  WiFi.softAPConfig(Ip, Ip, NMask);

  MyIP = WiFi.softAPIP();
  log_i("AP IP address: %s", MyIP.toString());

  wifi_connected = true;

#ifdef MDNS_ENABLE
  if (!MDNS.begin(BONOGPS_MDNS)) {
    log_e("Error setting up MDNS responder!");
  } else {
    log_i("mDNS responder started");
  }
#endif

  log_i("Start Web Portal");
  WebConfig_start();

#ifdef TASK_SCHEDULER
  tLedBlink.setInterval(500);
  tLedBlink.enableIfNot();
#endif

  // WLAN Server for GNSS data
  start_NMEA_server();
#ifdef NUMERICAL_BROADCAST_PROTOCOL
  // WLAN Server for GNSS data
  start_NBP_server();
#endif

#ifdef BUTTON
  button.onPressed(wifi_STA);
#endif
  stored_preferences.wifi_mode = WIFI_AP;

}

/********************************

   Web Configuration portal

 * ******************************/
#ifdef ESPWEBSERVER // this conflicts with WebServer.h included in WiFiManager
#include <ESPWebServer.hpp>
ESPWebServer webserver(80);
#else
#include <WebServer.h>
WebServer webserver(80);
#endif

// Helper to populate a page with style sheet
String generate_html_body(String input, bool add_menu = true) {
  // assumption: ap_ssid is populated
  return String(
    String(F("<!DOCTYPE html><html lang='en'><head><title>"))
    + ap_ssid 
    + F("</title><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'><style>*{box-sizing:border-box;text-align:center;width:100%;font-weight:300}body{font-family:Roboto,system-ui,Arial,Helvetica,sans-serif;font-size:5vw;margin:0}header{background-color:#666;padding:.5vw;text-align:center;color:#fff}article{float:left;padding:10px;width:100%;height:auto}details{display:table;clear:both}summary{font-size:larger;font-weight:400;padding:10px;background-color:#f1f1f1}footer{background-color:#777;padding:.2vw;text-align:center;color:#fff;clear:both;position:fixed;bottom:0;font-size:small}@media (min-width:800px){article{width:50%}*{font-size:2.5vw}}a,input{color:#fff;border-radius:8pt;background:red;text-decoration:none;padding:5pt}.bg input{display:none}label{border:solid;border-radius:8pt;padding:5px;margin:2px;border-color:#bdbdbd;border-width:2px;color:#9e9e9e}.bg input:checked+label,.bg input:checked+label:active{background:red;color:#fff;border-color:red}</style></head><body><script>function Select(e){fetch('/'+e).then(e=>e.text()).then(t=>console.log(e))}</script><header>")
    + (add_menu?"Back to <a href='/'>Main menu</a>":ap_ssid) 
    + (gps_powersave?"<br><em>Powersave mode</em>":"") 
    + "</header>\n"
    +  input 
#ifdef GIT_REPO   
    + "<footer>Version: <a style='font-size: small;background: none;text-decoration: underline;' target='_blank' href='"+String(GIT_REPO)+"'>"+generate_version()+"</a></footer></body></html>"
#else
    + "<footer>Version: "+generate_version()+"</footer></body></html>"    
#endif    
    );

}

String input_onoff(String divlabel, String parameter, bool parameter_status) {
  return 
  String("\n<article class='bg'>")
  +divlabel
  +String(" \n\t<input type='radio' id='")
  +parameter
  +String("/on' onchange='Select(this.id)' name='")
  +parameter
  + (parameter_status ? String("' checked>") : String("'>") )
  + String("\n\t<label class='button' for='")
  +parameter
  +String("/on'> On </label>\n\t<input type='radio' id='")
  +parameter
  +String("/off' onchange='Select(this.id)' name='")
  +parameter
  + (parameter_status ? String("'>") : String("' checked>"))
  +String("\n\t<label class='button' for='")
  +parameter
  +String("/off'> Off </label>\n\t</article>");
}

String html_select(String divlabel,String parameters[], String parameters_names[], bool parameters_status[], String groupname, int numberparams) {
  String result = String("\n<article class='bg'>")+divlabel;
  for (int i = 0; i < numberparams ; i++) {
    result = result
      +String("\n\t<input type='radio' id='")
      +groupname
      +String("/")
      +parameters[i]
      +String("'  onchange='Select(this.id)' name='")
      +groupname
      +(parameters_status[i] ? String("' checked>") : String("'>") )
      +String("\n\t<label class='button' for='")
      +groupname
      +String("/")
      +parameters[i]
      +String("'> ")
      +parameters_names[i]
      +String(" </label>");
  }
  return result+String("\n</article>");
}

void handle_menu() {
  // variables used to build html buttons
  String sv_values[2] = {"8","all"};
  String sv_names[2] = {"8","All"};
  bool sv_status[2] = {false,true};
  
  String baud_values[3] = {"38400","57600","115200"};
  String baud_names[3] = {"38k4","57k6","115k2"};
  bool baud_status[3] = {(stored_preferences.gps_baud_rate == 38400 ),(stored_preferences.gps_baud_rate == 57600 ), (stored_preferences.gps_baud_rate == 115200 )};

  String wifi_values[2] = {"sta","ap"};
  String wifi_names[2] = {"Client","AP"};
  bool wifi_status[2] = {(stored_preferences.wifi_mode == WIFI_STA ),(stored_preferences.wifi_mode == WIFI_AP )};

#ifdef TASK_SCHEDULER
  String pollgsagsv_values[3] = {"0","1","5"};
  String pollgsagsv_names[3] = {"Off","1 sec","5 sec"};
  bool pollgsagsv_status[3] = {(stored_preferences.nmeaGSAGSVpolling == 0),(stored_preferences.nmeaGSAGSVpolling == 1),(stored_preferences.nmeaGSAGSVpolling == 5)};
#endif
  
  webserver.send(200, html_text, generate_html_body(
#if defined(BTSPPENABLED) || defined(BLEENABLED)
                 String(F("\n<details><summary>Predefined settings</summary>\n<article><a href='/hlt'>HarryLapTimer</a></article>")) +
#ifdef BTSPPENABLED
                 String("\n<article><a href='/racechrono'>RaceChrono</a></article>") +
                 String("\n<article><a href='/trackaddict'>TrackAddict</a></article>") +
#endif
                String("</details>") +
#endif
                 String(F("\n<details open><summary>GPS runtime settings</summary>\n<article class='bg'>Update\n<input type='radio' id='rate/1hz' onchange='Select(this.id)' name='rate'"))
                 + ((stored_preferences.gps_rate == 1 ) ? String("checked") : String(" "))
                 + String(F("><label class='button' for='rate/1hz'>1 Hz</label>\n<input type='radio' id='rate/5hz' onchange='Select(this.id)' name='rate' "))
                 + ((stored_preferences.gps_rate == 5 ) ? String("checked") : String(" "))
                 + String(F("><label class='button' for='rate/5hz'>5 Hz</label>\n<input type='radio' id='rate/10hz' onchange='Select(this.id)' name='rate' "))
                 + ((stored_preferences.gps_rate == 10 ) ? String("checked") : String(" "))
                 + String(F("><label class='button' for='rate/10hz'>10 Hz</label></article>"))
                 + input_onoff("Stream GxGBS","gbs",stored_preferences.nmeaGBS)
                 + input_onoff("Stream GxGSA","gsa",stored_preferences.nmeaGSA)
                 + input_onoff("Stream GxGSV","gsv",stored_preferences.nmeaGSV)
#ifdef TASK_SCHEDULER
                 + html_select("Poll GSA + GSV ",pollgsagsv_values,pollgsagsv_names,pollgsagsv_status,String("poll/gsagsv"),3)
#endif
                 + html_select("SVs per Talker Id",sv_values,sv_names,sv_status,String("sv"),2)
                 + html_select("Port BAUD",baud_values,baud_names,baud_status,String("baud"),3)
                 + String(F("</details>\n<details>\n<summary>Connections</summary><article>List connected <a href='/clients'>clients</a></article>"))
                 + html_select("WiFi mode ",wifi_values,wifi_names,wifi_status,String("wifi"),2)
#ifdef BLEENABLED
                 + input_onoff("Bluetooth LE","ble",stored_preferences.ble_active)
#endif
#ifdef BTSPPENABLED
                 + input_onoff("Bluetooth SPP","btspp",stored_preferences.btspp_active)
#endif
                 + String(F("\n</details>\n<details><summary>Device</summary>\n<article>Device <a href='/status'>information</a></article>\n<article>Device <a href='/restart'>restart</a></article><article>Reset <a href='/reset_wifi'>WiFi credentials</a></article>\n<article>Save <a href='/save_config'>config</a></article><article>Suspend GPS for <a href='/powersave/1800'>30'</a> <a href='/powersave/3600'>1 hr</a></article></details>")),false));
}
#ifdef BLEENABLED
void handle_ble_off() {
  log_i("Turning off BLE");
  stored_preferences.ble_active = false;
  ble_stop();
  webserver.send(200, html_text, generate_html_body(F("Turn <b>off</b> BLE<br><a href='/save_config/withoutwifi'>Save settings</a>")));
}
void handle_ble_on() {
  log_i("Turning on BLE");
  stored_preferences.ble_active = true;
#ifdef BTSPPENABLED
  stored_preferences.btspp_active = false;
  bt_spp_stop();
  webserver.send(200, html_text, generate_html_body(F("Turned <b>ON</b> BLE, and turned <b>OFF</b> BT-SPP<p>This causes a restart of the device: to store permanently, disable BT-SPP, save config, enable BLE")));
#else
  webserver.send(200, html_text, generate_html_body(F("Turned <b>ON</b> BLE<br><a href='/save_config/withoutwifi'>Save settings</a>")));
#endif
  ble_start();
}
#endif
#ifdef BTSPPENABLED
void handle_btspp_off() {
  log_i("Turning off BT-SPP");
  stored_preferences.btspp_active = false;
  bt_spp_stop();
  webserver.send(200, html_text, generate_html_body(F("Turn <b>off</b> BLE")));
}
void handle_btspp_on() {
  log_i("Turning on BT-SPP");
  stored_preferences.btspp_active = true;
#ifdef BLEENABLED
  stored_preferences.ble_active = false;
  ble_stop();
  webserver.send(200, html_text, generate_html_body(F("Turned <b>ON</b> BLE, and turned <b>OFF</b> BT-SPP<br><a href='/save_config/withoutwifi'>Save settings</a>")));
#else
  webserver.send(200, html_text, generate_html_body(F("Turned <b>ON</b> BT-SPP")));
#endif
  bt_spp_start();
}
#endif
void handle_powersave_1hr() {
  log_i("enable power saving mode for 3600 seconds");
  // disable polling of GSA and GSV
#ifdef TASKSCHEDULER  
  control_poll_GSA_GSV(0);
  trestart_after_sleep.setInterval( 3600000 );
  trestart_after_sleep.enableDelayed();
#endif  
  push_gps_message(UBLOX_PWR_SAVE_1HR, sizeof(UBLOX_PWR_SAVE_1HR));
  gps_powersave=true;
  webserver.send(200, html_text, generate_html_body(F("Enabled power saving mode for 3600 seconds")));
}
void handle_powersave_30min() {
  log_i("enable power saving mode for 1800 seconds");
  // disable polling of GSA and GSV
#ifdef TASKSCHEDULER  
  control_poll_GSA_GSV(0);
  trestart_after_sleep.setInterval( 1800000 );
  trestart_after_sleep.enableDelayed();
#endif
  push_gps_message(UBLOX_PWR_SAVE_30MIN, sizeof(UBLOX_PWR_SAVE_30MIN));
  gps_powersave=true;
  webserver.send(200, html_text, generate_html_body(F("Enabled power saving mode for 1800 seconds")));
}
void handle_rate_1hz() {
  log_i("Set GPS Rate to 1 Hz");
  push_gps_message(UBLOX_INIT_1HZ, sizeof(UBLOX_INIT_1HZ));
  stored_preferences.gps_rate = 1;
  webserver.send(200, html_text, generate_html_body(F("Rate set to <b>1 Hz</b><br><a href='/save_config/withoutwifi'>Save settings</a>")));
}
void handle_rate_5hz() {
  log_i("Set GPS Rate to 5 Hz");
  push_gps_message(UBLOX_INIT_5HZ, sizeof(UBLOX_INIT_5HZ));
  stored_preferences.gps_rate = 5;
  webserver.send(200, html_text, generate_html_body(F("Rate set to <b>5 Hz</b><br><a href='/save_config/withoutwifi'>Save settings</a>")));
}
void handle_rate_10hz() {
  log_i("Set GPS Rate to 10 Hz");
  push_gps_message(UBLOX_INIT_10HZ, sizeof(UBLOX_INIT_10HZ));
  stored_preferences.gps_rate = 10;
  webserver.send(200, html_text, generate_html_body(F("Rate set to <b>10 Hz</b><br><a href='/save_config/withoutwifi'>Save settings</a>")));
}
void handle_baudrate_38400() {
  log_i("Set BAUD Rate to 38400");
  switch_baudrate(38400);
  webserver.send(200, html_text, generate_html_body(F("Baud Rate set to <b>38k4</b><br><a href='/save_config/withoutwifi'>Save settings</a>")));
}
void handle_baudrate_57600() {
  log_i("Set BAUD Rate to 57600");
  switch_baudrate(57600);
  webserver.send(200, html_text, generate_html_body(F("Baud Rate set to <b>57k6</b><br><a href='/save_config/withoutwifi'>Save settings</a>")));
}
void handle_baudrate_115200() {
  log_i("Set BAUD Rate to 115200");
  switch_baudrate(115200);
  webserver.send(200, html_text, generate_html_body(F("Baud Rate set to <b>115k2</b><br><a href='/save_config/withoutwifi'>Save settings</a>")));
}
void handle_gsv_on() {
  log_i("Enable GxGSV messages");
#ifdef TASKSCHEDULER    
  control_poll_GSA_GSV(0);
#endif 
  push_gps_message(UBLOX_GxGSV_ON, sizeof(UBLOX_GxGSV_ON));
  stored_preferences.nmeaGSV = true;
  stored_preferences.nmeaGSAGSVpolling = 0;
  webserver.send(200, html_text, generate_html_body(F("Enabled GxGSV messages<br><a href='/save_config/withoutwifi'>Save settings</a>")));
}
void handle_gsv_off() {
  log_i("Disable GxGSV messages");
  push_gps_message(UBLOX_GxGSV_OFF, sizeof(UBLOX_GxGSV_OFF));
  stored_preferences.nmeaGSV = false;
  webserver.send(200, html_text, generate_html_body(F("Disabled GxGSV messages<br><a href='/save_config/withoutwifi'>Save settings</a>")));
}
void handle_gsa_on() {
  log_i("Enable GxGSA messages");
#ifdef TASKSCHEDULER  
  control_poll_GSA_GSV(0);
#endif
  push_gps_message(UBLOX_GxGSA_ON, sizeof(UBLOX_GxGSA_ON));
  stored_preferences.nmeaGSA = true;
  stored_preferences.nmeaGSAGSVpolling = 0;
  webserver.send(200, html_text, generate_html_body(F("Enabled GxGSA messages<br><a href='/save_config/withoutwifi'>Save settings</a>")));
}
void handle_gsa_off() {
  log_i("Disable GxGSA messages");
  push_gps_message(UBLOX_GxGSA_OFF, sizeof(UBLOX_GxGSA_OFF));
  stored_preferences.nmeaGSA = false;
  webserver.send(200, html_text, generate_html_body(F("Disabled GxGSA messages<br><a href='/save_config/withoutwifi'>Save settings</a>")));
}
#ifdef TASK_SCHEDULER
void handle_pollgsagsv_on_1() {
  log_i("Enable polling of GSA and GSV messages every 1 sec");
  control_poll_GSA_GSV(1);
  webserver.send(200, html_text, generate_html_body(F("Enabled polling of alternate GSA and GSV messages every 1 second"))); 
}
void handle_pollgsagsv_on_5() {
  log_i("Enable polling of GSA and GSV messages every 5 sec");
  control_poll_GSA_GSV(5);  
  webserver.send(200, html_text, generate_html_body(F("Enabled polling of alternate GSA and GSV messages every 5 seconds"))); 
}
void handle_pollgsagsv_off() {
  log_i("Disable polling of GSA and GSV messages every");
  control_poll_GSA_GSV(0);
  webserver.send(200, html_text, generate_html_body(F("Disabled polling of alternate GSA and GSV messages"))); 
}
#endif

void handle_gbs_on() {
  log_i("Enable GxGBS messages");
  push_gps_message(UBLOX_GxGBS_ON, sizeof(UBLOX_GxGBS_ON));
  stored_preferences.nmeaGBS = true;
  webserver.send(200, html_text, generate_html_body(F("Enabled GxGBS messages<br><a href='/save_config/withoutwifi'>Save settings</a>")));
}
void handle_gbs_off() {
  log_i("Disable GxGBS messages");
  push_gps_message(UBLOX_GxGBS_OFF, sizeof(UBLOX_GxGBS_OFF));
  stored_preferences.nmeaGBS = false;
  webserver.send(200, html_text, generate_html_body(F("Disabled GxGBS messages<br><a href='/save_config/withoutwifi'>Save settings</a>")));
}
void handle_svchannel_8() {
  log_i("Restric to 8 SVs per Talker Id");
  push_gps_message(UBLOX_INIT_CHANNEL_8, sizeof(UBLOX_INIT_CHANNEL_8));
  webserver.send(200, html_text, generate_html_body(F("Restric NMEA output to 8 SVs per Talker Id<br><a href='/save_config/withoutwifi'>Save settings</a>")));
}
void handle_svchannel_all() {
  log_i("All SVs per Talker Id");
  push_gps_message(UBLOX_INIT_CHANNEL_ALL, sizeof(UBLOX_INIT_CHANNEL_ALL));
  webserver.send(200, html_text, generate_html_body(F("All SVs per Talker Id<br><a href='/save_config/withoutwifi'>Save settings</a>")));
}
#ifdef BTSPPENABLED
void handle_trackaddict() {
  // /trackaddict main page
  webserver.send(200, html_text, generate_html_body(
    String(F("<section><h1>TrackAddict <a target='_blank' href='https://www.hptuners.com/product/trackaddict-app/'>?</a></h1><article>Required options:<br><ul><li>Talker id GPS for all systems</li><li>no GSA GSV GBS streaming</li><li>no GSA GSV polling</li><li>10 Hz updates</li><li>BT-SPP Connection only</li></ul></article>")) 
    + input_onoff("Android","trackaddict",stored_preferences.trackaddict)
    + String("<article><p>A save config and restart are recommended after enabling/disabling BT-SPP</p></article></section>")
    ));
}
void handle_trackaddict_on() {
  // /trackaddict/on
  log_i("Set optimal configuration for Track Addict");
  gps_enable_trackaddict();
// #ifdef BLEENABLED
  stored_preferences.ble_active = false;
  ble_stop();
// #endif
  bt_spp_start();
  stored_preferences.btspp_active = true;
  webserver.send(200, html_text, generate_html_body(F("Set optimal configuration for Track Addict<p><a href='/save_config/withoutwifi'>Save</a> these settings")));
}
void handle_trackaddict_off() {
  // /trackaddict/off
  log_i("Unset optimal configuration for Track Addict");
  stored_preferences.trackaddict = false;
  if (stored_preferences.nmeaGSA) push_gps_message(UBLOX_GxGSA_ON, sizeof(UBLOX_GxGSA_ON));
  if (stored_preferences.nmeaGSV) push_gps_message(UBLOX_GxGSV_ON, sizeof(UBLOX_GxGSV_ON));
  if (stored_preferences.nmeaGBS) push_gps_message(UBLOX_GxGBS_ON, sizeof(UBLOX_GxGBS_ON));
#ifdef TASK_SCHEDULER  
  control_poll_GSA_GSV(stored_preferences.nmeaGSAGSVpolling);
#endif  
  push_gps_message(UBLOX_INIT_CHANNEL_ALL, sizeof(UBLOX_INIT_CHANNEL_ALL));
  webserver.send(200, html_text, generate_html_body(F("Unset optimal configuration for Track Addict: specific talker id, all SV's.<br><a href='/save_config/withoutwifi'>Save</a> these settings")));
}
#endif

#if defined(BTSPPENABLED) || defined(BLEENABLED)
void handle_hlt(){
  // /trackaddict main page
#if defined(BTSPPENABLED) && defined(BLEENABLED)
    webserver.send(200, html_text, generate_html_body(F("<section><h1>Harry Lap Timer <a target='_blank' href='https://www.gps-laptimer.de/'>?</a></h1><article>Recommended options:<br><ul><li>GBS streaming</li><li>GSA GSV polling</li><li>10 Hz updates</li><li>Android: BT-SPP Connection</li><li>iOS: BT-LE Connection</li></ul></article><article>Load options for:<p><a href='/hlt/ios'>iOS</a></p><p><a href='/hlt/android'>Android</a></p><p>A save config and restart are recommended after enabling/disabling BT-SPP</p></article></section>")));
#else
#ifdef BLEENABLED
    webserver.send(200, html_text, generate_html_body(F("<section><h1>Harry Lap Timer <a target='_blank' href='https://www.gps-laptimer.de/'>?</a></h1><article>Recommended options:<br><ul><li>GBS streaming</li><li>GSA GSV polling</li><li>10 Hz updates</li><li>iOS: BT-LE Connection</li></ul></article><article>Load  options forn:<p><a href='/hlt/ios'>iOS</a></article></section>"));
#endif
#ifdef BTSPPENABLED
   webserver.send(200, html_text, generate_html_body(F("<section><h1>Harry Lap Timer <a href='https://www.gps-laptimer.de/'>?</a></h1><article>Recommended options:<br><ul><li>GBS streaming</li><li>GSA GSV polling</li><li>10 Hz updates</li><li>Android: BT-SPP Connection</li></ul></article><article><Load options for:<p><a href='/hlt/android'>Android</a></p><p>A save config and restart are recommended after enabling/disabling BT-SPP</article></section>"));
#endif
#endif
}
#endif

#ifdef BTSPPENABLED
void handle_racechrono() {
  // /racechrono main page
   webserver.send(200, html_text, generate_html_body(F("<section><h1>RaceChrono <a target='_blank' href='https://racechrono.com/'>?</a></h1><article>Required options:<br><ul><li>Talker id GPS for all systems</li><li>no GSA GSV GBS streaming</li><li>no GSA GSV polling</li><li>10 Hz updates</li><li>BT-SPP Connection only</li></ul></article><article><p>Load options for:<p><a href='/racechrono/android'>Android</a></p></article></section>")));
}
void handle_racechrono_android() {
  // /hlt/android
  log_i("Setting optimal configuration for RaceChrono on Android: 10Hz, GSA+GSV Off, GBS On, BT-SPP");
  push_gps_message(UBLOX_INIT_10HZ, sizeof(UBLOX_INIT_10HZ));
  stored_preferences.gps_rate = 10;
  stored_preferences.nmeaGSA = false;
  stored_preferences.nmeaGSV = false;
#ifdef TASK_SCHEDULER
  stored_preferences.nmeaGSAGSVpolling=0;
  control_poll_GSA_GSV(0);
#else
  // control_poll will disable GSA and GSV on its own
  push_gps_message(UBLOX_GxGSA_OFF, sizeof(UBLOX_GxGSA_OFF));
  push_gps_message(UBLOX_GxGSV_OFF, sizeof(UBLOX_GxGSV_OFF));
#endif
  push_gps_message(UBLOX_GxGBS_ON, sizeof(UBLOX_GxGBS_ON));
  stored_preferences.nmeaGBS = true;
  push_gps_message(UBLOX_INIT_CHANNEL_ALL, sizeof(UBLOX_INIT_CHANNEL_ALL));
  log_i("Set optimal configuration for RaceChrono on Android");
  stored_preferences.trackaddict = false;
#ifdef BLEENABLED
  stored_preferences.ble_active = false;
  ble_stop();
#endif
  bt_spp_start();
  stored_preferences.btspp_active = true;
  webserver.send(200, html_text, generate_html_body(F("Set optimal configuration for RaceChrono on Android:</p><p><ul><li>All SV's</li><li>GSA+GSV off</li><li>GBS On</li><li>Updates at 10 Hz</li><li>BT-SPP connection</li></ul></p><p><a href='/save_config/withoutwifi'>Save</a> these settings</p>")));
}
void handle_hlt_android() {
  // /hlt/android
  log_i("Setting optimal configuration for Harry Lap Timer on Android: 10Hz, GSA+GSV Off, GSA+GSV polled on a 5 sec cycle, GBS On, BT-SPP");
  push_gps_message(UBLOX_INIT_10HZ, sizeof(UBLOX_INIT_10HZ));
  stored_preferences.gps_rate = 10;
  stored_preferences.nmeaGSA = false;
  stored_preferences.nmeaGSV = false;
#ifdef TASK_SCHEDULER
  stored_preferences.nmeaGSAGSVpolling=5;
  control_poll_GSA_GSV(5);
#else
  // control_poll will disable GSA and GSV on its own
  push_gps_message(UBLOX_GxGSA_OFF, sizeof(UBLOX_GxGSA_OFF));
  push_gps_message(UBLOX_GxGSV_OFF, sizeof(UBLOX_GxGSV_OFF));
#endif
  push_gps_message(UBLOX_GxGBS_ON, sizeof(UBLOX_GxGBS_ON));
  stored_preferences.nmeaGBS = true;
  push_gps_message(UBLOX_INIT_CHANNEL_ALL, sizeof(UBLOX_INIT_CHANNEL_ALL));
  log_i("Set optimal configuration for Harry Lap Timer or RaceChrono on Android");
  stored_preferences.trackaddict = false;
#ifdef BLEENABLED
  stored_preferences.ble_active = false;
  ble_stop();
#endif
  bt_spp_start();
  stored_preferences.btspp_active = true;
  webserver.send(200, html_text, generate_html_body(F("Set optimal configuration for Harry Lap Timer or RaceChrono on Android:</p><p><ul><li>Al SV's</li><li>GBS On</li><li>GSA+GSV stream off</li><li>GSA+GSV polled on a 5 sec cycle</li><li>Updates at 10 Hz</li><li>BT-SPP connection</li></ul></p><p><a href='/save_config/withoutwifi'>Save</a> these settings</p>")));
}
#endif
#ifdef BLEENABLED
void handle_hlt_ios() {
  // /hlt/ios
  log_i("Setting optimal configuration for Harry Lap Timer on iOS devices: 10Hz, GSA+GSV Off, GBS On, BLE");
  push_gps_message(UBLOX_INIT_10HZ, sizeof(UBLOX_INIT_10HZ));
  stored_preferences.gps_rate = 10;
  stored_preferences.nmeaGSA = false;
  stored_preferences.nmeaGSV = false;
#ifdef TASK_SCHEDULER
  stored_preferences.nmeaGSAGSVpolling=5;
  control_poll_GSA_GSV(5);
#else
  // control_poll will disable GSA and GSV on its own
  push_gps_message(UBLOX_GxGSA_OFF, sizeof(UBLOX_GxGSA_OFF));
  push_gps_message(UBLOX_GxGSV_OFF, sizeof(UBLOX_GxGSV_OFF));
#endif
  push_gps_message(UBLOX_GxGBS_ON, sizeof(UBLOX_GxGBS_ON));
  stored_preferences.nmeaGBS = true;
  push_gps_message(UBLOX_INIT_CHANNEL_ALL, sizeof(UBLOX_INIT_CHANNEL_ALL));
  stored_preferences.trackaddict = false;
  log_i("Set optimal configuration for Harry Lap Timer on iOS");
#ifdef BTSPPENABLED
  if (stored_preferences.btspp_active) {
      // Hack: since disabling spp and enabling BLE crashes the stack and restart ESP32, we store the current configuration for a restart
      stored_preferences.btspp_active = false;
      stored_preferences.ble_active = true;
      StoreNVMPreferences(true);
      webserver.send(200, html_text, generate_html_body(F("<b>RESTARTING</b> to set optimal configuration for Harry Lap Timer on iOS devices:</p><p><ul><li>All SV's</li><li>GBS On</li><li>GSA+GSV stream off</li><li>GSA+GSV polled on a 5 sec cycle</li><li>Updates at 10 Hz</li><li>BLE connection</li></ul></p><p>Settings have been already saved</p>")));
      delay(1000);
      ESP.restart();
      // This is how it should be:
      // stored_preferences.btspp_active = false;
      // bt_spp_stop();
  }
#endif
  if (stored_preferences.ble_active == false) {
    ble_start();
    stored_preferences.ble_active = true;
  }
  webserver.send(200, html_text, generate_html_body(F("Set optimal configuration for Harry Lap Timer on iOS devices:</p><p><ul><li>All SV's</li><li>GBS On</li><li>GSA+GSV stream off</li><li>GSA+GSV polled on a 5 sec cycle</li><li>Updates at 10 Hz</li><li>BLE connection</li></ul></p><p>Settings have been already saved</p>")));
}
#endif
void handle_wifi_sta() {
  log_i("Start WiFi in STA mode");
  webserver.send(200, html_text, generate_html_body(F("WiFi STATION mode started, trying to connect to a know Access Point. <br>Reconnect in a few seconds")));
  stored_preferences.wifi_mode = WIFI_STA;
  wifi_STA();
}
void handle_wifi_ap() {
  log_i("Start WiFi in AP mode");
  webserver.send(200, html_text, generate_html_body(String("WiFi Access Point mode started. <br>Reconnect in a few seconds to AP:" + String(ap_ssid))));
  stored_preferences.wifi_mode = WIFI_AP;
  wifi_AP();
}
void handle_restart() {
  webserver.send(200, html_text, generate_html_body(F("Please confirm <form action='/restart_execute' method='post'><input type='submit' value='Restart'></form>")));
}
void handle_restart_execute() {
  log_i("Restarting");
  webserver.send(200, html_text, generate_html_body(F("Restarting device")));
  delay(1000);
  ESP.restart();
  delay(1000);
}
void handle_status() {
  log_d("Logging status to web");
  webserver.send(200, html_text, generate_html_body(
                   String("<h1>Device status</h1><p>WiFi Mode: ")
                   + ( (stored_preferences.wifi_mode == WIFI_AP) ? String("AP<br>Access point name: ") + String(ap_ssid) : String("STA<br>Connected to AP: ") + WiFi.SSID() )
                   + String("<br>IP: ") + MyIP.toString()
                   + String("<br>TCP Port for NMEA repeater: ") + String(NMEAServerPort)
                   + String("<br>u-center URL: tcp://") + MyIP.toString() +String(":")+ String(NMEAServerPort)
#ifdef NUMERICAL_BROADCAST_PROTOCOL
                   + String("<br>Port for NBP: ") + String(NBPServerPort)
#else
                   + String("<p>NBP n/a in this firmware ")
#endif
#ifdef BLEENABLED
                   + ( stored_preferences.ble_active ? String("<p>BLE Enabled") : String("<p>BLE Not Enabled") )
                   + String("<br>BLE Name: ") + String(ble_device_id)
                   + String("<br>BLE Service: ") + String(BLE_SERVICE_UUID, HEX)
                   + String("<br>BLE Characteristic: ") + String(BLE_CHARACTERISTIC_UUID, HEX)
#else
                   + String("<p>BLE n/a in this firmware ")
#endif
#ifdef BTSPPENABLED
                   + ( stored_preferences.btspp_active ? String("<p>BT-SPP Enabled") : String("<p>BT-SPP Not Enabled") )
                   + String("<br>BT Name: ") + String(ble_device_id)
#else
                   + String("<p>BT-SPP n/a in this firmware ")
#endif
                   + String("<p>UART Baud: ") + stored_preferences.gps_baud_rate
                   + String("<br>Refresh rate: ") + stored_preferences.gps_rate + String( "Hz")
                   + String("<br>Main Talker ID: ") + ( stored_preferences.trackaddict ? String("GPS (TrackAddict)") : String("System Specific (default)") )
                   + String("<br>NMEA GxGSV: ") + ( stored_preferences.nmeaGSV ? String("ON") : String("OFF") )
                   + String("<br>NMEA GxGSA: ") + ( stored_preferences.nmeaGSA ? String("ON") : String("OFF") )
                   + String("<br>NMEA GxGBS: ") + ( stored_preferences.nmeaGBS ? String("ON") : String("OFF") )
                   + String("<br>GPS PowerSave: ") + ( gps_powersave ? String(" ON"):String("OFF"))
#ifdef UPTIME
                   + String("<br><br>Uptime: ") + String(uptime_formatter::getUptime())
#endif
                   + String("<br>Version: ") + generate_version()
                   + String("<br>Build date: ") + String(BONOGPS_BUILD_DATE) 
                   + String("<p>Total heap: ") + String(ESP.getHeapSize())
                   + String("<br>Free heap: ") + String(ESP.getFreeHeap())
                   + String("<br>Total PSRAM: ") + String(ESP.getPsramSize())
                   + String("<br>Free PSRAM: ") + String(ESP.getFreePsram())
                 ));
}
void handle_clients() {
  log_i("Logging clients status to web");
  webserver.send(200, html_text, generate_html_body(
                   ( NMEARemoteClient.connected() ? String("NMEA TCP/IP client: ") + NMEARemoteClient.remoteIP().toString() : String("NMEA TCP/IP client not connected") )
                   + String("<br>")
#ifdef NUMERICAL_BROADCAST_PROTOCOL
                   + ( NBPRemoteClient.connected() ? String("NBP TCP/IP client: ") + NBPRemoteClient.remoteIP().toString() : String("NBP TCP/IP client not connected") )
                   + String("<br>")
#endif
#ifdef BLEENABLED
                   + ( ble_deviceConnected ? String("BLE device: ") + ble_client : String("BLE device not connected") )
                   + String("<br>")
#endif
#ifdef BTSPPENABLED
                   + ( bt_deviceConnected ? String("BT-SPP device connected (address not yet implemented)") : String("BT-SPP device not connected") )
#endif
                 ));
}
void handle_resetapconfig() {
#ifdef WIFIMANAGER
  log_i("Resetting AP Configuration for WiFiManager");
  webserver.send(200, html_text, generate_html_body("WiFi configuration reset - restarting device"));
  WiFiManager wm;
  //reset settings - for testing
  wm.resetSettings();
  delay(1000);
  ESP.restart();
  delay(1000);
#else
  log_i("To Be Implemented Resetting AP Configuration");
  webserver.send(200, html_text, generate_html_body("To Be Implemented - WiFi configuration reset"));
#endif
}
void handle_saveconfig() {
  log_i("Storing preferences intermediate menu");
  webserver.send(200, html_text, generate_html_body(
  String(F("<h1>All configuration values</h1><p>Save <a href='/save_config/withwifi'>all config <b>including </b> WiFi </a></p>"))
  +String(F("<p>Save <a href='/save_config/withoutwifi'>all config except for "))
  + ( (stored_preferences.wifi_mode == WIFI_AP) ? String("WiFi-AP ") + String(ap_ssid) : String("WiFi-STA ") )
  +String(F("mode</a></p>\n<h1>Save only a specific WiFi mode</h1><p>Save <a href='/save_config/wifi/sta'>WiFi client mode WIFI-STA </a></p><p>Save <a href='/save_config/wifi/ap'>WiFi Access Point mode WIFI-AP</a></p>"))));
}
void handle_saveconfig_withwifi() {
  StoreNVMPreferences(true);
  log_i("Storing preferences including WiFi");
  webserver.send(200, html_text, generate_html_body(F("Preferences including WiFi stored")));
}
void handle_saveconfig_withoutwifi() {
  StoreNVMPreferences(false);
  log_i("Storing preferences excluding WiFi");
  webserver.send(200, html_text, generate_html_body(F("Preferences excluding WiFi stored")));
}
void handle_saveconfig_wifi_sta() {
  StoreNVMPreferencesWiFi("WIFI_STA");
  log_i("Storing preference WIFI_STA");
  webserver.send(200, html_text, generate_html_body(F("Stored preference WIFI_STA as default: this will be used at next restart")));
}
void handle_saveconfig_wifi_ap() {
  StoreNVMPreferencesWiFi("WIFI_AP");
  log_i("Storing preference WIFI_AP");
  webserver.send(200, html_text, generate_html_body(F("Stored preference WIFI_AP as default: this will be used at next restart")));
}
void handle_NotFound() {
  webserver.send(404, html_text, generate_html_body(F("Not found")));
}
void WebConfig_start() {
  webserver.on("/", handle_menu);
  webserver.on("/rate/1hz", handle_rate_1hz);
  webserver.on("/rate/5hz", handle_rate_5hz);
  webserver.on("/rate/10hz", handle_rate_10hz);
  webserver.on("/gsa/on", handle_gsa_on);
  webserver.on("/gsa/off", handle_gsa_off);
  webserver.on("/gsv/on", handle_gsv_on);
  webserver.on("/gsv/off", handle_gsv_off);
  webserver.on("/gbs/on", handle_gbs_on);
  webserver.on("/gbs/off", handle_gbs_off);
  webserver.on("/sv/8", handle_svchannel_8);
  webserver.on("/sv/all", handle_svchannel_all);
  webserver.on("/hlt", handle_hlt);
#ifdef BTSPPENABLED
  webserver.on("/hlt/android", handle_hlt_android);
  webserver.on("/trackaddict", handle_trackaddict);
  webserver.on("/trackaddict/on", handle_trackaddict_on);
  webserver.on("/trackaddict/off", handle_trackaddict_off);
  webserver.on("/racechrono", handle_racechrono);
  webserver.on("/racechrono/android", handle_racechrono_android);
#endif
#ifdef BLEENABLED
  webserver.on("/hlt/ios", handle_hlt_ios);
#endif
  webserver.on("/wifi/sta", handle_wifi_sta);
  webserver.on("/wifi/ap", handle_wifi_ap);
  webserver.on("/restart", handle_restart);
  webserver.on("/restart_execute", handle_restart_execute);
  webserver.on("/reset_wifi", handle_resetapconfig);
  webserver.on("/save_config", handle_saveconfig);
  webserver.on("/save_config/withwifi", handle_saveconfig_withwifi);
  webserver.on("/save_config/withoutwifi", handle_saveconfig_withoutwifi);
  webserver.on("/save_config/wifi/sta", handle_saveconfig_wifi_sta);
  webserver.on("/save_config/wifi/ap", handle_saveconfig_wifi_ap);
  webserver.on("/powersave/3600",handle_powersave_1hr);
  webserver.on("/powersave/1800",handle_powersave_30min);
  webserver.on("/status", handle_status);
  webserver.on("/clients", handle_clients);
  webserver.on("/baud/38400", handle_baudrate_38400);
  webserver.on("/baud/57600", handle_baudrate_57600);
  webserver.on("/baud/115200", handle_baudrate_115200);
#ifdef BLEENABLED
  webserver.on("/ble/on", handle_ble_on);
  webserver.on("/ble/off", handle_ble_off);
#endif
#ifdef BTSPPENABLED
  webserver.on("/btspp/on", handle_btspp_on);
  webserver.on("/btspp/off", handle_btspp_off);
#endif
#ifdef TASK_SCHEDULER
  webserver.on("/poll/gsagsv/0", handle_pollgsagsv_off);
  webserver.on("/poll/gsagsv/1", handle_pollgsagsv_on_1);
  webserver.on("/poll/gsagsv/5", handle_pollgsagsv_on_5);
#endif
  webserver.onNotFound(handle_NotFound);
  webserver.begin();
#ifdef MDNS_ENABLE
  MDNS.addService("http", "tcp", 80);
#endif
}

/********************************

  OTA

* ******************************/
#ifdef ENABLE_OTA
#ifndef OTA_AVAILABLE
#define OTA_AVAILABLE 300 // 300 seconds of OTA running
#endif

#ifdef TASK_SCHEDULER
Task tOTA (1000, OTA_AVAILABLE , &handle_OTA, &ts, false );
#endif

void handle_OTA() {
  ArduinoOTA.handle();
}
void OTA_start() {
  ArduinoOTA.setHostname(BONOGPS_MDNS);
  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    log_i("Start updating %s", type);
  })
  .onEnd([]() {
    log_i("\nEnd");
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    SerialMonitor.printf("Progress: %u%%\r", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    log_e("Error[ %u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  tOTA.enable();
}
#endif

/********************************

  BLE

* ******************************/

#ifdef BLEENABLED

#define DEVINFO_UUID              (uint16_t)0x180a
#define DEVINFO_MANUFACTURER_UUID (uint16_t)0x2a29 //0x2A29 MfgName utf8s
#define DEVINFO_NAME_UUID         (uint16_t)0x2a24 //0x2A24 ModelNum utf8s
#define DEVINFO_SERIAL_UUID       (uint16_t)0x2a25 //0x2A25 SerialNum utf8s
#define DEVINFO_FWREV_UUID        (uint16_t)0x2a26 //0x2A26 FirmwareRev utf8s

#define SERVICE_BATTERY_SERVICE_UUID              (uint16_t)0x180F // not used
#define CHARACTERISTIC_BATTERY_LEVEL_UUID         (uint16_t)0x2A19 // not used
#define CHARACTERISTIC_BATTERY_LEVEL_STATE_UUID   (uint16_t)0x2A1B // not used
#define CHARACTERISTIC_BATTERY_POWER_STATE_UUID   (uint16_t)0x2A1A // not used
//#define HardwareRev "0001"    //0x2A27 utf8s
//#define SystemID "000001"     //0x2A23 uint40

NimBLEServer* pServer = NULL;
NimBLECharacteristic* pCharacteristic = NULL;
uint16_t ble_mtu;

// Build BTLE messages here
uint8_t gps_currentmessage[MAX_UART_BUFFER_SIZE] ; // hold current buffer
uint8_t gps_currentchar = '$'; // hold current char
int gps_message_pointer = 0; //pointer

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
      ble_deviceConnected = true;
      ble_client = String(NimBLEAddress(desc->peer_ota_addr).toString().c_str());
      log_i("BLE Client address: %s", ble_client);
    };
    void onDisconnect(NimBLEServer* pServer) {
      ble_deviceConnected = false;
    }
};

void ble_start() {
#ifdef BTSPPENABLED
  bt_spp_stop();
#endif
  /// Create the BLE Device
  NimBLEDevice::init(ble_device_id);
  /** Optional: set the transmit power, default is 3db */
  NimBLEDevice::setPower(ESP_PWR_LVL_P9); /** +9db */
  NimBLEDevice::setPower(ESP_PWR_LVL_P9, ESP_BLE_PWR_TYPE_ADV); /** +9db */
  NimBLEDevice::setPower(ESP_PWR_LVL_P9, ESP_BLE_PWR_TYPE_CONN_HDL0); /** +9db */
  NimBLEDevice::setPower(ESP_PWR_LVL_P9, ESP_BLE_PWR_TYPE_CONN_HDL1); /** +9db */
  NimBLEDevice::setPower(ESP_PWR_LVL_P9, ESP_BLE_PWR_TYPE_CONN_HDL2); /** +9db */
  // Set the MTU to a larger value than standard 23
  NimBLEDevice::setMTU(BLE_MTU);
  ble_mtu = NimBLEDevice::getMTU();
  log_d("BLE MTU set to: %d", ble_mtu);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  NimBLEDevice::setSecurityAuth(/*BLE_SM_PAIR_AUTHREQ_BOND | BLE_SM_PAIR_AUTHREQ_MITM |*/ BLE_SM_PAIR_AUTHREQ_SC);
  // Create the BLE Server
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  // Create the BLE Service
  NimBLEService *pService = pServer->createService(BLE_SERVICE_UUID);
  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(BLE_CHARACTERISTIC_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  // Start the service
  pService->start();
  // other service with info
  pService = pServer->createService(DEVINFO_UUID);
  BLECharacteristic *pChar = pService->createCharacteristic(DEVINFO_MANUFACTURER_UUID, NIMBLE_PROPERTY::READ);
  pChar->setValue(DEVICE_MANUFACTURER);
  pChar = pService->createCharacteristic(DEVINFO_NAME_UUID, NIMBLE_PROPERTY::READ);
  pChar->setValue(DEVICE_NAME);
  pChar = pService->createCharacteristic(DEVINFO_SERIAL_UUID, NIMBLE_PROPERTY::READ);
  pChar->setValue(chip);
  pChar = pService->createCharacteristic(DEVINFO_FWREV_UUID, NIMBLE_PROPERTY::READ);
  pChar->setValue(generate_version());
  pService->start();
  // Start advertising
  NimBLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(pService->getUUID());
  // define appearance, from https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.gap.appearance.xml
  pAdvertising->setAppearance(5186);
  pAdvertising->setScanResponse(false);
  // pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue -> not for NimBLE
  /**This method is removed as it was no longer useful and consumed advertising space
    pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  */
  // NimBLEDevice::startAdvertising();
  pAdvertising->start();
  log_i("Waiting a client connection to notify...");
}

void ble_stop() {
  if (NimBLEDevice::getInitialized()) {
    NimBLEDevice::stopAdvertising();
    NimBLEDevice::deinit(true);
  }
}
#endif

#ifdef BTSPPENABLED
/*********************************************************************

  Bluetooth Classic

*******************************************************************/
void bt_spp_start() {
#ifdef BLEENABLED
  // disable BTLE
  ble_stop();
#endif
  SerialBT.begin(ble_device_id); //Bluetooth device name
  SerialBT.register_callback(bt_callback);
  log_i("Started BT-SPP port");
}
void bt_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    log_d("BT-SPP Client Connected");
#ifdef BLEENABLED
    ble_deviceConnected = false;
#endif
    bt_deviceConnected = true;
  }
  if (event == ESP_SPP_CLOSE_EVT) {
    log_d("BT-SPP Client closed connection");
    bt_deviceConnected = false;
  }
}
void bt_spp_stop() {
  SerialBT.end();
}
#endif



/*********************************************************************

  SETUP

*******************************************************************/
void gps_initialize_settings() {
    // GPS Connection
  // if preferences have a value <> than the one stored in the GPS, connect+switch
  if (GPS_STANDARD_BAUD_RATE == stored_preferences.gps_baud_rate) {
    log_i("Connecting to GPS at standard %d", stored_preferences.gps_baud_rate);
    gpsPort.begin(stored_preferences.gps_baud_rate);
  } else {
    log_i("Connecting to GPS at standard %d", GPS_STANDARD_BAUD_RATE);
    gpsPort.begin(GPS_STANDARD_BAUD_RATE);
    log_i("Re-Connecting to GPS at updated %d", stored_preferences.gps_baud_rate);
    switch_baudrate(stored_preferences.gps_baud_rate);
  }

  gpsPort.setRxBufferSize(UART_BUFFER_SIZE_RX);
  delay(50);

  if (stored_preferences.nmeaGSV) {
    push_gps_message(UBLOX_GxGSV_ON, sizeof(UBLOX_GxGSV_ON));
  } else {
    push_gps_message(UBLOX_GxGSV_OFF, sizeof(UBLOX_GxGSV_OFF));
  }

  if (stored_preferences.nmeaGSA) {
    push_gps_message(UBLOX_GxGSA_ON, sizeof(UBLOX_GxGSA_ON));
  } else {
    push_gps_message(UBLOX_GxGSA_OFF, sizeof(UBLOX_GxGSA_OFF));
  }

  if (stored_preferences.nmeaGBS) {
    push_gps_message(UBLOX_GxGBS_ON, sizeof(UBLOX_GxGBS_ON));
  } else {
    push_gps_message(UBLOX_GxGBS_OFF, sizeof(UBLOX_GxGBS_OFF));
  }

#ifdef BTSPPENABLED
  // apply settings required by TrackAddict
  if (stored_preferences.trackaddict) {
    gps_enable_trackaddict();
  } else {
    push_gps_message(UBLOX_INIT_CHANNEL_ALL, sizeof(UBLOX_INIT_CHANNEL_ALL));
  }
#endif

  switch (stored_preferences.gps_rate) {
    case 1:
      push_gps_message(UBLOX_INIT_1HZ, sizeof(UBLOX_INIT_1HZ));
      break;
    case 5:
      push_gps_message(UBLOX_INIT_5HZ, sizeof(UBLOX_INIT_5HZ));
      break;
    case 10:
      push_gps_message(UBLOX_INIT_10HZ, sizeof(UBLOX_INIT_10HZ));
      break;
    default:
      push_gps_message(UBLOX_INIT_5HZ, sizeof(UBLOX_INIT_5HZ));
      break;
  }
#ifdef TASK_SCHEDULER  
  control_poll_GSA_GSV(stored_preferences.nmeaGSAGSVpolling);
#endif 
}

void setup() {

  SerialMonitor.begin(LOG_BAUD_RATE); // Initialize the serial monitor
  delay(500);
  log_d("ESP32 SDK: %s", ESP.getSdkVersion());
  log_d("Arduino sketch: %s", __FILE__);
  log_d("Compiled on: %s", __DATE__);

  // Generate device name
  chip = (uint16_t)((uint64_t) ESP.getEfuseMac() >> 32);
  sprintf(ap_ssid, "%s-%04X", BONOGPS_AP, chip);
#if defined(BLEENABLED) || defined(BTSPPENABLED)
  sprintf(ble_device_id, "%s-%04X", BLE_DEVICE_ID, chip);
#endif

  log_d("Total heap: %d", ESP.getHeapSize());
  log_d("Free heap: %d", ESP.getFreeHeap());
  log_d("Total PSRAM: %d", ESP.getPsramSize());
  log_d("Free PSRAM: %d", ESP.getFreePsram());

  // Read desired status from NVM
  prefs.begin(BONOGPS_MDNS);
  ReadNVMPreferences();

  if (stored_preferences.ble_active) {
    // start BTLE
#ifdef BLEENABLED
    ble_start();
#endif
#ifdef BTSPPENABLED
  } else if (stored_preferences.btspp_active) {
    // else prevents BT-SPP and BLE to start at the same time which crashes the Bluetooth stacks
    // start BTSPP
    bt_spp_start();
#endif
  }

#ifdef BUTTON
  button.begin();
#endif

  // Start WiFi
  WiFi.setHostname(BONOGPS_MDNS);
  WiFi.setSleep(false);

  switch (stored_preferences.wifi_mode) {
    case WIFI_AP:
      wifi_AP();
      break;
    case WIFI_STA:
      wifi_STA();
      break;
    default:
      wifi_STA();
      break;
  }

  gps_initialize_settings();

}

void loop() {
  // check UART for data
  if (gpsPort.available()) {
    size_t len = gpsPort.available();
    uint8_t sbuf[len];
    gpsPort.readBytes(sbuf, len);
    // log max buffer size
    if (len > max_buffer) {
      max_buffer = len;
      log_w("New max buffer: %d", max_buffer);
    }
    if (wifi_connected && NMEARemoteClient && NMEARemoteClient.connected()) {
      //push UART data to the TCP client
      NMEARemoteClient.write(sbuf, len);
    }

#ifdef NUMERICAL_BROADCAST_PROTOCOL
    if (wifi_connected && NBPRemoteClient && NBPRemoteClient.connected()) {
      // send GPS data to NeoGPS processor for parsing
      for (int i = 0; i < len ; i++) {
        if (gps.decode((char) sbuf[i] ) == NMEAGPS::DECODE_COMPLETED) {
          log_d("Fix available, size: %d", sizeof(gps.fix()));
          my_fix = gps.fix();

          NBPRemoteClient.print("*NBP1,ALLUPDATE,");
          NBPRemoteClient.print(my_fix.dateTime);
          NBPRemoteClient.print(".");
          NBPRemoteClient.println(my_fix.dateTime_ms());
          if (my_fix.valid.location) {
            NBPRemoteClient.print("\"Latitude\",\"deg\":");
            NBPRemoteClient.println(my_fix.latitude(), 12);
            NBPRemoteClient.print("\"Longitude\",\"deg\":");
            NBPRemoteClient.println(my_fix.longitude(), 12);
            NBPRemoteClient.print("\"Altitude\",\"m\":");
            NBPRemoteClient.println(my_fix.altitude());
          }
          if (my_fix.valid.heading) {
            NBPRemoteClient.print("\"Heading\",\"deg\":");
            NBPRemoteClient.println(my_fix.heading());
          }
          if (my_fix.valid.speed) {
            NBPRemoteClient.print("\"Speed\",\"kmh\":");
            NBPRemoteClient.println(my_fix.speed_kph());
          }
          // NBPRemoteClient.print("\"Accuracy\",\"m\":");
          NBPRemoteClient.println("#");
          NBPRemoteClient.print("@NAME:");
          NBPRemoteClient.println(ap_ssid);
        }
      }
    } else {
      NBPCheckForConnections();
    }
#endif

#ifdef BTSPPENABLED
    if (stored_preferences.btspp_active && bt_deviceConnected) {
      // we have BT-SPP active
      SerialBT.write(sbuf, len);
    }
#endif

#ifdef BLEENABLED
    if (stored_preferences.ble_active && ble_deviceConnected) {
      // we have BLE active
      for (int i = 0; i < len ; i++) {
        if (gps_message_pointer > MAX_UART_BUFFER_SIZE - 3) {
          log_e("BLE Buffer saturated, resetting ");
          gps_currentmessage[0] = '$';
          gps_currentmessage[1] = '\0';
          gps_message_pointer = 1;
        }
        if (sbuf[i] == '$' ) {
          gps_currentmessage[gps_message_pointer++] = '\r';
          gps_currentmessage[gps_message_pointer++] = '\n';
          gps_currentmessage[gps_message_pointer] = '\0';
          // There is a new message -> Notify BLE of the changed value if connected and size says it is likely to be valid
          if (gps_message_pointer > MIN_NMEA_MESSAGE_SIZE) {
            pCharacteristic->setValue(gps_currentmessage);
            pCharacteristic->notify();
            delay(1);
          }
          gps_currentmessage[0] = '$';
          gps_currentmessage[1] = '\0';
          gps_message_pointer = 1;
        } else {
          // Accumulate more of the GPS message
          gps_currentmessage[gps_message_pointer++] = sbuf[i];
        }
      }
    }
#endif

  }

  if (wifi_connected) {
    // handle sending instructions to GPS via uBlox center connected with TCP/IP
    if ( NMEARemoteClient.connected() && NMEARemoteClient.available()) {
      gpsPort.write(NMEARemoteClient.read());
    } else {
      NMEACheckForConnections();
    }

    // TODO - impove handling of webserver so that it has a lower priority or spin a separate task
    webserver.handleClient();
  }

#ifdef BUTTON
  // Continuously read the status of the button.
  button.read();
#endif

#ifdef TASK_SCHEDULER
  // run all periodic tasks
  ts.execute();
#endif
}
