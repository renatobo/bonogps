/******************************************************************************

  BonoGPS: connect a GPS to mobile Apps to track lap times
  More info at https://github.com/renatobo/bonogps
  Renato Bonomini https://github.com/renatobo

******************************************************************************/

// For PlatformIO we need to include the Arduino framework
#include <Arduino.h>
// load PINout definitions from this header file
#include <bonogps_board_settings.h>

// Enable or disable compiling features
#define UPTIME       // add library to display for how long the device has been running
#define BTSPPENABLED // add BT-SPP stack, remove if unnecessary as it uses quite a bit of flash space
#define BLEENABLED   // add BLE stack, remove if unnecessary as it uses quite a bit of flash space
// #define ENABLE_OTA     // add code for OTA, to be enabled only when developing
#define MDNS_ENABLE    // Enable or disable mDNS - currently not working in all conditions. Small memory save to be worth removing
#define TASK_SCHEDULER // enable adv task scheduler. Small memory save to be worth removing
#define BUTTON         // Enable changing WIFI_STA and WIFI_AP via Boot button. Small memory save to be worth removing - Which button is defined below
#define SHORT_API      // URLs for commands such as enable/disable/change rate respond with a short json answer instead of a full web page

// Configure names and PIN's used for interfacing with external world
#define DEVICE_MANUFACTURER "https://github.com/renatobo"
#define DEVICE_NAME "Bono GPS"
// BONOGPS_FIRMWARE_VER is used in BLE "Firmware" data and in the /status page of web configuration
#define BONOGPS_BUILD_DATE __TIMESTAMP__
#ifdef GIT_REV
#define BONOGPS_FIRMWARE_VER GIT_REV
#else
// the following define is needed to display version when building this with the Arduino IDE
#define BONOGPS_FIRMWARE_VER "v1.1beta"
#endif
// Bonjour DNS name, access the GPS configuration page by appending .local as DNS
#define BONOGPS_MDNS "bonogps"
// Prefix for the AP name, followed by 4 digits device id
#define BONOGPS_AP "BonoGPS"
#define BONOGPS_PWD "bono5678"
// the AP name is generated on the fly, this is its maximum size
#define MAX_AP_NAME_SIZE 20

// How large should the RX buffer for the GPS port - more than 512 creates issues
#define UART_BUFFER_SIZE_RX 256
// Size of the intermediate buffer where we are storing the current sentence
#define MAX_UART_BUFFER_SIZE 256

// TCP Port for NMEA sentences repeater, used for Harry's LapTimer mostly, but also for proxying with uBlox
#define NMEA_TCP_PORT 1818
// Disabling GPS over NBP as it does not override phone data within TrackAddict right now, so kind of useless mostly
// #define NUMERICAL_BROADCAST_PROTOCOL
#ifdef NUMERICAL_BROADCAST_PROTOCOL
#define NEED_NEOGPS
#define NBP_TCP_PORT 35000
#endif

// Define configuration of Bluetooth Low Energy
#define BLE_DEVICE_ID "BonoGPS"
#define BLE_SERVICE_UUID (uint16_t)0x1819        // SERVICE_LOCATION_AND_NAVIGATION_UUID
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

/********************************
 * 
 * nothing you should need to configure from here below
 * 
********************************/

#if !(defined(ESP32))
#error This code is designed to run only on the ESP32 board
#endif

#if !(defined(LED_BUILTIN))
#error "You are missing the definition of LED_BUILTIN for your board or in bonogps_board_settings.h"
#endif
#if !(defined(WIFI_MODE_BUTTON))
#error "You are missing the definition of WIFI_MODE_BUTTON in bonogps_board_settings.h"
#endif
#if !(defined(RX2) && defined(TX2))
#error "You are missing definitions of Serial 2 PINS: select a board that has them already defined or change bonogps_board_settings.h"
#endif

// Library to store desidered preferences in NVM
#include <Preferences.h>
Preferences prefs;
// data to be stored
#include "esp_wifi.h"
#include <WiFi.h>

#define WIFI_KEY_MAXLEN (int)64
#define WIFI_SSID_MAXLEN (int)32

typedef struct
{
  unsigned long gps_baud_rate = GPS_STANDARD_BAUD_RATE; // initial or current baud rate
  uint8_t gps_rate = 5;
  WiFiMode_t wifi_mode = WIFI_AP;
  bool nmeaGSA = false;
  bool nmeaGSV = false;
  bool nmeaVTG = false;
  bool nmeaGLL = false;
  bool nmeaGBS = true;
  bool nmeaTcpServer = false;
  bool ble_active = true;
  bool btspp_active = false;
  bool trackaddict = false;
  bool racechrono = false;
  bool racetime = false;
  uint8_t nmeaGSAGSVpolling = 0;
  char wifi_ssid[WIFI_SSID_MAXLEN];
  char wifi_key[WIFI_KEY_MAXLEN];
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
#if !defined(GPS_FIX_TIME)
#error You must define GPS_FIX_TIME in GPSfix_cfg.h!
#endif
#if !defined(GPS_FIX_LOCATION)
#error You must uncomment GPS_FIX_LOCATION in GPSfix_cfg.h!
#endif
#if !defined(GPS_FIX_SPEED)
#error You must uncomment GPS_FIX_SPEED in GPSfix_cfg.h!
#endif
#if !defined(GPS_FIX_ALTITUDE)
#error You must uncomment GPS_FIX_ALTITUDE in GPSfix_cfg.h!
#endif
NMEAGPS gps;
gps_fix my_fix;
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
bool bt_spp_stop();
void bt_spp_start();
void bt_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
#endif

#ifdef BLEENABLED
// BLE stack
// Only used from NimbleArduino librray > 1.0.2, commit https://github.com/h2zero/NimBLE-Arduino/commit/569eb8a188c78fe780f4c2a24cf9247532cf55ea
#define CONFIG_BT_NIMBLE_ROLE_CENTRAL_DISABLED
#define CONFIG_BT_NIMBLE_ROLE_OBSERVER_DISABLED
#define CONFIG_BT_NIMBLE_MAX_CONNECTIONS 2
#define CONFIG_BT_NIMBLE_MAX_BONDS 2
#define CONFIG_BT_NIMBLE_SVC_GAP_DEVICE_NAME BONOGPS_MDNS
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
#define EASYBUTTON_DO_NOT_USE_SEQUENCES
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
void WebConfig_stop();
void gps_initialize_settings();
void restart_after_sleep();
void wifi_AP();
void wifi_OFF();
#ifdef ENABLE_OTA
void handle_OTA();
void OTA_start();
void OTA_stop();
#endif

/********************************

   Utilities

 * ******************************/
#ifdef BUILD_ENV_NAME
#define BONO_GPS_VERSION BONOGPS_FIRMWARE_VER " [" BUILD_ENV_NAME "]"
#else
#define BONO_GPS_VERSION BONOGPS_FIRMWARE_VER " [Arduino]"
#endif
/********************************

   GPS Settings
   These sentences are used to configure a uBlox series 8 device

 * ******************************/
// set rate of GPS to 5hz
const char UBLOX_INIT_5HZ[] PROGMEM = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
// set rate of GPS to 10Hz
const char UBLOX_INIT_10HZ[] PROGMEM = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};
// set rate of GPS to 1Hz
const char UBLOX_INIT_1HZ[] PROGMEM = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xE8, 0x03, 0x01, 0x00, 0x01, 0x00, 0x01, 0x39};
// set rate of GPS to 1Hz
const char UBLOX_INIT_16HZ[] PROGMEM = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xE8, 0x03, 0x01, 0x00, 0x01, 0x00, 0x01, 0x39};

// GLL_ON
const char UBLOX_GxGLL_ON[] PROGMEM = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x2F};
// GLL OFF
const char UBLOX_GxGLL_OFF[] PROGMEM = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A};

// VTG ON
const char UBLOX_GxVTG_ON[] PROGMEM = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x05, 0x4B};
// VTG OFF
const char UBLOX_GxVTG_OFF[] PROGMEM = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46};

// Enable GxGSA (GNSS DOP and Active Satellites)
const char UBLOX_GxGSA_ON[] PROGMEM = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00, 0x05, 0x41};
// Disable GxGSA
const char UBLOX_GxGSA_OFF[] PROGMEM = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31};
// Enable GxGSV (Sat View)
const char UBLOX_GxGSV_ON[] PROGMEM = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00, 0x06, 0x48};
// Disable GxGSV
const char UBLOX_GxGSV_OFF[] PROGMEM = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38};
// set Max SVs per Talker id to 8
const char UBLOX_INIT_CHANNEL_8[] PROGMEM = {0xB5, 0x62, 0x06, 0x17, 0x14, 0x00, 0x00, 0x41, 0x08, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7D, 0xE7};
// standard max SVs and extended digits for unsupported SVs
const char UBLOX_INIT_CHANNEL_ALL[] PROGMEM = {0xB5, 0x62, 0x06, 0x17, 0x14, 0x00, 0x00, 0x41, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x76, 0x63};
// use GP as Main talker and GSV Talker ID - this is needed for TrackAddict
const char UBLOX_INIT_MAINTALKER_GP[] PROGMEM =  {0xB5, 0x62, 0x06, 0x17, 0x14, 0x00, 0x00, 0x41, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x78};
// use GP as Main talker and GSV Talker ID and restrict to GPS SVs ony - this is needed for RaceChrono
const char UBLOX_INIT_MAINTALKER_GP_GPSONLY[] PROGMEM = {0xB5, 0x62, 0x06, 0x17, 0x14, 0x00, 0x10, 0x41, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x88, 0xB8};
// set gps port BAUD rate
const char UBLOX_BAUD_57600[] PROGMEM = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xE1, 0x00, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDA, 0xA9};
const char UBLOX_BAUD_38400[] PROGMEM = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x96, 0x00, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8F, 0x70};
const char UBLOX_BAUD_115200[] PROGMEM = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBC, 0x5E};
// power saving
// enable power saving mode for 1800 or 3600 seconds
// UBX - RXM - PMREQ request
const char UBLOX_PWR_SAVE_30MIN[] PROGMEM = {0xB5, 0x62, 0x02, 0x41, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x77, 0x1B, 0x00, 0x02, 0x00, 0x00, 0x00, 0xE8, 0x00, 0x00, 0x00, 0x0F, 0xF6};
const char UBLOX_PWR_SAVE_1HR[] PROGMEM = {0xB5, 0x62, 0x02, 0x41, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xEE, 0x36, 0x00, 0x02, 0x00, 0x00, 0x00, 0xE8, 0x00, 0x00, 0x00, 0xE1, 0x21};
//, 0xrestart UBX-CFG-RST with controlled GNSS only software, hotstart (<4 hrs) so that ephemeris still valid
const char UBLOX_WARMSTART[] PROGMEM = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x02, 0x00, 0x10, 0x68};
// Display precision in some apps
const char UBLOX_GxGBS_ON[] PROGMEM = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x09, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00, 0x0C, 0x72};
const char UBLOX_GxGBS_OFF[] PROGMEM = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x62};
// Poll GNGSA
const char UBLOX_GNGSA_POLL[] PROGMEM = "$EIGNQ,GSA*2D\r\n";
// Poll GNGSV
const char UBLOX_GNGSV_POLL[] PROGMEM = "$EIGNQ,GSV*3A\r\n";
// Poll GPGSA
const char UBLOX_GPGSA_POLL[] PROGMEM = "$EIGPQ,GSA*33\r\n";
// Poll GPGSV
const char UBLOX_GPGSV_POLL[] PROGMEM = "$EIGPQ,GSV*24\r\n";
bool gsaorgsv_turn = true;

void push_gps_message(const char message[], int message_size = 0)
{
  if (gps_powersave)
  {
    log_d("Disable powermode");
    gpsPort.end();
    delay(1000);
    // assumes stored_preferences global
    gps_powersave = false;
    gps_initialize_settings();
  }
  for (int i = 0; i < message_size; i++)
  {
    gpsPort.write(message[i]);
  }
}

#ifdef TASK_SCHEDULER
void poll_GSA_GSV_info()
{
  if (gsaorgsv_turn)
  {
    if (stored_preferences.racechrono)
    {
      // RaceChrono only understand GPGSA
      gpsPort.write(UBLOX_GPGSA_POLL);
    }
    else
    {
      gpsPort.write(UBLOX_GNGSA_POLL);
    }
  }
  else
  {
    if (stored_preferences.racechrono)
    {
      // RaceChrono only understand GPGSV
      gpsPort.write(UBLOX_GPGSV_POLL);
    }
    else
    {
      gpsPort.write(UBLOX_GNGSV_POLL);
    }
  }
  gsaorgsv_turn = !gsaorgsv_turn;
}

Task tpoll_GSA_GSV_info(0, TASK_FOREVER, poll_GSA_GSV_info, &ts, false);
void control_poll_GSA_GSV(int frequency)
{
  stored_preferences.nmeaGSAGSVpolling = frequency;
  if (frequency > 0)
  {
    push_gps_message(UBLOX_GxGSV_OFF, sizeof(UBLOX_GxGSV_OFF));
    push_gps_message(UBLOX_GxGSA_OFF, sizeof(UBLOX_GxGSA_OFF));
    tpoll_GSA_GSV_info.setInterval(frequency * 500);
    tpoll_GSA_GSV_info.enable();
  }
  else
  {
    tpoll_GSA_GSV_info.disable();
  }
}

// handle restart when config is lost
Task trestart_after_sleep(0, 1, restart_after_sleep, &ts, false);
void restart_after_sleep()
{
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

void switch_baudrate(uint32_t newbaudrate)
{
  // stored_preferences assumed global
  log_d("Send UBX-CFG for rate %d", newbaudrate);
  switch (newbaudrate)
  {
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
  log_d("Changing baud on UART2 from %u to %u , pins RX2=%d TX2=%d", gpsPort.baudRate(), newbaudrate, RX2, TX2);
  gpsPort.updateBaudRate(newbaudrate);
  stored_preferences.gps_baud_rate = newbaudrate;
}

// read from NVM via preferences library
void ReadNVMPreferences()
{
  // prefs assumed global
  // stored_preferences assumed global
  String string_wifi_mode;

  switch (stored_preferences.wifi_mode)
  {
  case WIFI_AP:
    string_wifi_mode = "WIFI_AP";
    break;
  case WIFI_STA:
    string_wifi_mode = "WIFI_STA";
    break;
  case WIFI_OFF:
    string_wifi_mode = "WIFI_OFF";
    break;
  default:
    string_wifi_mode = "WIFI_STA";
    break;
  }

  stored_preferences.gps_baud_rate = prefs.getULong("gpsbaudrate", stored_preferences.gps_baud_rate);
  stored_preferences.gps_rate = prefs.getUChar("gpsrate", stored_preferences.gps_rate);
  stored_preferences.nmeaGSAGSVpolling = prefs.getUChar("gsagsvpoll", stored_preferences.nmeaGSAGSVpolling);
  // wifi mode : remember https://www.esp32.com/viewtopic.php?t=12085
  string_wifi_mode = prefs.getString("wifi");
  if (string_wifi_mode == "WIFI_STA")
  {
    stored_preferences.wifi_mode = WIFI_STA;
    log_d("Preference read as WIFI_STA");
  }
  else if (string_wifi_mode == "WIFI_AP")
  {
    stored_preferences.wifi_mode = WIFI_AP;
    log_d("Preference read as WIFI_AP");
  }
  else if (string_wifi_mode == "WIFI_OFF")
  {
    stored_preferences.wifi_mode = WIFI_OFF;
    log_d("Preference read as WIFI_OFF");
  }
  else 
  {
    stored_preferences.wifi_mode = WIFI_AP;
    log_d("Error reading WiFi, Preference set as WIFI_AP");
  }

  // this being a char*, we pass size
  prefs.getString("wifikey", stored_preferences.wifi_key, WIFI_KEY_MAXLEN);
  prefs.getString("wifissid", stored_preferences.wifi_ssid, WIFI_SSID_MAXLEN);
  stored_preferences.nmeaGSA = prefs.getBool("nmeagsa", stored_preferences.nmeaGSA);
  stored_preferences.nmeaGSV = prefs.getBool("nmeagsv", stored_preferences.nmeaGSV);
  stored_preferences.nmeaGBS = prefs.getBool("nmeagbs", stored_preferences.nmeaGBS);
  stored_preferences.nmeaVTG = prefs.getBool("nmeavtg", stored_preferences.nmeaVTG);
  stored_preferences.nmeaGLL = prefs.getBool("nmeagll", stored_preferences.nmeaGLL);
  stored_preferences.nmeaTcpServer = prefs.getBool("nmeatcp", stored_preferences.nmeaTcpServer);
  stored_preferences.ble_active = prefs.getBool("ble", stored_preferences.ble_active);
  stored_preferences.btspp_active = prefs.getBool("btspp", stored_preferences.btspp_active);
  stored_preferences.trackaddict = prefs.getBool("trackaddict", stored_preferences.trackaddict);
  stored_preferences.racechrono = prefs.getBool("racechrono", stored_preferences.racechrono);
  stored_preferences.racetime = prefs.getBool("racetime", stored_preferences.racetime);

  if (stored_preferences.ble_active && stored_preferences.btspp_active)
  {
    // prevents both settings to be ON
#ifdef BLEENABLED
    stored_preferences.btspp_active = false;
#else
    stored_preferences.ble_active = false;
#endif
  }
}

// write to preferences
void StoreNVMPreferences(bool savewifi = false)
{
  // prefs assumed global
  // stored_preferences assumed global
  String string_wifi_mode;
  size_t size_t_written;

  if (savewifi)
  {
    switch (stored_preferences.wifi_mode)
    {
    case WIFI_AP:
      string_wifi_mode = "WIFI_AP";
      break;
    case WIFI_STA:
      string_wifi_mode = "WIFI_STA";
      break;
    case WIFI_OFF:
      string_wifi_mode = "WIFI_OFF";
      break;
    default:
      string_wifi_mode = "WIFI_STA";
      break;
    }
    size_t_written = prefs.putString("wifi", string_wifi_mode);
  }
  size_t_written = prefs.putString("wifikey", stored_preferences.wifi_key);
  size_t_written = prefs.putString("wifissid", stored_preferences.wifi_ssid);

  size_t_written = prefs.putULong("gpsbaudrate", stored_preferences.gps_baud_rate);
  size_t_written = prefs.putUChar("gpsrate", stored_preferences.gps_rate);
  size_t_written = prefs.putUChar("gsagsvpoll", stored_preferences.nmeaGSAGSVpolling);
  size_t_written = prefs.putBool("nmeagsa", stored_preferences.nmeaGSA);
  size_t_written = prefs.putBool("nmeagsv", stored_preferences.nmeaGSV);
  size_t_written = prefs.putBool("nmeagbs", stored_preferences.nmeaGBS);
  size_t_written = prefs.putBool("nmeavtg", stored_preferences.nmeaVTG);
  size_t_written = prefs.putBool("nmeagll", stored_preferences.nmeaGLL);
  size_t_written = prefs.putBool("nmeatcp", stored_preferences.nmeaTcpServer);
  size_t_written = prefs.putBool("ble", stored_preferences.ble_active);
  size_t_written = prefs.putBool("btspp", stored_preferences.btspp_active);
  size_t_written = prefs.putBool("trackaddict", stored_preferences.trackaddict);
  size_t_written = prefs.putBool("racechrono", stored_preferences.racechrono);
  size_t_written = prefs.putBool("racetime", stored_preferences.racetime);
  
  if (size_t_written > 0)
  {
    log_i("Preferences written");
  }
  else
  {
    log_e("Preferences NOT written");
  }
}

void StoreNVMPreferencesWiFi(String string_wifi_mode)
{
  // prefs assumed global
  size_t size_t_written;
  size_t_written = prefs.putString("wifi", string_wifi_mode);
  if (size_t_written > 0)
  {
    log_i("Preferences %s written", string_wifi_mode);
  }
  else
  {
    log_e("Preferences %s NOT written", string_wifi_mode);
  }
}

void StoreNVMPreferencesWiFiCreds()
{
  // prefs assumed global
  size_t size_t_written;
  size_t_written = prefs.putString("wifikey", stored_preferences.wifi_key);
  if (size_t_written > 0)
  {
    log_i("WiFi Key written (size %d)", size_t_written);
  }
  else
  {
    log_e("WiFi Key NOT written");
  }
  size_t_written = prefs.putString("wifissid", stored_preferences.wifi_ssid);
  if (size_t_written > 0)
  {
    log_i("WiFi SSID written (size %d)", size_t_written);
  }
  else
  {
    log_e("WiFi SSID NOT written");
  }
}

void gps_disable_all()
{
  stored_preferences.trackaddict = false;
  stored_preferences.racechrono = false;
  stored_preferences.racetime = false;
}
void gps_enable_common()
{
  gps_disable_all();
  push_gps_message(UBLOX_GxGSA_OFF, sizeof(UBLOX_GxGSA_OFF));
  stored_preferences.nmeaGSA = false;
  push_gps_message(UBLOX_GxGSV_OFF, sizeof(UBLOX_GxGSV_OFF));
  stored_preferences.nmeaGSV = false;
  push_gps_message(UBLOX_GxGBS_OFF, sizeof(UBLOX_GxGBS_OFF));
  stored_preferences.nmeaGBS = false;
  push_gps_message(UBLOX_GxVTG_OFF, sizeof(UBLOX_GxVTG_OFF));
  stored_preferences.nmeaVTG = false;
  push_gps_message(UBLOX_GxGLL_OFF, sizeof(UBLOX_GxGLL_OFF));
  stored_preferences.nmeaGLL = false;
  push_gps_message(UBLOX_INIT_10HZ, sizeof(UBLOX_INIT_10HZ));
  stored_preferences.gps_rate = 10;
#ifdef TASK_SCHEDULER
  control_poll_GSA_GSV(0);
#else
  stored_preferences.nmeaGSAGSVpolling = 0;
#endif
}
void gps_enable_trackaddict()
{
  log_i("Setting GPS to specific Track Addict needs");
  gps_enable_common();
  push_gps_message(UBLOX_INIT_MAINTALKER_GP, sizeof(UBLOX_INIT_MAINTALKER_GP));
  push_gps_message(UBLOX_GxGLL_ON, sizeof(UBLOX_GxGLL_ON));
  stored_preferences.nmeaGLL = true;
  stored_preferences.trackaddict = true;
}
void gps_enable_racetime()
{
  log_i("Setting GPS to specific RaceTime needs");
  gps_enable_common();
  push_gps_message(UBLOX_GxVTG_ON, sizeof(UBLOX_GxVTG_ON));
  stored_preferences.nmeaVTG = true;
  push_gps_message(UBLOX_GxGLL_ON, sizeof(UBLOX_GxGLL_ON));
  stored_preferences.nmeaGLL = true;
  push_gps_message(UBLOX_INIT_MAINTALKER_GP, sizeof(UBLOX_INIT_MAINTALKER_GP));
  stored_preferences.racetime = true;
}
void gps_enable_racechrono()
{
  log_i("Setting GPS to specific RaceChrono needs");
  gps_enable_common();
  push_gps_message(UBLOX_INIT_MAINTALKER_GP_GPSONLY, sizeof(UBLOX_INIT_MAINTALKER_GP_GPSONLY));
#ifdef TASK_SCHEDULER
  control_poll_GSA_GSV(5);
  stored_preferences.nmeaGSAGSVpolling = 5;
#else
  stored_preferences.nmeaGSAGSVpolling = 0;
#endif
  stored_preferences.racechrono = true;
}

#ifdef NUMERICAL_BROADCAST_PROTOCOL
const uint NBPServerPort = NBP_TCP_PORT;
WiFiServer NBPServer(NBPServerPort);
WiFiClient NBPRemoteClient;
#endif

const uint NMEAServerPort = NMEA_TCP_PORT;
WiFiServer NMEAServer(NMEAServerPort);
WiFiClient NMEARemoteClient;

bool ledon;
void led_blink()
{
  //toggle state
  ledon = !ledon;
  digitalWrite(LED_BUILTIN, (ledon ? HIGH : LOW)); // set pin to the opposite state
}

#ifdef TASK_SCHEDULER
Task tLedBlink(0, TASK_FOREVER, &led_blink, &ts, false);
#endif

/********************************

    WiFi connection

 * ******************************/
bool wifi_connected = false;
IPAddress MyIP;

void NMEACheckForConnections()
{
  if (NMEAServer.hasClient())
  {
    // If we are already connected to another computer, then reject the new connection. Otherwise accept the connection.
    if (NMEARemoteClient.connected())
    {
      log_w("NMEA TCP Connection rejected");
      NMEAServer.available().stop();
    }
    else
    {
      NMEARemoteClient = NMEAServer.available();
      // TODO not printing strings correctly
      log_d("NMEA TCP Connection accepted from client: %s", NMEARemoteClient.remoteIP().toString());
    }
  }
}
void start_NMEA_server()
{
  log_i("Start GNSS TCP/IP Service");
  NMEAServer.begin();
  NMEAServer.setNoDelay(true);
}

void stop_NMEA_server()
{
  log_i("Stop GNSS TCP/IP Service");
  NMEAServer.stop();
}

#ifdef NUMERICAL_BROADCAST_PROTOCOL
unsigned long nbptimer = 0;
void NBPCheckForConnections()
{
  if (NBPServer.hasClient())
  {
    // If we are already connected to another computer, then reject the new connection. Otherwise accept the connection.
    if (NBPRemoteClient.connected())
    {
      log_w("NBP TCP Connection rejected");
      NBPServer.available().stop();
    }
    else
    {
      NBPRemoteClient = NBPServer.available();
      log_i("NBP TCP Connection accepted from client: %s", NBPRemoteClient.remoteIP().toString());
    }
  }
}
void start_NBP_server()
{
  log_i("Start NBP TCP/IP Service");
  NBPServer.begin();
  NBPServer.setNoDelay(true);
}
#endif

// Start STATION mode to connect to a well-known Access Point
void wifi_STA()
{
  if (stored_preferences.nmeaTcpServer)
    stop_NMEA_server();
  // WiFi Access
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  // start ticker_wifi with 50 ms because we start in STA mode and try to connect
#ifdef TASK_SCHEDULER
  log_d("Start rapid blinking");
  tLedBlink.setInterval(50);
  tLedBlink.enable();
#endif

  // TODO : add connection to STA mode with stored passwords here
  WiFi.begin(stored_preferences.wifi_ssid, stored_preferences.wifi_key);
  wifi_connected = false;

  int times = 0;
  while (WiFi.status() != WL_CONNECTED && times < 20)
  {
    delay(250);
    log_i("Connecting to WiFi %s , trial %d", stored_preferences.wifi_ssid, times++);
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    log_i("Connected to SSID %s", stored_preferences.wifi_ssid);
    #ifdef TASK_SCHEDULER
    tLedBlink.setInterval(250);
    #endif
    wifi_connected = true;
#ifdef ENABLE_OTA
    log_i("Start OTA service");
    OTA_start();
#endif
    log_i("Start Web Portal");
    WebConfig_start();
    // WLAN Server for GNSS data
    if (stored_preferences.nmeaTcpServer)
      start_NMEA_server();
#ifdef NUMERICAL_BROADCAST_PROTOCOL
    // WLAN Server for GNSS data
    start_NBP_server();
#endif

    MyIP = WiFi.localIP();

#ifdef BUTTON
    button.onPressed(wifi_OFF);
#endif
    stored_preferences.wifi_mode = WIFI_STA;
  }
  else
  {
    log_e("Could not connect to SSID %s, reverting to AP mode", stored_preferences.wifi_ssid);
    wifi_AP();
  }
}

void wifi_AP()
{
  // WiFi Access
  if (stored_preferences.nmeaTcpServer)
    stop_NMEA_server();
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
  if (!MDNS.begin(BONOGPS_MDNS))
  {
    log_e("Error setting up MDNS responder!");
  }
  else
  {
    log_i("mDNS responder started");
  }
#endif

  log_i("Start Web Portal");
  WebConfig_start();

#ifdef TASK_SCHEDULER
  log_d("Start blinking");
  tLedBlink.setInterval(1000);
  tLedBlink.enable();
#endif

  // WLAN Server for GNSS data
  if (stored_preferences.nmeaTcpServer)
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

void wifi_OFF()
{

  #ifdef TASK_SCHEDULER
  log_d("Flash blinking");
  tLedBlink.setInterval(50);
  #endif

  #ifdef ENABLE_OTA
    log_i("Stop OTA service");
    OTA_stop();
  #endif
  #ifdef NUMERICAL_BROADCAST_PROTOCOL
  // WLAN Server for GNSS data
  stop_NBP_server();
  #endif
  // WiFi Access
  if (stored_preferences.nmeaTcpServer)
    stop_NMEA_server();
  
  log_i("Stop Web Portal");
  WebConfig_stop();
  
  #ifdef MDNS_ENABLE
  MDNS.end();
  #endif

  WiFi.mode(WIFI_OFF);
  log_i("Stopped WiFi service");

  #ifdef TASK_SCHEDULER
  log_d("Stop blinking");
  tLedBlink.setInterval(0);
  tLedBlink.disable();
  ledon=HIGH;
  led_blink();
  #endif

  #ifdef BUTTON
  button.onPressed(wifi_AP);
  #endif

  stored_preferences.wifi_mode = WIFI_OFF;

}

/********************************

   Web Configuration portal

 * ******************************/
#include <WebServer.h>
WebServer webserver(80);

const char html_text[] PROGMEM = "text/html";
const char html_css[] PROGMEM = "text/css";
const char text_json[] PROGMEM = "application/json";
const char json_ok[] PROGMEM = "{'status':'ok'}";
const char json_error[] PROGMEM = "{'status':'error'}";
const char WEBPORTAL_CSS[] PROGMEM = "*{box-sizing:border-box;text-align:center;width:100%;font-weight:300}body{font-family:Roboto,system-ui,Arial,Helvetica,sans-serif;font-size:5vw;margin:0}header{background-color:#666;padding:.5vw;text-align:center;color:#fff}article{float:left;padding:10px;width:100%;height:auto}details{display:table;clear:both}summary{font-size:larger;font-weight:400;padding:10px;background-color:#f1f1f1}footer{background-color:#777;padding:.2vw;text-align:center;color:#fff;clear:both;position:fixed;bottom:0;font-size:small}@media (min-width:800px){article{width:50%}*{font-size:2.5vw}}a,input{color:#fff;border-radius:8pt;background:red;text-decoration:none;padding:5pt}.bg input{display:none}label{border:solid;border-radius:8pt;padding:5px;margin:2px;border-color:#bdbdbd;border-width:2px;color:#9e9e9e}.bg input:checked+label,.bg input:checked+label:active{background:red;color:#fff;border-color:red}";
const char WEBPORTAL_HEADER[] PROGMEM = "<!DOCTYPE html>\n<html lang='en'>\n\t<head>\n\t\t<title>Bono GPS</title>\n\t\t<meta charset='utf-8'>\n\t\t<meta name='viewport' content='width=device-width, initial-scale=1'>\n\t\t<link rel='stylesheet' href='/css'>\n\t</head>\n<body>\n<script>function Select(e){fetch('/'+e).then(e=>e.text()).then(t=>console.log(e))}</script>\n<header>";
#ifdef GIT_REPO
const char WEBPORTAL_FOOTER[] PROGMEM = "\n<footer>Version: <a style='font-size: small;background: none;text-decoration: underline;' target='_blank' href='" GIT_REPO "'>" BONO_GPS_VERSION "</a></footer>\n</body>\n</html>";
#else
const char WEBPORTAL_FOOTER[] PROGMEM = "\n<footer>Version: " BONO_GPS_VERSION "</footer>\n</body>\n</html>";
#endif
const char WEBPORTAL_ROOT_OPTIONS[] PROGMEM = "\n</details>\n<details><summary>Device</summary>\n<article>Suspend GPS for <a href='/powersave/1800'>30'</a> <a href='/powersave/3600'>1 hr</a></article>\n<article>Disable<a href='/wifioff'>WiFi</a></article>\n<article><a href='/preset'>Load Preset</a></article>\n<article><a href='/savecfg'>Save config</a></article>\n<article><a href='/status'>Information</a></article>\n<article><a href='/savewificreds'>Set WiFi credentials</a></article>\n<article><a href='/restart'>Restart</a><br><br></article></details>";
const char WEBPORTAL_OPTION_CHECKED[] PROGMEM = "' checked>";
const char WEBPORTAL_OPTION_LABELCLASSBTN[] PROGMEM = "\n\t<label class='button' for='";
const char WEBPORTAL_OPTION_ARTICLECLASS[] PROGMEM = "\n<article class='bg'>";
const char WEBPORTAL_OPTION_SELECT_ONCHANGE[] PROGMEM = "onchange='Select(this.id)' name='";

// Helpers to populate a page with style sheet
String generate_html_header(bool add_menu = true)
{
  String htmlbody((char *)0);
  htmlbody.reserve(500);
  htmlbody += (add_menu ? F("Back to <a href='/'>Main menu</a>") : String(ap_ssid));
  if (gps_powersave) htmlbody += F("<br><em>Powersave mode</em>") ;
  htmlbody += F("</header>\n");
  return htmlbody;
}
String generate_html_footer()
{
  return String(WEBPORTAL_FOOTER);
}
String generate_html_body(String input, bool add_menu = true)
{
  // assumption: ap_ssid is populated
  String htmlbody((char *)0);
  htmlbody.reserve(2000);
  htmlbody += String(WEBPORTAL_HEADER);
  htmlbody += generate_html_header(add_menu);
  htmlbody += input;
  htmlbody += String(WEBPORTAL_FOOTER);
  return htmlbody;
}

String input_onoff(String divlabel, String parameter, bool parameter_status)
{
  String message((char *)0);
  message.reserve(500);
  message += String(WEBPORTAL_OPTION_ARTICLECLASS);
  message += divlabel;
  message += F("\n\t<input type='radio' id='");
  message += parameter;
  message += F("/on' ");
  message += String(WEBPORTAL_OPTION_SELECT_ONCHANGE);
  message += parameter;
  if (parameter_status)
  {
    message += String(WEBPORTAL_OPTION_CHECKED);
  }
  else
  {
    message += "'>";
  }
  message += String(WEBPORTAL_OPTION_LABELCLASSBTN);
  message += parameter;
  message += F("/on'> On </label>\n\t<input type='radio' id='");
  message += parameter;
  message += F("/off' ");
  message += String(WEBPORTAL_OPTION_SELECT_ONCHANGE);
  message += parameter;
  if (parameter_status)
  {
    message += "'>";
  }
  else
  {
    message += String(WEBPORTAL_OPTION_CHECKED);
  }
  message += String(WEBPORTAL_OPTION_LABELCLASSBTN);
  message += parameter;
  message += F("/off'> Off </label>\n\t</article>");
  return message;
}

String html_select(String divlabel, String parameters[], String parameters_names[], bool parameters_status[], String groupname, int numberparams)
{
  String message((char *)0);
  message.reserve(500);
  message += String(WEBPORTAL_OPTION_ARTICLECLASS);;
  message += divlabel;
  for (int i = 0; i < numberparams; i++)
  {
    message += F("\n\t<input type='radio' id='");
    message += groupname;
    message += String("/");
    message += parameters[i];
    message += String("'");
    message += String(WEBPORTAL_OPTION_SELECT_ONCHANGE);
    message += groupname;
    if (parameters_status[i])
    {
      message += String(WEBPORTAL_OPTION_CHECKED);
    }
    else
    {
      message += String("'>");
    }
    message += String(WEBPORTAL_OPTION_LABELCLASSBTN);
    message += groupname;
    message += String("/");
    message += parameters[i];
    message += String("'>");
    message += parameters_names[i];
    message += F(" </label>");
  }
  message += "\n</article>";
  return message;
}

void handle_css()
{
  log_i("Handle CSS response");
  webserver.sendHeader("Cache-Control", "max-age=3600");
  webserver.send_P(200, html_css, WEBPORTAL_CSS);
}
void handle_menu()
{
  // variables used to build html buttons
  String sv_values[2] = {"8", "all"};
  String sv_names[2] = {"8", "All"};
  bool sv_status[2] = {false, true};

  String baud_values[3] = {"38400", "57600", "115200"};
  String baud_names[3] = {"38k4", "57k6", "115k2"};
  bool baud_status[3] = {(stored_preferences.gps_baud_rate == 38400), (stored_preferences.gps_baud_rate == 57600), (stored_preferences.gps_baud_rate == 115200)};

  String wifi_values[2] = {"sta", "ap"};
  String wifi_names[2] = {"Client", "AP"};
  bool wifi_status[2] = {(stored_preferences.wifi_mode == WIFI_STA), (stored_preferences.wifi_mode == WIFI_AP)};

#ifdef TASK_SCHEDULER
  String pollgsagsv_values[3] = {"0", "1", "5"};
  String pollgsagsv_names[3] = {"Off", "1 sec", "5 sec"};
  bool pollgsagsv_status[3] = {(stored_preferences.nmeaGSAGSVpolling == 0), (stored_preferences.nmeaGSAGSVpolling == 1), (stored_preferences.nmeaGSAGSVpolling == 5)};
#endif

  log_i("Handle webserver root response");
  String mainpage((char *)0);
  mainpage.reserve(3500);
  mainpage += generate_html_header(false);
  mainpage += F("\n<details open><summary>GPS runtime settings</summary>\n<article class='bg'>Update\n<input type='radio' id='rate/1hz' ");
  mainpage += String(WEBPORTAL_OPTION_LABELCLASSBTN);
  mainpage += ((stored_preferences.gps_rate == 1) ? "rate' checked>" : "rate'> ");
  mainpage += String(WEBPORTAL_OPTION_LABELCLASSBTN);
  mainpage += F("rate/1hz'>1 Hz</label>\n<input type='radio' id='rate/5hz' ");
  mainpage += String(WEBPORTAL_OPTION_SELECT_ONCHANGE);
  mainpage += ((stored_preferences.gps_rate == 5) ? "rate' checked>" : "rate'> ");
  mainpage += String(WEBPORTAL_OPTION_LABELCLASSBTN);
  mainpage += F("'rate/5hz'>5 Hz</label>\n<input type='radio' id='rate/10hz' ");
  mainpage += String(WEBPORTAL_OPTION_SELECT_ONCHANGE);
  mainpage += ((stored_preferences.gps_rate == 10) ? "rate' checked>" : "rate'> ");
  mainpage += String(WEBPORTAL_OPTION_LABELCLASSBTN);
  mainpage += F("rate/10hz'>10 Hz</label></article>");
  mainpage += input_onoff("Stream GxGBS", "gbs", stored_preferences.nmeaGBS);
  mainpage += input_onoff("Stream GxGSA", "gsa", stored_preferences.nmeaGSA);
  mainpage += input_onoff("Stream GxGSV", "gsv", stored_preferences.nmeaGSV);
#ifdef TASK_SCHEDULER
  mainpage += html_select("Poll GSA + GSV ", pollgsagsv_values, pollgsagsv_names, pollgsagsv_status, String("poll/gsagsv"), 3);
#endif
  mainpage += html_select("SVs per Talker Id", sv_values, sv_names, sv_status, String("sv"), 2);
  mainpage += html_select("Port BAUD", baud_values, baud_names, baud_status, String("baud"), 3);
  mainpage += F("</details>\n<details>\n<summary>Connections</summary><article>List connected <a href='/clients'>clients</a></article>");
  mainpage += html_select("WiFi mode ", wifi_values, wifi_names, wifi_status, String("wifi"), 2);
#ifdef BLEENABLED
  mainpage += input_onoff("Bluetooth LE", "ble", stored_preferences.ble_active);
#endif
#ifdef BTSPPENABLED
  mainpage += input_onoff("Bluetooth SPP", "btspp", stored_preferences.btspp_active);
#endif
  mainpage += input_onoff("TCP/IP", "tcpserver", stored_preferences.nmeaTcpServer);
  log_i("sending webserver root response");
  webserver.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webserver.send_P(200, html_text, WEBPORTAL_HEADER);
  webserver.sendContent(mainpage);
  webserver.sendContent_P(WEBPORTAL_ROOT_OPTIONS);
  webserver.sendContent_P(WEBPORTAL_FOOTER);
}

void handle_preset()
{
  String mainpage((char *)0);
  mainpage.reserve(1000);
  mainpage += generate_html_header(true);
#if defined(BTSPPENABLED) || defined(BLEENABLED)
  // hlt main page
#if defined(BTSPPENABLED) && defined(BLEENABLED)
  mainpage += F("<details open><summary>Harry Lap Timer <a target='_blank' href='https://www.gps-laptimer.de/'>?</a></summary><article>Recommended options:<br><ul><li>GBS streaming</li><li>GSA GSV polling</li><li>10 Hz updates</li><li>Android: BT-SPP Connection</li><li>iOS: BLE Connection</li></ul></article><article>Load options for:<p><a href='/hlt/tcpip'>iOS + Android: WiFi</a></p><p><a href='/hlt/ios'>iOS: BLE</a></p><p><a href='/hlt/android'>Android: BT-SPP</a></p><p>A save config and restart are recommended after enabling/disabling BT-SPP</p></article></details>");
#else
#ifdef BLEENABLED
  mainpage += F("<details open><summary>Harry Lap Timer <a target='_blank' href='https://www.gps-laptimer.de/'>?</a></summary><article>Recommended options:<br><ul><li>GBS streaming</li><li>GSA GSV polling</li><li>10 Hz updates</li><li>iOS: BLE Connection</li></ul></article><article>Load  options forn:<p><a href='/hlt/tcpip'>iOS + Android: WiFi</a></p><p><a href='/hlt/ios'>iOS: BLE</a></article></details>");
#endif
#ifdef BTSPPENABLED
  mainpage += F("<details open><summary>Harry Lap Timer <a href='https://www.gps-laptimer.de/'>?</a></summary><article>Recommended options:<br><ul><li>GBS streaming</li><li>GSA GSV polling</li><li>10 Hz updates</li><li>Android: BT-SPP Connection</li></ul></article><article><Load options for:<p><a href='/hlt/tcpip'>iOS + Android: WiFi</a></p><p><a href='/hlt/android'>Android: BT-SPP</a></p><p>A save config and restart are recommended after enabling/disabling BT-SPP</article></details>");
#endif
#endif // defined(BTSPPENABLED) && defined(BLEENABLED)

#ifdef BTSPPENABLED
  // racechrono main page
  mainpage += F("<details open><summary>RaceChrono <a target='_blank' href='https://racechrono.com/'>?</a></summary><article>Recommended options:<br><ul><li>Talker id GPS for all systems</li><li>Restrict GSV to GPS</li><li>no GBS</li><li>GSA+GSV polling every 5 sec</li><li>10 Hz updates</li><li>BT-SPP Connection only</li></ul></article><article><p>Load options for:<p><a href='/racechrono/android'>Android: BT-SPP</a></p></article></details>");

// racetime main page
  mainpage += F("<details open><summary>Racetime Lite <a target='_blank' href='https://www.racetimeapp.com/en/'>?</a></summary><article>Recommended options:<br><ul><li>Talker id GPS for all systems</li><li>GLL+VTG+RMC+GGA Enabled</li><li>no GSA GSV GBS</li><li>10 Hz updates</li><li>BT-SPP Connection only</li></ul></article><article><p>Load options for:<p><a href='/racetime/android'>Android: BT-SPP</a></p></article></details>");

  // trackaddict main page
  mainpage += F("<details open><summary>TrackAddict <a target='_blank' href='https://www.hptuners.com/product/trackaddict-app/'>?</a></summary><article>Required options:<br><ul><li>Talker id GPS for all systems</li><li>no GSA GSV GBS </li><li>GLL Streaming</li><li>10 Hz updates</li><li>BT-SPP Connection only</li></ul></article>");
  mainpage += input_onoff("Android: BT-SPP", "trackaddict", stored_preferences.trackaddict);
  mainpage += F("<article><p>A save config and restart are recommended after enabling/disabling BT-SPP</p></article></details>");
#endif

#else
  String mainpage = "No settings available with this firmware options";
#endif
  log_i("Handle load preset");
  webserver.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webserver.send_P(200, html_text, WEBPORTAL_HEADER);
  webserver.sendContent(mainpage);
  webserver.sendContent_P(WEBPORTAL_FOOTER);
}

#ifdef BLEENABLED
void handle_ble()
{
  String onoff = webserver.pathArg(0);
  log_i("Set BLE Server to %s ", onoff);
  if (onoff == "on")
  {
#ifdef BTSPPENABLED
    if (stored_preferences.btspp_active)
    {
      stored_preferences.btspp_active = false;
      bt_spp_stop();
      delay(250);
    }
#endif
    if (!stored_preferences.ble_active)
    {
      stored_preferences.ble_active = true;
      ble_start();
    }
#ifdef SHORT_API
    webserver.send_P(200, text_json, json_ok);
#else
#ifdef BTSPPENABLED
    webserver.send(200, html_text, generate_html_body(F("Turned <b>ON</b> BLE, and turned <b>OFF</b> BT-SPP<p>This causes a restart of the device: to store permanently, disable BT-SPP, save config, enable BLE")));
#else
    webserver.send(200, html_text, generate_html_body(F("Turned <b>ON</b> BLE<br><a href='/savecfg/nowifi'>Save settings</a>")));
#endif // BTSPPENABLED
#endif // SHORT_API
  }
  else if (onoff == "off")
  {
    if (stored_preferences.ble_active)
    {
      stored_preferences.ble_active = false;
      ble_stop();
    }
#ifdef SHORT_API
    webserver.send_P(200, text_json, json_ok);
#else
    webserver.send(200, html_text, generate_html_body(F("Turn <b>off</b> BLE<br><a href='/savecfg/nowifi'>Save settings</a>")));
#endif // SHORT_API
  }
}
#endif // BLEENABLED

#ifdef BTSPPENABLED
void handle_btspp()
{
  String onoff = webserver.pathArg(0);
  log_i("Set BT-SPP Server to %s ", onoff);
  if (onoff == "on")
  {

#ifdef BLEENABLED
    if (stored_preferences.ble_active)
    {
      stored_preferences.ble_active = false;
      ble_stop();
    }
#endif
    if (!stored_preferences.btspp_active)
    {
      stored_preferences.btspp_active = true;
      bt_spp_start();
    }
  }
  else if (onoff == "off")
  {
    if (stored_preferences.btspp_active)
    {
      stored_preferences.btspp_active = false;
      bt_spp_stop();
    }
  }
  else
  {
#ifdef SHORT_API
    webserver.send_P(500, text_json, json_error);
#else
    webserver.send(500, html_text, generate_html_body(String(F("Error: BT-SPP not set to")) + onoff));
#endif
  }
#ifdef SHORT_API
  webserver.send_P(200, text_json, json_ok);
#else
  webserver.send(200, html_text, generate_html_body(String(F("BT-SPP set to")) + onoff));
#endif
}
#endif // BTSPPENABLED
void handle_powersave()
{
  String sttime = webserver.pathArg(0);
  log_i("Set power saving mode for %s sec", sttime);
  // disable polling of GSA and GSV
  int time = sttime.toInt();
#ifdef TASK_SCHEDULER
  control_poll_GSA_GSV(0);
  trestart_after_sleep.setInterval(time * 1000);
  trestart_after_sleep.enableDelayed();
#endif
  switch (time)
  {
  case 3600:
    push_gps_message(UBLOX_PWR_SAVE_1HR, sizeof(UBLOX_PWR_SAVE_1HR));
    break;
  case 1800:
    push_gps_message(UBLOX_PWR_SAVE_30MIN, sizeof(UBLOX_PWR_SAVE_30MIN));
    break;
  default:
    break;
  }
  gps_powersave = true;
#ifdef SHORT_API
  webserver.send_P(200, text_json, json_ok);
#else
  webserver.send(200, html_text, generate_html_body(F("Enabled power saving mode for 3600 seconds")));
#endif
}

static const char savecfg[] PROGMEM = "<br><a href='/savecfg/nowifi'>Save settings</a>";
void handle_rate()
{
  String strate = webserver.pathArg(0);
  log_i("Set GPS Rate to %s Hz", strate);
  int rate = strate.toInt();
  stored_preferences.gps_rate = rate;
  switch (rate)
  {
  case 10:
    push_gps_message(UBLOX_INIT_10HZ, sizeof(UBLOX_INIT_10HZ));
    break;
  case 5:
    push_gps_message(UBLOX_INIT_5HZ, sizeof(UBLOX_INIT_5HZ));
    break;
  default:
    push_gps_message(UBLOX_INIT_1HZ, sizeof(UBLOX_INIT_1HZ));
    break;
  }
#ifdef SHORT_API
  webserver.send_P(200, text_json, json_ok);
#else
  String message((char *)0);
  message.reserve(70);
  message += F("Rate set to ");
  message += strate;
  message += FPSTR(savecfg);
  webserver.send(200, html_text, generate_html_body(message));
#endif
}

void handle_baudrate()
{
  String baudrate = webserver.pathArg(0);
  log_i("Set BAUD Rate to %s", baudrate);
  switch_baudrate(baudrate.toInt());
#ifdef SHORT_API
  webserver.send_P(200, text_json, json_ok);
#else
  String message((char *)0);
  message.reserve(70);
  message += F("Baud set to ");
  message += baudrate;
  message += FPSTR(savecfg);
  webserver.send(200, html_text, generate_html_body(message));
#endif
}

void handle_tcpserver()
{
  // stored_preferences.nmeaTcpServer
  String onoff = webserver.pathArg(0);
  log_i("Set TCP/IP Server to %s ", onoff);
  if (onoff == "on")
  {
    start_NMEA_server();
    stored_preferences.nmeaTcpServer = true;
  }
  else if (onoff == "off")
  {
    stop_NMEA_server();
    stored_preferences.nmeaTcpServer = false;
  }
  else
  {
#ifdef SHORT_API
    webserver.send_P(500, text_json, json_error);
#else
    String message((char *)0);
    message.reserve(70);
    message += F("Error state when changing TCP/IP server status");
    message += FPSTR(savecfg);
    webserver.send(200, html_text, generate_html_body(message));
#endif
    return;
  }
#ifdef SHORT_API
  webserver.send_P(200, text_json, json_ok);
#else
  String message((char *)0);
  message.reserve(70);
  message += F("TCP/IP Server now: ");
  message += choice;
  message += FPSTR(savecfg);
  webserver.send(200, html_text, generate_html_body(message));
#endif
}
void handle_gsv()
{
  String onoff = webserver.pathArg(0);
  log_i("Set GSV to %s ", onoff);
#ifdef TASK_SCHEDULER
  control_poll_GSA_GSV(0);
#endif
  if (onoff == "on")
  {
    push_gps_message(UBLOX_GxGSV_ON, sizeof(UBLOX_GxGSV_ON));
    stored_preferences.nmeaGSV = true;
    stored_preferences.nmeaGSAGSVpolling = 0;
  }
  else if (onoff == "off")
  {
    push_gps_message(UBLOX_GxGSV_OFF, sizeof(UBLOX_GxGSV_OFF));
    stored_preferences.nmeaGSV = false;
  }
  else
  {
#ifdef SHORT_API
    webserver.send_P(500, text_json, json_error);
#else
    String message((char *)0);
    message.reserve(70);
    message += F("Error state when changing GxGSV messages");
    message += FPSTR(savecfg);
    webserver.send(200, html_text, generate_html_body(message));
#endif
    return;
  }
#ifdef SHORT_API
  webserver.send_P(200, text_json, json_ok);
#else
  String message((char *)0);
  message.reserve(70);
  message += F("Enabled GxGSV messages");
  message += FPSTR(savecfg);
  webserver.send(200, html_text, generate_html_body(message));
#endif
}

void handle_gsa()
{
  String onoff = webserver.pathArg(0);
  log_i("Set GSA to %s ", onoff);
#ifdef TASK_SCHEDULER
  control_poll_GSA_GSV(0);
#endif
  if (onoff == "on")
  {
    push_gps_message(UBLOX_GxGSA_ON, sizeof(UBLOX_GxGSA_ON));
    stored_preferences.nmeaGSA = true;
    stored_preferences.nmeaGSAGSVpolling = 0;
  }
  else if (onoff == "off")
  {
    push_gps_message(UBLOX_GxGSA_OFF, sizeof(UBLOX_GxGSA_OFF));
    stored_preferences.nmeaGSA = false;
  }
  else
  {
#ifdef SHORT_API
    webserver.send_P(500, text_json, json_error);
#else
    String message((char *)0);
    message.reserve(70);
    message += F("Error state when changing GxGSA messages");
    message += FPSTR(savecfg);
    webserver.send(200, html_text, generate_html_body(message));
#endif
    return;
  }
#ifdef SHORT_API
  webserver.send_P(200, text_json, json_ok);
#else
  String message((char *)0);
  message.reserve(70);
  message += F("Enabled GxGSA messages");
  message += FPSTR(savecfg);
  webserver.send(200, html_text, generate_html_body(message));
#endif
}

#ifdef TASK_SCHEDULER
void handle_pollgsagsv()
{
  String stfrequency = webserver.pathArg(0);
  log_i("Set polling of GSA and GSV messages every %s sec", stfrequency);
  control_poll_GSA_GSV(stfrequency.toInt());
#ifdef SHORT_API
  webserver.send_P(200, text_json, json_ok);
#else
  String message((char *)0);
  message.reserve(70);
  message += F("Polling of GSA+GSV messages every ");
  message += stfrequency;
  message += FPSTR(savecfg);
  webserver.send(200, html_text, generate_html_body(message));
#endif
}
#endif // #ifdef TASK_SCHEDULER

void handle_gbs()
{
  String onoff = webserver.pathArg(0);
  log_i("Set GxGBS to %s ", onoff);
#ifdef TASK_SCHEDULER
  control_poll_GSA_GSV(0);
#endif
  if (onoff == "on")
  {
    push_gps_message(UBLOX_GxGBS_ON, sizeof(UBLOX_GxGBS_ON));
    stored_preferences.nmeaGBS = true;
  }
  else if (onoff == "off")
  {
    push_gps_message(UBLOX_GxGBS_OFF, sizeof(UBLOX_GxGBS_OFF));
    stored_preferences.nmeaGBS = false;
  }
  else
  {
#ifdef SHORT_API
    webserver.send_P(500, text_json, json_error);
#else
    String message((char *)0);
    message.reserve(70);
    message += F("Error state when changing GBS messages, value: ");
    message += onoff;
    message += FPSTR(savecfg);
    webserver.send(200, html_text, generate_html_body(message));
#endif
    return;
  }
#ifdef SHORT_API
  webserver.send_P(200, text_json, json_ok);
#else
  String message((char *)0);
  message.reserve(70);
  message += F("Set GxGSA messages to ");
  message += onoff;
  message += FPSTR(savecfg);
  webserver.send(200, html_text, generate_html_body(message));
#endif
}

void handle_svchannel()
{
  String channels = webserver.pathArg(0);
  log_i("Set %s SVs per Talker Id", channels);
  if (channels == "8")
  {
    push_gps_message(UBLOX_INIT_CHANNEL_8, sizeof(UBLOX_INIT_CHANNEL_8));
  }
  else if (channels == "off")
  {
    push_gps_message(UBLOX_INIT_CHANNEL_ALL, sizeof(UBLOX_INIT_CHANNEL_ALL));
  }
  else
  {
#ifdef SHORT_API
    webserver.send_P(200, text_json, json_error);
#else
    String message((char *)0);
    message.reserve(70);
    message += F("Error when setting SVs per Talked Id, value: ");
    message += channels;
    message += FPSTR(savecfg);
    webserver.send(200, html_text, generate_html_body(message));
#endif
    return;
  }

#ifdef SHORT_API
  webserver.send_P(200, text_json, json_ok);
#else
  String message((char *)0);
  message.reserve(70);
  message += "Set" + channels + " SVs ";
  message += FPSTR(savecfg);
  webserver.send(200, html_text, generate_html_body(message));
#endif
}

#ifdef BTSPPENABLED
void handle_trackaddict()
{
  String choice = webserver.pathArg(0);

  log_i("Set optimal configuration for Track Addict to %s", choice);

  if (choice == "on")
  { // /trackaddict/on
    gps_enable_trackaddict();
#ifdef BLEENABLED
    stored_preferences.ble_active = false;
    ble_stop();
#endif
    bt_spp_start();
    stored_preferences.btspp_active = true;
  }
  else if (choice == "off")
  { // /trackaddict/off
    gps_disable_all();
    if (stored_preferences.nmeaGSA)
      push_gps_message(UBLOX_GxGSA_ON, sizeof(UBLOX_GxGSA_ON));
    if (stored_preferences.nmeaGSV)
      push_gps_message(UBLOX_GxGSV_ON, sizeof(UBLOX_GxGSV_ON));
    if (stored_preferences.nmeaGBS)
      push_gps_message(UBLOX_GxGBS_ON, sizeof(UBLOX_GxGBS_ON));
#ifdef TASK_SCHEDULER
    control_poll_GSA_GSV(stored_preferences.nmeaGSAGSVpolling);
#endif
    // this also resets the Main Talker ID and GPS SV only options to the default (all talkers, all SV)
    push_gps_message(UBLOX_INIT_CHANNEL_ALL, sizeof(UBLOX_INIT_CHANNEL_ALL));
  }
  else
  {
#ifdef SHORT_API
    webserver.send_P(200, text_json, json_ok);
#else
    String message((char *)0);
    message.reserve(70);
    message += F("Set TrackAddict ");
    message += FPSTR(savecfg);
    webserver.send(200, html_text, generate_html_body(String("Error setting TrackAddict")));
#endif
  }
#ifdef SHORT_API
  webserver.send_P(500, text_json, json_error);
#else
  webserver.send(200, html_text, generate_html_body(String("Error setting TrackAddict")));
#endif
}
void handle_trackaddict_off()
{
}
#endif

#ifdef BTSPPENABLED
void handle_racechrono_android()
{
  // /racechrono/android
  log_i("Setting optimal configuration for RaceChrono on Android: 10Hz, GSA+GSV+GBS Off, BT-SPP");
  gps_enable_racechrono();
#ifdef BLEENABLED
  stored_preferences.ble_active = false;
  ble_stop();
#endif
  bt_spp_start();
  stored_preferences.btspp_active = true;
  String message((char *)0);
  message.reserve(70);
  message += F("Set optimal configuration for RaceChrono on Android:");
  message += FPSTR(savecfg);
  webserver.send(200, html_text, generate_html_body(message));
}
void handle_racetime_android()
{
  // /racetime/android
  log_i("Setting optimal configuration for RaceTime on Android: 10Hz, GSA+GSV+GBS Off, GLL+VTG ON, BT-SPP");
  gps_enable_racetime();
#ifdef BLEENABLED
  stored_preferences.ble_active = false;
  ble_stop();
#endif
  bt_spp_start();
  stored_preferences.btspp_active = true;
  String message((char *)0);
  message.reserve(70);
  message += F("Set optimal configuration for RaceTime on Android:");
  message += FPSTR(savecfg);
  webserver.send(200, html_text, generate_html_body(message));
}
void handle_hlt_android()
{
  // /hlt/android
  log_i("Setting optimal configuration for Harry Lap Timer on Android: 10Hz, GSA+GSV Off, GSA+GSV polled on a 5 sec cycle, GBS On, BT-SPP");
  push_gps_message(UBLOX_INIT_10HZ, sizeof(UBLOX_INIT_10HZ));
  stored_preferences.gps_rate = 10;
  stored_preferences.nmeaGSA = false;
  stored_preferences.nmeaGSV = false;
#ifdef TASK_SCHEDULER
  stored_preferences.nmeaGSAGSVpolling = 5;
  control_poll_GSA_GSV(5);
#else
  // control_poll will disable GSA and GSV on its own
  push_gps_message(UBLOX_GxGSA_OFF, sizeof(UBLOX_GxGSA_OFF));
  push_gps_message(UBLOX_GxGSV_OFF, sizeof(UBLOX_GxGSV_OFF));
#endif
  push_gps_message(UBLOX_GxGBS_ON, sizeof(UBLOX_GxGBS_ON));
  stored_preferences.nmeaGBS = true;
  // this also resets the Main Talker ID and GPS SV only options to the default (all talkers, all SV)
  push_gps_message(UBLOX_INIT_CHANNEL_ALL, sizeof(UBLOX_INIT_CHANNEL_ALL));
  log_i("Set optimal configuration for Harry Lap Timer on Android");
  gps_disable_all();
#ifdef BLEENABLED
  stored_preferences.ble_active = false;
  ble_stop();
#endif
  bt_spp_start();
  stored_preferences.btspp_active = true;
  String message((char *)0);
  message.reserve(70);
  message += F("Set optimal configuration for Harry Lap Timer on Android:");
  message += FPSTR(savecfg);
  webserver.send(200, html_text, generate_html_body(message));
}
#endif
void handle_hlt()
{
  // TODO find which device type
  String choice = webserver.pathArg(0);
  // /hlt/tcpip
  log_i("Setting optimal configuration for Harry Lap Timer on %s: 10Hz, GSA+GSV Polling, GBS On", choice);
  push_gps_message(UBLOX_INIT_10HZ, sizeof(UBLOX_INIT_10HZ));
  stored_preferences.gps_rate = 10;
  stored_preferences.nmeaGSA = false;
  stored_preferences.nmeaGSV = false;
#ifdef TASK_SCHEDULER
  stored_preferences.nmeaGSAGSVpolling = 5;
  control_poll_GSA_GSV(stored_preferences.nmeaGSAGSVpolling);
#else
  // control_poll will disable GSA and GSV on its own
  push_gps_message(UBLOX_GxGSA_OFF, sizeof(UBLOX_GxGSA_OFF));
  push_gps_message(UBLOX_GxGSV_OFF, sizeof(UBLOX_GxGSV_OFF));
#endif
  push_gps_message(UBLOX_GxGBS_ON, sizeof(UBLOX_GxGBS_ON));
  stored_preferences.nmeaGBS = true;
    // this also resets the Main Talker ID and GPS SV only options to the default (all talkers, all SV)
  push_gps_message(UBLOX_INIT_CHANNEL_ALL, sizeof(UBLOX_INIT_CHANNEL_ALL));
  gps_disable_all();
  if (choice == "tcpip")
  {
    if (!stored_preferences.nmeaTcpServer)
    {
      stored_preferences.nmeaTcpServer = true;
      start_NMEA_server();
    }
#ifdef BLEENABLED
    if (stored_preferences.ble_active)
    {
      ble_stop();
      stored_preferences.ble_active = false;
    }
#endif
#ifdef BTSPPENABLED
    if (stored_preferences.btspp_active)
    {
      stored_preferences.btspp_active = false;
      bt_spp_stop();
    }
#endif
  }
  else if (choice == "ios")
  {
#ifdef BTSPPENABLED
    if (stored_preferences.btspp_active)
    {
      // Hack: since disabling spp and enabling BLE crashes the stack and restart ESP32, we store the current configuration for a restart
      stored_preferences.btspp_active = false;
      stored_preferences.ble_active = true;
      StoreNVMPreferences(true);
      webserver.send(200, html_text, generate_html_body(F("<b>RESTARTING</b> to set optimal configuration for Harry Lap Timer on iOS devices:</p><p>Settings have been already saved</p>")));
      delay(1000);
      ESP.restart();
      // This is how it should be:
      // stored_preferences.btspp_active = false;
      // bt_spp_stop();
    }
#endif
    if (stored_preferences.ble_active == false)
    {
      ble_start();
      stored_preferences.ble_active = true;
    }
  }
  String message((char *)0);
  message.reserve(70);
  message += F("Set optimal configuration for Harry Lap Timer on:");
  message += choice;
  message += FPSTR(savecfg);
  webserver.send(200, html_text, generate_html_body(message));
}

void handle_wifi_mode()
{
  String strwifimode = webserver.pathArg(0);
  log_i("Start WiFi in %s mode", strwifimode);
  if (strwifimode == "sta")
  {
    webserver.send(200, html_text, generate_html_body(F("WiFi STATION mode started, trying to connect to a know Access Point. <br>Reconnect in a few seconds")));
    stored_preferences.wifi_mode = WIFI_STA;
    wifi_STA();
  }
  else if (strwifimode == "ap")
  {
    log_i("Start WiFi in AP mode");
    webserver.send(200, html_text, generate_html_body(String("WiFi Access Point mode started. <br>Reconnect in a few seconds to AP:" + String(ap_ssid))));
    stored_preferences.wifi_mode = WIFI_AP;
    wifi_AP();
  }
  else
  {
    log_e("Incorrect '%s' mode for WiFi");
    webserver.send(400, html_text, generate_html_body(String("Wrong WiFi mode selected")));
  }
}
void handle_wifioff()
{
  webserver.send(200, html_text, generate_html_body(F("Please confirm disabling WiFi - you will need to power cycle the unit or use the BOOT button to re-enable it<form action='/wifioff' method='post'><input type='submit' value='Turn WiFi OFF'></form>")));
}
void handle_wifioff_post()
{
  log_i("WiFi OFF");
  webserver.send(200, html_text, generate_html_body(F("Powering off WiFi - power cycle the unit or use BOOT to re-enable it")));
  delay(1000);
  wifi_OFF();
}

void handle_restart()
{
  webserver.send(200, html_text, generate_html_body(F("Please confirm <form action='/restart' method='post'><input type='submit' value='Restart'></form>")));
}
void handle_restart_execute()
{
  log_i("Restarting");
  webserver.send(200, html_text, generate_html_body(F("Restarting device")));
  delay(1000);
  ESP.restart();
  delay(1000);
}
void handle_status()
{
  log_d("Logging status to web");
  String message((char *)0);
  message.reserve(1400);
  message += F("<h1>Device status</h1><p>WiFi Mode: ");

  if (stored_preferences.wifi_mode == WIFI_AP)
  {
    message += F("AP<br>Access point name: ");
    message += ap_ssid;
  }
  else
  {
    message += F("STA<br>Connected to AP: ");
    message += WiFi.SSID();
  }

  message += F("<br>IP: ");
  message += MyIP.toString();
  message += F("<br>TCP Port for NMEA repeater: ");
  message += (stored_preferences.nmeaTcpServer ? F("ON") : F("OFF"));
  message += F("<br> Port:");
  message += String(NMEAServerPort);
  message += F("<br>u-center URL: tcp://");
  message += MyIP.toString();
  message += String(":");
  message += String(NMEAServerPort);
#ifdef NUMERICAL_BROADCAST_PROTOCOL
  message += F("<br>Port for NBP: ") + String(NBPServerPort);
#else
  message += F("<p>NBP n/a in this firmware ");
#endif
#ifdef BLEENABLED
  message += (stored_preferences.ble_active ? F("<p>BLE Enabled") : F("<p>BLE Not Enabled"));
  message += F("<br>BLE Name: ");
  message += ble_device_id;
  message += F("<br>BLE Service: ");
  message += String(BLE_SERVICE_UUID, HEX);
  message += F("<br>BLE Characteristic: ");
  message += String(BLE_CHARACTERISTIC_UUID, HEX);
#else
  message += F("<p>BLE n/a in this firmware ");
#endif
#ifdef BTSPPENABLED
  message += (stored_preferences.btspp_active ? F("<p>BT-SPP Enabled") : F("<p>BT-SPP Not Enabled"));
  message += F("<br>BT Name: ");
  message += ble_device_id;
#else
  message += F("<p>BT-SPP n/a in this firmware ");
#endif
  message += F("<p>UART Baud: ");
  message += stored_preferences.gps_baud_rate;
  message += F("<br>Refresh rate: ");
  message += stored_preferences.gps_rate + String("Hz");
  message += F("<br>Main Talker ID: ");
  message += ((stored_preferences.trackaddict || stored_preferences.racechrono || stored_preferences.racetime)? F("GPS ") : F("System Specific (default)"));
  message += F("<br>Restrict SV to GPS: ");
  message += (stored_preferences.racechrono? F("Yes ") : F("No (default)"));
  message += F("<br>NMEA GxGSV: ");
  message += (stored_preferences.nmeaGSV ? String("ON") : String("OFF"));
  message += F("<br>NMEA GxGSA: ");
  message += (stored_preferences.nmeaGSA ? String("ON") : String("OFF"));
  message += F("<br>NMEA GxGBS: ");
  message += (stored_preferences.nmeaGBS ? String("ON") : String("OFF"));
  message += F("<br>NMEA GxGLL: ");
  message += (stored_preferences.nmeaGLL ? String("ON") : String("OFF"));
  message += F("<br>NMEA GxVTG: ");
  message += (stored_preferences.nmeaVTG ? String("ON") : String("OFF"));
  message += F("<br>GPS PowerSave: ");
  message += (gps_powersave ? String(" ON") : String("OFF"));
#ifdef UPTIME
  message += F("<br><br>Uptime: ");
  message += String(uptime_formatter::getUptime());
#endif
  message += F("<br>Version: ");
  message += String(BONO_GPS_VERSION);
  message += F("<br>Build date: ");
  message += String(BONOGPS_BUILD_DATE);
#ifdef ENABLE_OTA
  message += F("<br>OTA Built-in");
#else
  message += F("<br>OTA n/a in this firmware ");
#endif
  message += F("<p>Board: ");
  message += String(BOARD_NAME);
  message += F("<p>Total heap: ");
  message += String(ESP.getHeapSize());
  message += F("<br>Free heap: ");
  message += String(ESP.getFreeHeap());
  message += F("<br>Total PSRAM: ");
  message += String(ESP.getPsramSize());
  message += F("<br>Free PSRAM: ");
  message += String(ESP.getFreePsram());
  message += F("<p>SDK version: ");
  message += ESP.getSdkVersion();
  message += F("<br>Chip Revision: ");
  message += String(ESP.getChipRevision());
  message += F("<br>CPU Freq: ");
  message += String(ESP.getCpuFreqMHz());

  webserver.send(200, html_text, generate_html_body(message));
}
void handle_clients()
{
  log_i("Logging clients status to web");
  String message((char *)0);
  message.reserve(1000);
  message += F("NMEA TCP/IP client");
  if (!NMEARemoteClient.connected())
  {
    message += " not";
  }

#ifdef NUMERICAL_BROADCAST_PROTOCOL
  message += F(" connected<br>NBP TCP/IP client");
  if (NBPRemoteClient.connected())
  {
    message += " not";
  }
#endif
#ifdef BLEENABLED
  message += F(" connected<br>BLE device");
  if (!ble_deviceConnected)
  {
    message += " not";
  }
#endif
#ifdef BTSPPENABLED
  message += F(" connected<br>BT-SPP device");
  if (!bt_deviceConnected)
  {
    message += " not";
  }
#endif
  message += F(" connected");
  webserver.send(200, html_text, generate_html_body(message));
}

const char WEBPORTAL_SAVECONFIG[] PROGMEM = "<article><form action='/savewificreds' method='post'>WiFi SSID<br><input style='background: none; color: black' name='ssid' maxlength='32'><p>WiFi Key<br><input style='background: none; color: black' type='password' name='key' maxlength='64'><p><input type='submit' value='Save'></form></article>";
void handle_saveconfig_wifi_creds()
{
  log_d("Form to save wifi credentials");
  webserver.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webserver.send_P(200, html_text, WEBPORTAL_HEADER);
  webserver.sendContent(generate_html_header(true));
  webserver.sendContent_P(WEBPORTAL_SAVECONFIG);
  webserver.sendContent_P(WEBPORTAL_FOOTER);
}

void handle_saveconfig_wifi_creds_post()
{
  String wifissid = webserver.arg("ssid");
  String wifikey = webserver.arg("key");
  // TODO Check inputs
  log_i("WiFi SSID %s", wifissid);
  log_d("WiFi Key %s", wifikey);
  wifissid.toCharArray(stored_preferences.wifi_ssid, WIFI_SSID_MAXLEN);
  wifikey.toCharArray(stored_preferences.wifi_key, WIFI_KEY_MAXLEN);
  // TODO check if writes were ok
  StoreNVMPreferencesWiFiCreds();
  // TODO check is storing went fine
  webserver.send(200, html_text, generate_html_body(F("<article>New WiFi SSID and Key stored <p><a href='/wifi/sta'>Try them</a></article>")));
}

void handle_saveconfig()
{
  log_i("Storing preferences intermediate menu");
  String message((char *)0);
  message.reserve(1000);
  message += F("<h1>All configuration values</h1><p>Save <a href='/savecfg/wifi'>all config <b>including</b> WiFi </a></p><p>Save <a href='/savecfg/nowifi'>all but WiFi-");
  if (stored_preferences.wifi_mode == WIFI_AP)
  {
    message += "AP";
  }
  else
  {
    message += "STA";
  }
  message += F(" mode</a></p>\n<h1>Save only a specific WiFi mode</h1><p>Save <a href='/savewifi/sta'>WiFi client mode WiFi-STA </a></p><p>Save <a href='/savewifi/ap'>WiFi Access Point mode WiFi-AP</a></p>");
  webserver.send(200, html_text, generate_html_body(message));
}
void handle_saveconfig_wifi()
{
  String withorwout = webserver.pathArg(0);
  log_i("Save config %s ", withorwout);
  if (withorwout == "wifi")
  {
    StoreNVMPreferences(true);
  }
  else
  {
    StoreNVMPreferences(false);
  }
  String message = "Preferences [";
  message += withorwout;
  message += "] stored";
  webserver.send(200, html_text, generate_html_body(message));
}

void handle_saveconfig_wifimode()
{
  String strwifimode = webserver.pathArg(0);
  log_i("Save WiFi  %s mode", strwifimode);
  if (strwifimode == "sta")
    StoreNVMPreferencesWiFi("WIFI_STA");
  if (strwifimode == "ap")
    StoreNVMPreferencesWiFi("WIFI_AP");
  String message = F("Stored preference for wifi :");
  message += strwifimode;
  webserver.send(200, html_text, generate_html_body(message));
}

void handle_NotFound()
{
  webserver.send(404, html_text, generate_html_body(F("<h1> 404 Not found</h1><p>The requested resource was not found on this server.</p>")));
}
void WebConfig_start()
{
  webserver.on("/", handle_menu);
  webserver.on("/css", handle_css);
  webserver.on("/rate/{}hz", handle_rate);
  webserver.on("/gsa/{}", handle_gsa);
  webserver.on("/gsv/{}", handle_gsv);
  webserver.on("/gbs/{}", handle_gbs);
  webserver.on("/sv/{}", handle_svchannel);
  webserver.on("/tcpserver/{}", handle_tcpserver);
#ifdef BTSPPENABLED
  webserver.on("/trackaddict/{}", handle_trackaddict);
  webserver.on("/racechrono/android", handle_racechrono_android);
  webserver.on("/racetime/android", handle_racetime_android);
#endif
  webserver.on("/hlt/{}", handle_hlt);
  webserver.on("/wifi/{}", handle_wifi_mode);
  webserver.on("/restart", HTTP_GET, handle_restart);
  webserver.on("/restart", HTTP_POST, handle_restart_execute);
  webserver.on("/wifioff", HTTP_GET, handle_wifioff);
  webserver.on("/wifioff", HTTP_POST, handle_wifioff_post);
  webserver.on("/savecfg", handle_saveconfig);
  webserver.on("/savecfg/{}", handle_saveconfig_wifi);
  webserver.on("/savewifi/{}", handle_saveconfig_wifimode);
  webserver.on("/savewificreds", HTTP_GET, handle_saveconfig_wifi_creds);
  webserver.on("/savewificreds", HTTP_POST, handle_saveconfig_wifi_creds_post);
  webserver.on("/preset", handle_preset);
  webserver.on("/powersave/{}", handle_powersave);
  webserver.on("/status", handle_status);
  webserver.on("/clients", handle_clients);
  webserver.on("/baud/{}", handle_baudrate);
#ifdef BLEENABLED
  webserver.on("/ble/{}", handle_ble);
#endif
#ifdef BTSPPENABLED
  webserver.on("/btspp/{}", handle_btspp);
#endif
#ifdef TASK_SCHEDULER
  webserver.on("/poll/gsagsv/{}", handle_pollgsagsv);
#endif
  webserver.onNotFound(handle_NotFound);
  webserver.begin();
#ifdef MDNS_ENABLE
  MDNS.addService("http", "tcp", 80);
#endif
}

void WebConfig_stop()
{
  webserver.close();
}

/********************************

  OTA

* ******************************/
#ifdef ENABLE_OTA
#ifndef OTA_AVAILABLE
#define OTA_AVAILABLE 300 // 300 seconds of OTA running
#endif

#ifdef TASK_SCHEDULER
Task tOTA(1000, OTA_AVAILABLE, &handle_OTA, &ts, false);
#endif

void handle_OTA()
{
  ArduinoOTA.handle();
}
void OTA_start()
{
  ArduinoOTA.setHostname(BONOGPS_MDNS);
  ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        log_i("Stop GPS serial logger", type);
        gpsPort.end();
        tLedBlink.setInterval(50);
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
        if (error == OTA_AUTH_ERROR)
          Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
          Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
          Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
          Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)
          Serial.println("End Failed");
      });
  ArduinoOTA.begin();
  tOTA.enable();
}

void OTA_stop() {
  ArduinoOTA.end();
  tOTA.disable();
}
#endif

/********************************

  BLE

* ******************************/

#ifdef BLEENABLED

#define DEVINFO_UUID (uint16_t)0x180a
#define DEVINFO_MANUFACTURER_UUID (uint16_t)0x2a29 //0x2A29 MfgName utf8s
#define DEVINFO_NAME_UUID (uint16_t)0x2a24         //0x2A24 ModelNum utf8s
#define DEVINFO_SERIAL_UUID (uint16_t)0x2a25       //0x2A25 SerialNum utf8s
#define DEVINFO_FWREV_UUID (uint16_t)0x2a26        //0x2A26 FirmwareRev utf8s

#define SERVICE_BATTERY_SERVICE_UUID (uint16_t)0x180F            // not used
#define CHARACTERISTIC_BATTERY_LEVEL_UUID (uint16_t)0x2A19       // not used
#define CHARACTERISTIC_BATTERY_LEVEL_STATE_UUID (uint16_t)0x2A1B // not used
#define CHARACTERISTIC_BATTERY_POWER_STATE_UUID (uint16_t)0x2A1A // not used
//#define HardwareRev "0001"    //0x2A27 utf8s
//#define SystemID "000001"     //0x2A23 uint40

NimBLEServer *pServer = NULL;
NimBLECharacteristic *pCharacteristic = NULL;
uint16_t ble_mtu;

// Build BTLE messages here
uint8_t gps_currentmessage[MAX_UART_BUFFER_SIZE]; // hold current buffer
uint8_t gps_currentchar = '$';                    // hold current char
int gps_message_pointer = 0;                      //pointer

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(NimBLEServer *pServer, ble_gap_conn_desc *desc)
  {
    ble_deviceConnected = true;
    ble_client = String(NimBLEAddress(desc->peer_ota_addr).toString().c_str());
    log_i("BLE Client address: %s", ble_client);
  };
  void onDisconnect(NimBLEServer *pServer)
  {
    ble_deviceConnected = false;
  }
};

void ble_start()
{
#ifdef BTSPPENABLED
  bt_spp_stop();
#endif
  /// Create the BLE Device
  NimBLEDevice::init(ble_device_id);
  /** Optional: set the transmit power, default is 3db */
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);                             /** +9db */
  NimBLEDevice::setPower(ESP_PWR_LVL_P9, ESP_BLE_PWR_TYPE_ADV);       /** +9db */
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
  pChar->setValue(BONO_GPS_VERSION);
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

void ble_stop()
{
  if (NimBLEDevice::getInitialized())
  {
    NimBLEDevice::stopAdvertising();
    NimBLEDevice::deinit(true);
  }
}
#endif

#ifdef BTSPPENABLED
/*********************************************************************

  Bluetooth Classic

*******************************************************************/
void bt_spp_start()
{
#ifdef BLEENABLED
  // disable BTLE
  ble_stop();
#endif
  SerialBT.begin(ble_device_id); //Bluetooth device name
  SerialBT.register_callback(bt_callback);
  log_i("Started BT-SPP port");
}
void bt_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
  if (event == ESP_SPP_SRV_OPEN_EVT)
  {
    log_d("BT-SPP Client Connected");
#ifdef BLEENABLED
    ble_deviceConnected = false;
#endif
    bt_deviceConnected = true;
  }
  if (event == ESP_SPP_CLOSE_EVT)
  {
    log_d("BT-SPP Client closed connection");
    bt_deviceConnected = false;
  }
}
bool bt_spp_stop()
{
  // Prior to version 1.1, this was
  // SerialBT.end();
  // The below procedure is more stable for restarts, yet not working 100% of the time still
  if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE)
  {
    log_i("bt stopped");
    return true;
  }
  if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED)
  {
    log_i("bt is enabled");
    if (esp_bt_controller_disable())
    {
      log_e("BT Disable failed");
      return false;
    }
    while (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED)
      ;
  }
  if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED)
  {
    log_i("is inited");
    if (esp_bt_controller_deinit())
    {
      log_e("BT deint failed");
      return false;
    }
    while (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED)
      ;
    return true;
  }
  return false;
}
#endif

/*********************************************************************

  SETUP

*******************************************************************/
void gps_initialize_settings()
{
  // GPS Connection
  // if preferences have a value <> than the one stored in the GPS, connect+switch
  log_d("Start UART connection on RX pin %d TX pin %d and autobaudrate",RX2, TX2);
  gpsPort.begin(0,SERIAL_8N1, RX2, TX2, false, 10000UL);
  if (gpsPort.baudRate()>0) {
    log_d("Connected with autobaudrate at %u on RX pin %d TX pin %d ",gpsPort.baudRate(), RX2, TX2);
  } else {
    log_e("Can't auto find BAUD rate on RX pin %d TX pin %d , forcing %u",RX2, TX2, GPS_STANDARD_BAUD_RATE);
    // TODO: enable pulsing error on the LED to signal the user that something is bad
    gpsPort.begin(GPS_STANDARD_BAUD_RATE,SERIAL_8N1, RX2, TX2);
  }
   
  if (gpsPort.baudRate() != stored_preferences.gps_baud_rate)
  {
    log_i("Re-Connecting to GPS at updated %u", stored_preferences.gps_baud_rate);
    switch_baudrate(stored_preferences.gps_baud_rate);
  }

  gpsPort.setRxBufferSize(UART_BUFFER_SIZE_RX);
  delay(50);

  if (stored_preferences.nmeaGSV)
  {
    push_gps_message(UBLOX_GxGSV_ON, sizeof(UBLOX_GxGSV_ON));
  }
  else
  {
    push_gps_message(UBLOX_GxGSV_OFF, sizeof(UBLOX_GxGSV_OFF));
  }

  if (stored_preferences.nmeaGSA)
  {
    push_gps_message(UBLOX_GxGSA_ON, sizeof(UBLOX_GxGSA_ON));
  }
  else
  {
    push_gps_message(UBLOX_GxGSA_OFF, sizeof(UBLOX_GxGSA_OFF));
  }

  if (stored_preferences.nmeaGBS)
  {
    push_gps_message(UBLOX_GxGBS_ON, sizeof(UBLOX_GxGBS_ON));
  }
  else
  {
    push_gps_message(UBLOX_GxGBS_OFF, sizeof(UBLOX_GxGBS_OFF));
  }

#ifdef BTSPPENABLED
  // apply settings required by TrackAddict
  if (stored_preferences.trackaddict)
  {
    gps_enable_trackaddict();
  }
  else if (stored_preferences.racechrono)
  {
    gps_enable_racechrono();
  }
  else if (stored_preferences.racetime)
  {
    gps_enable_racetime();
  }  else
  {
    push_gps_message(UBLOX_INIT_CHANNEL_ALL, sizeof(UBLOX_INIT_CHANNEL_ALL));
  }
#endif

  switch (stored_preferences.gps_rate)
  {
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

void setup()
{
  SerialMonitor.begin(LOG_BAUD_RATE); // Initialize the serial monitor
  delay(200);
  log_d("ESP32 SDK: %s", ESP.getSdkVersion());
  log_d("Arduino sketch: %s", __FILE__);
  log_d("Compiled on: %s", __DATE__);

  //set led pin as output
  pinMode(LED_BUILTIN, OUTPUT);
  tLedBlink.setInterval(100);
  tLedBlink.enable();

  // Generate device name
  chip = (uint16_t)((uint64_t)ESP.getEfuseMac() >> 32);
  sprintf(ap_ssid, "%s-%04X", BONOGPS_AP, chip);
#if defined(BLEENABLED) || defined(BTSPPENABLED)
  sprintf(ble_device_id, "%s-%04X", BLE_DEVICE_ID, chip);
#endif

  // Read desired status from NVM
  prefs.begin(BONOGPS_MDNS);
  ReadNVMPreferences();

  if (stored_preferences.ble_active)
  {
    // start BTLE
#ifdef BLEENABLED
    ble_start();
#endif
#ifdef BTSPPENABLED
  }
  else if (stored_preferences.btspp_active)
  {
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
  // WiFi.setSleep(false);

  // WifiAP or STA will also start the NMEA server on port 1818, if the service is turned on (starting with v0.4)
  switch (stored_preferences.wifi_mode)
  {
  case WIFI_AP:
    wifi_AP();
    break;
  case WIFI_STA:
    wifi_STA();
    break;
  case WIFI_OFF:
    break;
  default:
    wifi_AP();
    break;
  }

  gps_initialize_settings();

  log_d("Total heap: %d", ESP.getHeapSize());
  log_d("Free heap: %d", ESP.getFreeHeap());
  log_d("Total PSRAM: %d", ESP.getPsramSize());
  log_d("Free PSRAM: %d", ESP.getFreePsram());
}

void loop()
{
  // check UART for data
  if (gpsPort.available())
  {
    size_t len = gpsPort.available();
    uint8_t sbuf[len];
    gpsPort.readBytes(sbuf, len);
    // log max buffer size
    if (len > max_buffer)
    {
      max_buffer = len;
      log_w("New max buffer: %d", max_buffer);
    }
    if (wifi_connected && NMEARemoteClient && NMEARemoteClient.connected())
    {
      //push UART data to the TCP client
      NMEARemoteClient.write(sbuf, len);
    }

#ifdef NUMERICAL_BROADCAST_PROTOCOL
    if (wifi_connected && NBPRemoteClient && NBPRemoteClient.connected())
    {
      // send GPS data to NeoGPS processor for parsing
      for (int i = 0; i < len; i++)
      {
        if (gps.decode((char)sbuf[i]) == NMEAGPS::DECODE_COMPLETED)
        {
          log_d("Fix available, size: %d", sizeof(gps.fix()));
          my_fix = gps.fix();

          NBPRemoteClient.print("*NBP1,ALLUPDATE,");
          NBPRemoteClient.print(my_fix.dateTime);
          NBPRemoteClient.print(".");
          NBPRemoteClient.println(my_fix.dateTime_ms());
          if (my_fix.valid.location)
          {
            NBPRemoteClient.print("\"Latitude\",\"deg\":");
            NBPRemoteClient.println(my_fix.latitude(), 12);
            NBPRemoteClient.print("\"Longitude\",\"deg\":");
            NBPRemoteClient.println(my_fix.longitude(), 12);
            NBPRemoteClient.print("\"Altitude\",\"m\":");
            NBPRemoteClient.println(my_fix.altitude());
          }
          if (my_fix.valid.heading)
          {
            NBPRemoteClient.print("\"Heading\",\"deg\":");
            NBPRemoteClient.println(my_fix.heading());
          }
          if (my_fix.valid.speed)
          {
            NBPRemoteClient.print("\"Speed\",\"kmh\":");
            NBPRemoteClient.println(my_fix.speed_kph());
          }
          // NBPRemoteClient.print("\"Accuracy\",\"m\":");
          NBPRemoteClient.println("#");
          NBPRemoteClient.print("@NAME:");
          NBPRemoteClient.println(ap_ssid);
        }
      }
    }
    else
    {
      NBPCheckForConnections();
    }
#endif

#ifdef BTSPPENABLED
    if (bt_deviceConnected && stored_preferences.btspp_active)
    {
      // we have BT-SPP active
      SerialBT.write(sbuf, len);
    }
#endif

#ifdef BLEENABLED
    if (ble_deviceConnected && stored_preferences.ble_active)
    {
      // we have BLE active
      for (int i = 0; i < len; i++)
      {
        if (gps_message_pointer > MAX_UART_BUFFER_SIZE - 3)
        {
          log_e("BLE Buffer saturated, resetting ");
          gps_currentmessage[0] = (uint8_t) '$';
          gps_currentmessage[1] = (uint8_t) '\0';
          gps_message_pointer = 1;
        }
        if (sbuf[i] == 36) // $ char
        {
          gps_currentmessage[gps_message_pointer++] = (uint8_t) '\r';
          gps_currentmessage[gps_message_pointer++] = (uint8_t) '\n';
          gps_currentmessage[gps_message_pointer++] = (uint8_t) '\0';
          // There is a new message -> Notify BLE of the changed value if connected and size says it is likely to be valid
          if (gps_message_pointer > MIN_NMEA_MESSAGE_SIZE)
          {
            pCharacteristic->setValue(gps_currentmessage, gps_message_pointer);
            pCharacteristic->notify();
            delay(1);
          }
          gps_currentmessage[0] = (uint8_t) '$';
          gps_currentmessage[1] = (uint8_t) '\0';
          gps_message_pointer = 1;
        }
        else
        {
          // Accumulate more of the GPS message
          gps_currentmessage[gps_message_pointer++] = sbuf[i];
        }
      }
    }
#endif
  }

  if (wifi_connected)
  {
    // handle sending instructions to GPS via uBlox center connected with TCP/IP
    if (NMEARemoteClient.connected() && NMEARemoteClient.available())
    {
      gpsPort.write(NMEARemoteClient.read());
    }
    else
    {
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
