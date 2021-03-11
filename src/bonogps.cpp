/******************************************************************************

  BonoGPS: connect a GPS to mobile Apps to track lap times
  More info at https://github.com/renatobo/bonogps
  Renato Bonomini https://github.com/renatobo

******************************************************************************/

// For PlatformIO we need to include the Arduino framework
#include <Arduino.h>
// load PINout definitions from this header file
#include "bonogps_board_settings.h"

/*
 Enable or disable compiling features
 */

#define BTSPPENABLED // add BT-SPP stack, remove if unnecessary as it uses quite a bit of flash space
#define BLEENABLED   // add BLE stack, remove if unnecessary as it uses quite a bit of flash space
/* Enable here if you are using Arduino IDE, otherwise use -DENABLE_OTA in platformio
#define ENABLE_OTA     // add code for OTA, to be enabled only when developing
*/
// you should not disable these unless there is a problem with the feature
#define UPTIME       // add library to display for how long the device has been running
#define MDNS_ENABLE    // Enable or disable mDNS - currently not working in all conditions. Small memory save to be worth removing
#define TASK_SCHEDULER // enable adv task scheduler. Small memory save to be worth removing
#define BUTTON         // Enable changing WIFI_STA and WIFI_AP via Boot button. Small memory save to be worth removing - Which button is defined below
#define SHORT_API      // URLs for commands such as enable/disable/change rate respond with a short json answer instead of a full web page

// Configure names and PIN's used for interfacing with external world
#define DEVICE_MANUFACTURER "https://github.com/renatobo"
#define DEVICE_NAME "Bono GPS"
#define BONOGPS_BUILD_DATE __TIMESTAMP__
// BONOGPS_FIRMWARE_VER is used in BLE "Firmware" data and in the /status page of web configuration
#ifdef GIT_REV
// if code is under Git revision and compiled from platformio using the additional git_rev_macro.py script, GIT_REV is set
#define BONOGPS_FIRMWARE_VER GIT_REV
#else
// the following define is needed to display version when building this with the Arduino IDE
#define BONOGPS_FIRMWARE_VER "v1.2.2"
#endif
// GIT_REPO is used to build links to online software release notes and documentation
#ifndef GIT_REPO
#define GIT_REPO "renatobo/bonogps"
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

// Define configuration of Bluetooth Low Energy
#define BLE_DEVICE_ID "BonoGPS"
#define BLE_SERVICE_UUID (uint16_t)0x1819        // SERVICE_LOCATION_AND_NAVIGATION_UUID
#define BLE_CHARACTERISTIC_UUID (uint16_t)0x2A67 // CHARACTERISTIC_LOCATION_AND_SPEED_CHARACTERISTIC_UUID
#define BLE_MTU 185

// GPS port on UART2
#define gpsPort Serial2
// How much time before autobauding times out
#define GPS_UART_TIMEOUT 10000UL
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
#else
#ifndef LED_WIFI
#define LED_WIFI LED_BUILTIN
#endif // #ifndef LED_WIFI
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

// if we have both, we can define a task to check period OTA requests
#if defined(ENABLE_OTA) && defined(TASK_SCHEDULER)
// we will define its charateristics later
Task tOTA;
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
bool ble_deviceConnected = false;
void ble_start();
void ble_stop();
String ble_client_address;
#endif

#if defined(BLEENABLED) || defined(BTSPPENABLED)
char ble_device_id[MAX_AP_NAME_SIZE];
#endif

// Respond to button
#ifdef BUTTON
// remove unneeded feature to trim down library size
#define EASYBUTTON_DO_NOT_USE_SEQUENCES
#define WIFI_ENABLE_STA_DURATION 2000
#include <EasyButton.h>
// Instance of the button to switch wifi mode
EasyButton button(WIFI_MODE_BUTTON);
#endif

#if defined(SHOWBATTERY)
// Right now this is tailored to the TP4054 charging available on GPIO35 for LOLIN D32 Pro
// check https://www.youtube.com/watch?t=88&v=yZjpYmWVLh8&feature=youtu.be for how
// This could be extended to other solutions as needed

float ReadBatteryVoltage()
{
  return analogRead(GPIO_BATTERY) / 4096.0 * 7.445;
}

uint8_t LiPoChargePercentage(float voltage)
{
  // use as LiPoChargePercentage(ReadBatteryVoltage())
  // LOLIN D32 PRO uses a TP4054 so max V should be 4.2V https://datasheetspdf.com/pdf/1090540/NanJingTopPower/TP4054/1
  // 0% <= 3.3
  // 100% > 4.2
  uint8_t returnvalue;
  float normalizedcharge = ( voltage - 3.3 ) / (4.2 - 3.3) * 100;
  if (normalizedcharge > 100) 
  {
    returnvalue = 100;
  } else if (normalizedcharge <0)
  {
    returnvalue = 0;
  } else
  {
    returnvalue = (uint8_t) normalizedcharge;
  }
  return returnvalue;
}
#endif //#if defined(SHOWBATTERY)

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

void poweroff() {
#ifdef BTSPPENABLED
  bt_spp_stop();
#endif
#ifdef BLEENABLED
  ble_stop();
#endif
  wifi_OFF();

  // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/sleep_modes.html
  // rtc_gpio_isolate(GPIO_NUM_12);
  esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  esp_deep_sleep_start();
}
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
const char UBLOX_INIT_MAINTALKER_GP[] PROGMEM = {0xB5, 0x62, 0x06, 0x17, 0x14, 0x00, 0x00, 0x41, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x78};
// use GP as Main talker and GSV Talker ID and restrict to GPS SVs ony - this is needed for RaceChrono
const char UBLOX_INIT_MAINTALKER_GP_GPSONLY[] PROGMEM = {0xB5, 0x62, 0x06, 0x17, 0x14, 0x00, 0x10, 0x41, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x88, 0xB8};
// set gps port BAUD rate
const char UBLOX_BAUD_57600[] PROGMEM = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xE1, 0x00, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDA, 0xA9};
const char UBLOX_BAUD_38400[] PROGMEM = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x96, 0x00, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8F, 0x70};
const char UBLOX_BAUD_115200[] PROGMEM = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBC, 0x5E};
// power saving
// enable power saving mode for 1800 or 3600 seconds
// UBX-RXM-PMREQ request
const char UBLOX_PWR_SAVE_30MIN[] PROGMEM = {0xB5, 0x62, 0x02, 0x41, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x77, 0x1B, 0x00, 0x02, 0x00, 0x00, 0x00, 0xE8, 0x00, 0x00, 0x00, 0x0F, 0xF6};
const char UBLOX_PWR_SAVE_1HR[] PROGMEM = {0xB5, 0x62, 0x02, 0x41, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xEE, 0x36, 0x00, 0x02, 0x00, 0x00, 0x00, 0xE8, 0x00, 0x00, 0x00, 0xE1, 0x21};
const char UBLOX_PWR_OFF[] PROGMEM = { 0xB5, 0x62, 0x02, 0x41, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x5D, 0x4B};
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
    delay(250);
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
  gpsPort.end();
  delay(250);
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
    string_wifi_mode = "WIFI_OFF";
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
      string_wifi_mode = "WIFI_OFF";
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

const uint NMEAServerPort = NMEA_TCP_PORT;
WiFiServer NMEAServer(NMEAServerPort);
WiFiClient NMEARemoteClient;

bool wifi_ledon;
void wifiled_blink()
{
  //toggle state
  wifi_ledon = !wifi_ledon;
  digitalWrite(LED_WIFI, (wifi_ledon ? HIGH : LOW)); // set pin to the opposite state
}

#ifdef TASK_SCHEDULER
Task tLedWiFiBlink(0, TASK_FOREVER, &wifiled_blink, &ts, false);
#ifdef LED_ACTIVE_EXTERNAL
bool active_ledon;
#define LEDPWMFREQ  5000
#define LEDCHANNEL (uint8_t) 0
#define LEDPWMRESOLUTION (uint8_t ) 8
void activeled_blink()
{
  //toggle state
  active_ledon = !active_ledon;
  uint8_t highvalue=30;
  if (gps_powersave) {
    highvalue=0;
  }
  ledcWrite(LEDCHANNEL, (active_ledon ? highvalue : 5));
  // digitalWrite(LED_ACTIVE_EXTERNAL, (active_ledon ? HIGH : LOW)); // set pin to the opposite state
}
// if we need an external led to show status, use a timer
Task tLedActiveBlink(0, TASK_FOREVER, &activeled_blink, &ts, false);
#endif
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
      log_d("NMEA TCP Connection accepted from client: %s", NMEARemoteClient.remoteIP().toString().c_str());
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

// Start STATION mode to connect to a well-known Access Point
void wifi_STA()
{
  if (stored_preferences.nmeaTcpServer)
    stop_NMEA_server();

  log_i("Stop Web Portal");
  WebConfig_stop();

  // WiFi Access
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  // start ticker_wifi with 50 ms because we start in STA mode and try to connect
#ifdef TASK_SCHEDULER
  log_d("Start rapid blinking");
  tLedWiFiBlink.setInterval(50);
  tLedWiFiBlink.enable();
#endif

  // connect to STA mode with stored passwords here
  WiFi.begin(stored_preferences.wifi_ssid, stored_preferences.wifi_key);
  wifi_connected = false;

  int times = 0;
  // Another press will move to wifi_OFF
  #ifdef BUTTON
    button.onPressed(wifi_OFF);
  #endif
  while (WiFi.status() != WL_CONNECTED && times < 50)
  {
    delay(50);
    log_i("Connecting to WiFi %s , trial %d", stored_preferences.wifi_ssid, times++);
    #ifdef BUTTON
    // If we see WiFi failing and we want to change mode on the fly, we need to read button status here
    button.read();
    #endif
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    log_i("Connected to SSID %s", stored_preferences.wifi_ssid);
#ifdef TASK_SCHEDULER
    tLedWiFiBlink.setInterval(250);
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
    MyIP = WiFi.localIP();
    stored_preferences.wifi_mode = WIFI_STA;
  }
  else
  {
    log_e("Could not connect to SSID %s, reverting to AP mode", stored_preferences.wifi_ssid);
      // Recover to wifi_STA when button is pressed
      #ifdef BUTTON
      button.onPressed(wifi_OFF);
      #endif
      wifi_AP();
  }
}

void wifi_AP()
{
  // WiFi Access
  if (stored_preferences.nmeaTcpServer)
    stop_NMEA_server();

  log_i("Stop Web Portal");
  WebConfig_stop();
  
  // explicitly set mode, esp defaults to STA+AP
  WiFi.mode(WIFI_AP); 
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
  log_d("Start blinking slowly 1sec frequency");
  tLedWiFiBlink.setInterval(1000);
  tLedWiFiBlink.enable();
#endif

  // WLAN Server for GNSS data
  if (stored_preferences.nmeaTcpServer)
    start_NMEA_server();

#ifdef BUTTON
  button.onPressed(wifi_OFF);
#endif
  stored_preferences.wifi_mode = WIFI_AP;
}

void wifi_OFF()
{

#ifdef TASK_SCHEDULER
  log_d("Flash blinking");
  tLedWiFiBlink.setInterval(50);
#endif

#ifdef ENABLE_OTA
  log_i("Stop OTA service");
  OTA_stop();
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
  tLedWiFiBlink.setInterval(0);
  tLedWiFiBlink.disable();
  wifi_ledon = HIGH;
  wifiled_blink();
#endif

#ifdef BUTTON
  button.onPressed(wifi_AP);
#endif

  stored_preferences.wifi_mode = WIFI_OFF;
}

/********************************

   Web Configuration portal

 * ******************************/
// change from standard 1436 to a minimal 100
// #define HTTP_UPLOAD_BUFLEN 100
#include <WebServer.h>
#include <uri/UriBraces.h>
WebServer webserver(80);

const char html_text[] PROGMEM = "text/html";
const char html_css[] PROGMEM = "text/css";
const char text_json[] PROGMEM = "application/json";
const char json_ok[] PROGMEM = "{'status':'ok'}";
const char json_error[] PROGMEM = "{'status':'error'}";
// The following variable is in a separate file generated from its source but stored in minified format
// run generate_css.sh 
#ifdef SHOWBATTERY
// add the css portion for the battery gauge
#include "bonogps_css_base_battery.h"
#else 
#include "bonogps_css_base.h"
#endif
const char WEBPORTAL_HEADER[] PROGMEM = "<!DOCTYPE html>\n<html lang='en'>\n\t<head>\n\t\t<title>Bono GPS</title>\n\t\t<meta charset='utf-8'>\n\t\t<meta name='viewport' content='width=device-width, initial-scale=1'>\n\t\t<link rel='stylesheet' href='/css'>\n\t</head>\n<body>\n<script>function Select(e){fetch('/'+e).then(e=>e.text()).then(t=>console.log(e))}</script>\n<header>";
const char WEBPORTAL_FOOTER[] PROGMEM = "\n<footer>Version: <a style='font-size: small;background: none;text-decoration: underline;' target='_blank' href='https://github.com/" GIT_REPO "'>" BONO_GPS_VERSION "</a></footer>\n</body>\n</html>";
#ifdef TASK_SCHEDULER
const char WEBPORTAL_ROOT_OPTIONS[] PROGMEM = "\n</details>\n<details><summary>Device</summary>\n<article>Suspend GPS for <a href='/powersave/1800'>30'</a> <a href='/powersave/3600'>1 hr</a></article>\n<article>Disable <a href='/wifioff'>WiFi</a></article>\n<article><a href='/preset'>Load Preset</a></article>\n<article><a href='/savecfg'>Save config</a></article>\n<article><a href='/status'>Information</a></article>\n<article><a href='/savewificreds'>Set WiFi credentials</a></article>\n<article>Turn device and GPS <a href='/poweroff'>OFF</a></article>\n<article><a href='/restart'>Restart</a><br><br></article></details>";
#else
const char WEBPORTAL_ROOT_OPTIONS[] PROGMEM = "\n</details>\n<details><summary>Device</summary>\n<article>Disable <a href='/wifioff'>WiFi</a></article>\n<article><a href='/preset'>Load Preset</a></article>\n<article><a href='/savecfg'>Save config</a></article>\n<article><a href='/status'>Information</a></article>\n<article><a href='/savewificreds'>Set WiFi credentials</a></article>\n<article><a href='/restart'>Restart</a><br><br></article></details>";
#endif
const char WEBPORTAL_OPTION_CHECKED[] PROGMEM = "' checked>";
const char WEBPORTAL_OPTION_LABELCLASSBTN[] PROGMEM = "\n\t<label class='button' for='";
const char WEBPORTAL_OPTION_ARTICLECLASS[] PROGMEM = "\n<article class='bg'>";
const char WEBPORTAL_OPTION_SELECT_ONCHANGE[] PROGMEM = "onchange='Select(this.id)' name='";

#define WEBPORTAL_HEADER_STATIC_SIZE 350
#define WEBPORTAL_HEADER_DYN_SIZE 150
#define WEBPORTAL_FOOTER_STATIC_SIZE 210

// Helpers to populate a page with style sheet
String generate_html_header(bool add_menu = true)
{
  String htmlbody((char *)0);
  htmlbody.reserve(WEBPORTAL_HEADER_DYN_SIZE);

  if (add_menu) {
    htmlbody +=  F("Back to <a href='/'>Main menu</a>") ;
  } else {
    htmlbody += String(ap_ssid);
    #ifdef SHOWBATTERY
    htmlbody += "&nbsp;\n<div inline class=\"bat\">\n<div class=\"lvl\" style=\"width: ";
    htmlbody += String(+LiPoChargePercentage(ReadBatteryVoltage()));
    htmlbody += "%;\"></div></div>";
    #endif
  }

  if (gps_powersave)
    htmlbody += F("<br><em>Powersave mode</em>");
  htmlbody += F("</header>\n");
  return htmlbody;
}
String generate_html_body(String input, bool add_menu = true)
{
  // assumption: ap_ssid is populated
  String htmlbody((char *)0);
  htmlbody.reserve(WEBPORTAL_HEADER_STATIC_SIZE + WEBPORTAL_HEADER_DYN_SIZE + WEBPORTAL_FOOTER_STATIC_SIZE + input.length());
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

String html_select(String divlabel, String parameters[], String parameters_names[], const bool parameters_status[], String groupname, int numberparams)
{
  String message((char *)0);
  message.reserve(500);
  message += String(WEBPORTAL_OPTION_ARTICLECLASS);
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
  webserver.sendHeader("Cache-Control", "max-age=86400");
  webserver.send_P(200, html_css, WEBPORTAL_CSS);
}
void handle_menu()
{
  // variables used to build html buttons
  String sv_values[2] = {"8", "all"};
  String sv_names[2] = {"8", "All"};
  const bool sv_status[2] = {false, true};

  String baud_values[3] = {"38400", "57600", "115200"};
  String baud_names[3] = {"38k4", "57k6", "115k2"};
  const bool baud_status[3] = {(stored_preferences.gps_baud_rate == 38400), (stored_preferences.gps_baud_rate == 57600), (stored_preferences.gps_baud_rate == 115200)};

  String wifi_values[2] = {"sta", "ap"};
  String wifi_names[2] = {"Client", "AP"};
  const bool wifi_status[2] = {(stored_preferences.wifi_mode == WIFI_STA), (stored_preferences.wifi_mode == WIFI_AP)};

#ifdef TASK_SCHEDULER
  String pollgsagsv_values[3] = {"0", "1", "5"};
  String pollgsagsv_names[3] = {"Off", "1 sec", "5 sec"};
  const bool pollgsagsv_status[3] = {(stored_preferences.nmeaGSAGSVpolling == 0), (stored_preferences.nmeaGSAGSVpolling == 1), (stored_preferences.nmeaGSAGSVpolling == 5)};
#endif

  log_i("Handle webserver root response");
  String mainpage((char *)0);
  // we reserve enough space to populate the main page
  mainpage.reserve(4500);
  mainpage += generate_html_header(false);
  mainpage += F("\n<details open><summary>GPS runtime settings</summary>\n<article class='bg'>Update\n<input type='radio' id='rate/1hz' ");
  
  mainpage += String(WEBPORTAL_OPTION_SELECT_ONCHANGE);
  mainpage += ((stored_preferences.gps_rate == 1) ? "rate' checked>" : "rate'> ");
  mainpage += String(WEBPORTAL_OPTION_LABELCLASSBTN);
  mainpage += F("rate/1hz'>1 Hz</label>\n<input type='radio' id='rate/5hz' ");
  
  mainpage += String(WEBPORTAL_OPTION_SELECT_ONCHANGE);
  mainpage += ((stored_preferences.gps_rate == 5) ? "rate' checked>" : "rate'> ");
  mainpage += String(WEBPORTAL_OPTION_LABELCLASSBTN);
  mainpage += F("rate/5hz'>5 Hz</label>\n<input type='radio' id='rate/10hz' ");
  
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
#if defined(PLATFORMIO)
  #define DOCUMENTATION_CONNECTING "https://github.com/" GIT_REPO "/../../../../bonogps#connecting-to-an-app"
#else
  #define DOCUMENTATION_CONNECTING "https://github.com/" GIT_REPO "#connecting-to-an-app"
#endif
  mainpage += F("<details open><summary>Setup and Configuration help</summary><p>Check the <a style='color: #0645AD;background: none;text-decoration: underline;' target='_blank' href='" DOCUMENTATION_CONNECTING "'>online documentation</a> for more details and visual guides</p></details>");
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

#ifdef TASK_SCHEDULER
void handle_powersave()
{
  String sttime = webserver.pathArg(0);
  log_i("Set power saving mode for %s sec", sttime);
  // disable polling of GSA and GSV
  int time = sttime.toInt();
  control_poll_GSA_GSV(0);
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
  // Enabling restart of GPS functions - we don't get here when poweroff is enabled
  trestart_after_sleep.setInterval(time * 1000);
  trestart_after_sleep.enableDelayed();
#ifdef SHORT_API
  webserver.send_P(200, text_json, json_ok);
#else
  webserver.send(200, html_text, generate_html_body(F("Enabled power saving mode for 3600 seconds")));
#endif
}
#endif

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

void handle_deepsleep()
{
  webserver.send(200, html_text, generate_html_body(F("Please confirm <form action='/poweroff' method='post'><input type='submit' value='Power Off'></form>")));
}

void handle_deepsleep_execute()
{
  log_i("Powering off");
  webserver.send(200, html_text, generate_html_body(F("Powering off GPS and device - use the Reset button to re-activate")));
  control_poll_GSA_GSV(0);
  // https://portal.u-blox.com/s/question/0D52p00008HKCQxCAP/shutting-down-neom8-indefinitely
  push_gps_message(UBLOX_PWR_OFF, sizeof(UBLOX_PWR_OFF));
  delay(1000);
  poweroff();
}

void handle_status()
{
  const char details_stop_start[] PROGMEM = "</article></details>\n<details";
  const char article_stop_start[] PROGMEM = "</article>\n\t<article>";

  log_d("Logging status to web");
  String message((char *)0);
  message.reserve(1400);
  message += F("\n<details open><summary>WiFi Mode: ");

  if (stored_preferences.wifi_mode == WIFI_AP)
  {
    message += F("AP</summary>\n\t<article>Access point name: ");
    message += ap_ssid;
  }
  else
  {
    message += F("STA</summary>\n\t<article>Connected to AP: ");
    message += WiFi.SSID();
  }

  message += article_stop_start;
  message += F("IP: ");
  message += MyIP.toString();
  message += details_stop_start;
  if (stored_preferences.nmeaTcpServer)
    message += F(" open");
  message += F("><summary>WiFi/TCP-IP Service: ");
  message += (stored_preferences.nmeaTcpServer ? F("ON") : F("OFF"));
  message += F("</summary><article>TCP/IP NMEA repeater on Port:");
  message += String(NMEAServerPort);
  message += article_stop_start;
  message += F("u-center URL: tcp://");
  message += MyIP.toString();
  message += String(":");
  message += String(NMEAServerPort);
#ifdef BLEENABLED
  message += details_stop_start;
  if (stored_preferences.ble_active)
    message += F(" open");
  message += F("><summary>BLE Service: ");
  message += (stored_preferences.ble_active ? F("ON") : F("OFF"));
  message += F("</summary>\n\t<article>BLE Name: ");
  message += ble_device_id;
  message += article_stop_start; message += F("BLE Service: ");
  message += String(BLE_SERVICE_UUID, HEX);
  message += F("<br>BLE Characteristic: ");
  message += String(BLE_CHARACTERISTIC_UUID, HEX);
#else
  message += details_stop_start;
  message += F("><summary>BLE n/a in this firmware</summary></details>");
#endif
#ifdef BTSPPENABLED
  message += details_stop_start;
  if (stored_preferences.btspp_active)
    message += F(" open");
  message += F("><summary>BT-SPP Service: ");
  message += (stored_preferences.btspp_active ? F("ON") : F("OFF"));
  message += F("</summary>\n\t<article>BT Name: ");
  message += ble_device_id;
#else
  message += details_stop_start;
  message += F("><summary>BT-SPP n/a in this firmware</summary></details>");
#endif
  message += details_stop_start;
  message += F(" open><summary>GPS Module</summary><article>");
  message += F("UART Baud: ");
  message += stored_preferences.gps_baud_rate;
  message += F("<br>Refresh rate: ");
  message += stored_preferences.gps_rate + String("Hz");
  message += F("<br>Main Talker ID: ");
  message += ((stored_preferences.trackaddict || stored_preferences.racechrono || stored_preferences.racetime) ? F("GPS ") : F("System Specific (default)"));
  message += F("<br>Restrict SV to GPS: ");
  message += (stored_preferences.racechrono ? F("Yes ") : F("No (default)"));
  message += article_stop_start;
  message += F("NMEA GxGSV: ");
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
  message += details_stop_start;
  message += F(" open><summary>BonoGPS Board info</summary><article>BonoGPS Version: ");
  message += String(BONO_GPS_VERSION);
  message += F("<br>Build date: ");
  message += String(BONOGPS_BUILD_DATE);
#ifdef ENABLE_OTA
  #ifdef TASK_SCHEDULER
  if (tOTA.isEnabled()) 
  {
    message += F("<br>OTA active");
  }
  else
  {
    message += F("<br>OTA built-in but inactive, restart to activate");
  }
  #else
  message += F("<br>OTA Built-in");
  #endif
#else
  message += F("<br>OTA n/a in this firmware ");
#endif
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
  message += article_stop_start;
  message += F("Board: ");
  message += String(ARDUINO_BOARD);
  message += F("<br>Chip Revision: ");
  message += String(ESP.getChipRevision());
  message += F("<br>CPU Freq: ");
  message += String(ESP.getCpuFreqMHz());
#if defined(SHOWBATTERY)
  float voltage = ReadBatteryVoltage();
  message += F("<p>Battery Voltage: ");
  message += String(voltage);
  message += F("<br>Battery %: ");
  message += String(LiPoChargePercentage(voltage));
#endif
#ifdef UPTIME
  message += F("<br><br>Uptime: ");
  message += String(uptime_formatter::getUptime());
#endif
  message += F("</article></details>");

  webserver.send(200, html_text, generate_html_body(message));
}
void handle_clients()
{
  log_i("Logging clients status to web");
  String message((char *)0);
  message.reserve(1000);
  message += F("NMEA TCP/IP client ");
  if (NMEARemoteClient.connected())
  {
    message += NMEARemoteClient.remoteIP().toString().c_str();
  }
  else
  {
    message += "not ";
  }

#ifdef BLEENABLED
  message += F(" connected<br>BLE device ");
  if (ble_deviceConnected)
  {
    message += ble_client_address;
  }
  else
  {
    message += "not ";
  }
#endif
#ifdef BTSPPENABLED
  message += F(" connected<br>BT-SPP device ");
  if (!bt_deviceConnected)
  {
    message += "not ";
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
  // TODO check is storing value went fine
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
  message += F(" mode</a></p>\n<h1>Save only a specific WiFi mode</h1><p>Save <a href='/savewifi/off'>No WiFi (suggested on the field)</a><p>Save <a href='/savewifi/sta'>WiFi client mode WiFi-STA </a></p><p>Save <a href='/savewifi/ap'>WiFi Access Point mode WiFi-AP</a></p>");
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
  if (strwifimode == "sta") {
    StoreNVMPreferencesWiFi("WIFI_STA");
  } else if (strwifimode == "ap") {
    StoreNVMPreferencesWiFi("WIFI_AP");
  } else {
    StoreNVMPreferencesWiFi("WIFI_OFF");
  }
    
  String message = F("Stored preference for wifi: ");
  strwifimode.toUpperCase();
  message += strwifimode;
  webserver.send(200, html_text, generate_html_body(message));
}

void handle_NotFound()
{
  webserver.send(404, html_text, generate_html_body(F("<h1> 404 Not found</h1><p>The requested resource was not found on this server.</p>")));
}
void WebConfig_start()
{
  webserver.onNotFound(handle_NotFound);
  webserver.on("/", handle_menu);
  webserver.on("/css", handle_css);
  webserver.on(UriBraces("/rate/{}hz"), handle_rate);
  webserver.on(UriBraces("/gsa/{}"), handle_gsa);
  webserver.on(UriBraces("/gsv/{}"), handle_gsv);
  webserver.on(UriBraces("/gbs/{}"), handle_gbs);
  webserver.on(UriBraces("/sv/{}"), handle_svchannel);
  webserver.on(UriBraces("/tcpserver/{}"), handle_tcpserver);
  webserver.on(UriBraces("/hlt/{}"), handle_hlt);
  webserver.on(UriBraces("/wifi/{}"), handle_wifi_mode);
  webserver.on("/restart", HTTP_GET, handle_restart);
  webserver.on("/restart", HTTP_POST, handle_restart_execute);
  webserver.on("/poweroff", HTTP_GET, handle_deepsleep);
  webserver.on("/poweroff", HTTP_POST, handle_deepsleep_execute);
  webserver.on("/wifioff", HTTP_GET, handle_wifioff);
  webserver.on("/wifioff", HTTP_POST, handle_wifioff_post);
  webserver.on("/savecfg", handle_saveconfig);
  webserver.on(UriBraces("/savecfg/{}"), handle_saveconfig_wifi);
  webserver.on(UriBraces("/savewifi/{}"), handle_saveconfig_wifimode);
  webserver.on("/savewificreds", HTTP_GET, handle_saveconfig_wifi_creds);
  webserver.on("/savewificreds", HTTP_POST, handle_saveconfig_wifi_creds_post);
  webserver.on("/preset", handle_preset);
  webserver.on("/status", handle_status);
  webserver.on("/clients", handle_clients);
  webserver.on(UriBraces("/baud/{}"), handle_baudrate);
#ifdef BTSPPENABLED
  webserver.on(UriBraces("/btspp/{}"), handle_btspp);
  webserver.on(UriBraces("/trackaddict/{}"), handle_trackaddict);
  webserver.on("/racechrono/android", handle_racechrono_android);
  webserver.on("/racetime/android", handle_racetime_android);
#endif
#ifdef BLEENABLED
  webserver.on(UriBraces("/ble/{}"), handle_ble);
#endif
#ifdef TASK_SCHEDULER
  webserver.on(UriBraces("/powersave/{}"), handle_powersave);
  webserver.on(UriBraces("/poll/gsagsv/{}"), handle_pollgsagsv);
#endif
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
#ifndef OTA_AVAILABILITY_SECS
#define OTA_AVAILABILITY_SECS 300 // 300 seconds of OTA running
#endif

#ifdef TASK_SCHEDULER
void handle_OTA()
{
  ArduinoOTA.handle();
}
#endif

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
        #ifdef BLEENABLED
        if (stored_preferences.ble_active)
        {
          ble_stop();
        }
        #endif
        #ifdef BTSPPENABLED
        if (stored_preferences.btspp_active)
        {
          bt_spp_stop();
        }
        #endif
        if (stored_preferences.nmeaTcpServer) 
        {
          stop_NMEA_server();
        }
        #ifdef TASK_SCHEDULER
        tLedWiFiBlink.setInterval(50);
        #endif
        log_i("Start updating %s", type);
      })
      .onEnd([]() {
        log_i("\nEnd");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
         log_i("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        log_e("Error[ %u]: ", error);
        if (error == OTA_AUTH_ERROR)
          log_e("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
          log_e("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
          log_e("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
          log_e("Receive Failed");
        else if (error == OTA_END_ERROR)
          log_e("End Failed");
      });
  ArduinoOTA.begin();
  #ifdef TASK_SCHEDULER
  tOTA.set(1000, OTA_AVAILABILITY_SECS, &handle_OTA);
  ts.addTask(tOTA);
  tOTA.enable();
  #endif
}

void OTA_stop()
{
  ArduinoOTA.end();
  #ifdef TASK_SCHEDULER
  tOTA.disable();
  #endif
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

#define BAT_LVL_SERVICE_UUID (uint16_t)0x180F       // used when SHOWBATTERY enabled
#define BAT_LVL_CHAR_UUID (uint16_t)0x2A19       // used when SHOWBATTERY enabled
#define BATTERY_NOTIFICATION_PERIOD_MS 60000 // battery updated once a minute

// write commands to the device
#define BLE_RECONFIG_CHARACTERISTIC_UUID (uint16_t)0x2A05

// #define HardwareRev "0001"    //0x2A27 utf8s
// #define SystemID "000001"     //0x2A23 uint40

NimBLEServer *pServer = NULL;
NimBLECharacteristic *pCharacteristicGPS = NULL;
NimBLECharacteristic *pCharacteristicCommand = NULL;
#if defined(SHOWBATTERY)
NimBLECharacteristic *pCharBattery = NULL;

#if defined(TASK_SCHEDULER)
// Create the period task to notify BLE of battery status
void BLEBatteryNotify() {
  if (pCharBattery) {
    pCharBattery->setValue(LiPoChargePercentage(ReadBatteryVoltage()));
    pCharBattery->notify();
  }
}
Task tBLEBatteryNotify(BATTERY_NOTIFICATION_PERIOD_MS, TASK_FOREVER, BLEBatteryNotify, &ts, false);
#endif //#if defined(TASK_SCHEDULER)

#endif
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
    ble_client_address = NimBLEAddress(desc->peer_id_addr).toString().c_str();
    log_i("BLE Client address: %s", ble_client_address.c_str());

    #if defined(SHOWBATTERY) && defined(TASK_SCHEDULER)
    tBLEBatteryNotify.enable();
    #endif

  };
  void onDisconnect(NimBLEServer *pServer)
  {
    ble_deviceConnected = false;
    ble_client_address = "";

    #if defined(SHOWBATTERY) && defined(TASK_SCHEDULER)
    tBLEBatteryNotify.disable();
    #endif
  }
};

class MyCharacteristicCallbacks : public NimBLECharacteristicCallbacks
{
  void onWrite(NimBLECharacteristic *pCharacteristic,ble_gap_conn_desc * 	desc ) {
    log_i("Characteristic %s, written value: %d",pCharacteristic->getUUID().toString().c_str(), pCharacteristic->getValue().c_str());
    if (pCharacteristic->getUUID() == pCharacteristicCommand->getUUID())
    {
      String command = String(pCharacteristic->getUUID().toString().c_str());
      if (command.equalsIgnoreCase("reboot")) {
        log_e("reboot");
        //TODO: this should be handled gently, not simply stop and reboot 
        NimBLEDevice::getServer()->disconnect(desc->conn_handle);
        ESP.restart();  
      } else if (command.equalsIgnoreCase("wifista"))
      {
        wifi_STA();
      }
    }
  }
  void onWrite(NimBLECharacteristic *pCharacteristic) {
    log_i("Characteristic %s, written value: %d",pCharacteristic->getUUID().toString().c_str(), pCharacteristic->getValue().c_str());
    if (pCharacteristic->getUUID() == pCharacteristicCommand->getUUID())
    {
      String command = String(pCharacteristic->getUUID().toString().c_str());
      if (command.equalsIgnoreCase("reboot")) {
        log_e("reboot");
        //TODO: this should be handled gently, not simply stop and reboot 
        ESP.restart();  
      } else if (command.equalsIgnoreCase("wifista"))
      {
        wifi_STA();
      }
    }
  }
};
static MyCharacteristicCallbacks chrCommandCallbacks;

void ble_start()
{
#ifdef BTSPPENABLED
  bt_spp_stop();
#endif
  /// Create the BLE Device
  NimBLEDevice::init(ble_device_id);
  // NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_RANDOM);
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
  // Create the BLE Service for GPS - Service and Location
  NimBLEService *pServiceGPS = pServer->createService(BLE_SERVICE_UUID);
  // Create a BLE Characteristic for the GPS - Speed and Location characteristic
  pCharacteristicGPS = pServiceGPS->createCharacteristic(BLE_CHARACTERISTIC_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);  
  // Start the service
  pServiceGPS->start();
  // Start advertising
  NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(pServiceGPS->getUUID());

  // Device info service
  NimBLEService *pServiceInfo = pServer->createService(DEVINFO_UUID);
  NimBLECharacteristic *pChar = pServiceInfo->createCharacteristic(DEVINFO_MANUFACTURER_UUID, NIMBLE_PROPERTY::READ);
  pChar->setValue(DEVICE_MANUFACTURER);
  pChar = pServiceInfo->createCharacteristic(DEVINFO_NAME_UUID, NIMBLE_PROPERTY::READ);
  pChar->setValue(DEVICE_NAME);
  pChar = pServiceInfo->createCharacteristic(DEVINFO_SERIAL_UUID, NIMBLE_PROPERTY::READ);
  char serialnumber[8];
  sprintf(serialnumber,"GPS%04X",chip);
  pChar->setValue(serialnumber);
  pChar = pServiceInfo->createCharacteristic(DEVINFO_FWREV_UUID, NIMBLE_PROPERTY::READ);
  pChar->setValue(BONO_GPS_VERSION);
  // Add a write characteristic to handle commands via BLE - reserved for commands "reboot" and "wifista"
  pCharacteristicCommand = pServiceInfo->createCharacteristic(BLE_RECONFIG_CHARACTERISTIC_UUID, NIMBLE_PROPERTY::WRITE_NR);
  pCharacteristicCommand->setCallbacks(&chrCommandCallbacks);
  pServiceInfo->start();
  pAdvertising->addServiceUUID(pServiceInfo->getUUID());

#if defined(SHOWBATTERY)
  NimBLEService *pServiceBattery = pServer->createService(BAT_LVL_SERVICE_UUID);
  if (pServiceBattery)
  {
    log_d("BLE pServiceBattery created");
    
    #ifdef TASK_SCHEDULER
    pCharBattery = pServiceBattery->createCharacteristic(BAT_LVL_CHAR_UUID,NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    #else
    pCharBattery = pServiceBattery->createCharacteristic(BAT_LVL_CHAR_UUID,NIMBLE_PROPERTY::READ);
    #endif
    if (pCharBattery)
    {
      pServiceBattery->start();
      uint8_t batterypct=LiPoChargePercentage(ReadBatteryVoltage());
      pCharBattery->setValue(batterypct);
      log_d("BLE pCharBattery value set to: %d", batterypct);
      pAdvertising->addServiceUUID(pServiceBattery->getUUID());
    }
    else
    {
      log_e("BLE pCharBattery NOT CREATED");
    }
    
  }
  else
  {
    log_e("BLE pServiceBattery NOT created");
  }

#endif

  // define appearance, from https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.gap.appearance.xml
  pAdvertising->setAppearance(5186);
  /** If your device is battery powered you may consider setting scan response to false as it will extend battery life at the expense of less data sent.  */
  pAdvertising->setScanResponse(false);
  // pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue -> find out more at https://github.com/h2zero/NimBLE-Arduino/issues/129
  // pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter

  pAdvertising->start();
  log_i("Waiting a client connection to notify...");
}

void ble_stop()
{
  if (NimBLEDevice::getInitialized())
  {
    NimBLEDevice::stopAdvertising();
    NimBLEDevice::deinit(true);
    #if defined(SHOWBATTERY) && defined(TASK_SCHEDULER)

    #endif
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
  log_d("Send a ping to start the GPS if it was powersaved");
  // BAUD matters almost nothing, we just need some data on UART
  gpsPort.begin(GPS_STANDARD_BAUD_RATE, SERIAL_8N1, RX2, TX2);
  push_gps_message(UBLOX_WARMSTART, sizeof(UBLOX_WARMSTART));
  gpsPort.flush();
  delay(250);
  gpsPort.end();
  delay(250);
  // if preferences have a value <> than the one stored in the GPS, connect+switch
  log_d("Start UART connection on RX pin %d TX pin %d and autobaudrate", RX2, TX2);
  gpsPort.begin(0, SERIAL_8N1, RX2, TX2, false, GPS_UART_TIMEOUT);
  if (gpsPort.baudRate() > 0)
  {
    log_d("Connected with autobaudrate at %u on RX pin %d TX pin %d ", gpsPort.baudRate(), RX2, TX2);
  }
  else
  {
    log_e("Can't auto find BAUD rate on RX pin %d TX pin %d , forcing %u", RX2, TX2, GPS_STANDARD_BAUD_RATE);
    // TODO: enable pulsing error on the LED to signal the user that something is bad
    gpsPort.begin(GPS_STANDARD_BAUD_RATE, SERIAL_8N1, RX2, TX2);
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
  }
  else
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
  pinMode(LED_WIFI, OUTPUT);
  #ifdef TASK_SCHEDULER
  tLedWiFiBlink.setInterval(100);
  tLedWiFiBlink.enable();
  #endif

  #ifdef LED_ACTIVE_EXTERNAL
  // Use LED_ACTIVE_EXTERNAL to show status of device
  pinMode(LED_ACTIVE_EXTERNAL, OUTPUT);
  #ifdef TASK_SCHEDULER
  // configure LED PWM functionalitites
  ledcSetup(LEDCHANNEL, LEDPWMFREQ, LEDPWMRESOLUTION);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(LED_ACTIVE_EXTERNAL, LEDCHANNEL);
  tLedActiveBlink.setInterval(1000);
  tLedActiveBlink.enable();
  #endif
  #endif
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
  button.onPressedFor(WIFI_ENABLE_STA_DURATION,wifi_STA);
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
  default:
    wifi_OFF();
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
          gps_currentmessage[0] = (uint8_t)'$';
          gps_currentmessage[1] = (uint8_t)'\0';
          gps_message_pointer = 1;
        }
        if (sbuf[i] == 36) // $ char
        {
          gps_currentmessage[gps_message_pointer++] = (uint8_t)'\r';
          gps_currentmessage[gps_message_pointer++] = (uint8_t)'\n';
          gps_currentmessage[gps_message_pointer++] = (uint8_t)'\0';
          // There is a new message -> Notify BLE of the changed value if connected and size says it is likely to be valid
          if (gps_message_pointer > MIN_NMEA_MESSAGE_SIZE)
          {
            pCharacteristicGPS->setValue(gps_currentmessage, gps_message_pointer);
            pCharacteristicGPS->notify();
            delay(1);
          }
          gps_currentmessage[0] = (uint8_t)'$';
          gps_currentmessage[1] = (uint8_t)'\0';
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

    webserver.handleClient();
  }

#ifdef BUTTON
  // Continuously read the status of the button.
  button.read();
#endif

#ifdef TASK_SCHEDULER
  // run all periodic tasks
  ts.execute();
#else
  // if scheduler is not available, we need to check for OTA (if builtin)
  #ifdef ENABLE_OTA
  ArduinoOTA.handle();
  #endif // #ifdef ENABLE_OTA
#endif // #ifdef TASK_SCHEDULER
}
