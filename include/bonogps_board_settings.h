/******************************************************************************

  BonoGPS: connect a GPS to mobile Apps to track lap times
  More info at https://github.com/renatobo/bonogps
  Renato Bonomini https://github.com/renatobo

  This file contains pinout definitions for a few tested boards:
  - https://docs.platformio.org/en/latest/boards/espressif32/esp32doit-devkit-v1.html
  - https://docs.platformio.org/en/latest/boards/espressif32/lolin_d32_pro.html

  If you have an undefined board, you need to
  - identify the LED_BUILTIN pin: this is usually the builtin blue led, used to signal WiFi status
  - identify the WIFI_MODE_BUTTON pin: this is BOOT for the DOIT-DevKit as it's already there, undefined for the LOLIN
  - identify UART2 PINs RX2 TX2, if not already defined by your board

******************************************************************************/

#if defined(ARDUINO_LOLIN_D32_PRO)

#define BOARD_NAME "Lolin D32 PRO" // Display in information page
// Where is Serial2 connected - for Lolin we reconfigure to 2 free pins
// until https://github.com/espressif/arduino-esp32/pull/4520 is in place, we need to manually define it 
#define RX2 GPIO_NUM_4 // 12
#define TX2 GPIO_NUM_2 // 14
// Which pin controls the button to switch to STA mode
// You need to attach a button to this pin
#define WIFI_MODE_BUTTON GPIO_NUM_25

#ifndef LED_BUILTIN
#define LED_BUILTIN 5
#define GPIO_BATTERY GPIO_NUM_35 // Read Battery status from PIN 25
#define SHOWBATTERY // Show the battery charge indicator on the top menu of the web configuration panel
#endif

#elif defined(ARDUINO_ESP32_DEV)

#define BOARD_NAME "esp32-devkit" // Display in information page
#define RX2 16 // Standard label Rx2 on board
#define TX2 17 // Standard label Tx2 on board
#define WIFI_MODE_BUTTON 0 // default is: use the boot button to switch wifi modes
#define LED_BUILTIN 2 // this should not be needed if you choose the right board

#else

#define BOARD_NAME "esp32-custom" // Display in information page
// #define RX2 GPIO_NUM_17 // Serial2 standard location on devkit
// #define TX2 GPIO_NUM_16 // Serial2 standard location on devkit
#define LED_BUILTIN 2 // usually 2 or 5
#define WIFI_MODE_BUTTON 0 // default is: use the boot button to switch wifi modes

#endif