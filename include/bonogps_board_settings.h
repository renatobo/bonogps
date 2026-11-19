/******************************************************************************

  BonoGPS: connect a GPS to mobile Apps to track lap times
  More info at https://github.com/renatobo/bonogps
  Renato Bonomini https://github.com/renatobo

  This file contains pinout definitions for a few tested boards:
  - https://docs.platformio.org/en/latest/boards/espressif32/esp32doit-devkit-v1.html
  - https://docs.platformio.org/en/latest/boards/espressif32/lolin_d32_pro.html
  - https://github.com/Xinyuan-LilyGO/T-Energy-S3 (built with -DLILYGO_T_ENERGY_S3, see platformio.ini)

  If you have an undefined board, you need to
  - identify the LED_BUILTIN pin: this is usually the builtin blue led, used to signal WiFi status
  - identify the WIFI_MODE_BUTTON pin: this is BOOT for the DOIT-DevKit as it's already there, undefined for the LOLIN
  - identify UART2 PINs RX2 TX2, if not already defined by your board

******************************************************************************/

#if defined(ARDUINO_LOLIN_D32_PRO)

// Where is Serial2 connected - for Lolin we reconfigure to 2 free pins
// until https://github.com/espressif/arduino-esp32/pull/4520 is in place, we need to manually define it 
#undef RX2
#undef TX2
#define RX2 GPIO_NUM_4 // 12
#define TX2 GPIO_NUM_2 // 14
// Which pin controls the button to switch to STA mode
// You need to attach a button to this pin
#define WIFI_MODE_BUTTON GPIO_NUM_25
// Use external led on GPIO_NUM_12 to show signs of life
// #define LED_ACTIVE_EXTERNAL GPIO_NUM_12

// Use external led on GPIO_NUM_14 to show status of WiFi
// #define LED_WIFI GPIO_NUM_14
#ifndef LED_BUILTIN
#define LED_BUILTIN 5
#endif
#define GPIO_BATTERY GPIO_NUM_35 // Read Battery status from PIN 25
#define SHOWBATTERY // Show the battery charge indicator on the top menu of the web configuration panel

#elif defined(LILYGO_T_ENERGY_S3)

// BK-880 GPS wired to the UART0 pins, which are free for reuse as Serial2
// because Serial/flashing uses the ESP32-S3's native USB CDC instead
// (see ARDUINO_USB_CDC_ON_BOOT=1 in platformio.ini)
#undef RX2
#undef TX2
#define RX2 GPIO_NUM_44 // BK-880 TX -> here
#define TX2 GPIO_NUM_43 // BK-880 RX <- here
// BOOT-0 button doubles as the WiFi mode switch, same as the other boards
#define WIFI_MODE_BUTTON GPIO_NUM_0
// There is no onboard status LED on the T-Energy-S3: this is an unconnected
// placeholder pin required by the firmware. Wire an external LED (with a
// resistor) to it, or point this at another free GPIO, for visual WiFi status
#undef LED_BUILTIN
#define LED_BUILTIN GPIO_NUM_2
// Battery voltage divider output, labeled BATTERY_ADC_DATA / IO3 in LilyGo's docs
#define GPIO_BATTERY GPIO_NUM_3
// SHOWBATTERY is left off by default: the 7.445 scale factor in
// ReadBatteryVoltage() (src/bonogps.cpp) was calibrated for the Lolin D32
// Pro's divider and hasn't been verified against the T-Energy-S3 schematic.
// Confirm/adjust the scale factor for this board's divider before enabling.
// #define SHOWBATTERY

#elif defined(ARDUINO_ESP32_DEV)
#undef RX2
#undef TX2
#define RX2 16 // Standard label Rx2 on board
#define TX2 17 // Standard label Tx2 on board
#define WIFI_MODE_BUTTON 0 // default is: use the boot button to switch wifi modes
#define LED_BUILTIN 2 // this should not be needed if you choose the right board

#else

// #define RX2 GPIO_NUM_17 // Serial2 standard location on devkit
// #define TX2 GPIO_NUM_16 // Serial2 standard location on devkit
#define LED_BUILTIN 2 // usually 2 or 5
#define WIFI_MODE_BUTTON 0 // default is: use the boot button to switch wifi modes

#endif