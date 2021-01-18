# LOLIN D32 PRO 

- [No BOOT Button](#no-boot-button)
- [Serial2](#serial2)
- [Battery monitoring](#battery-monitoring)

## No BOOT Button

You need to supply an external temporary button to enable the WiFi switching option that the builtin BOOT button provides on the DevKit board.

Its pin is defined in `include/bonogps_board_settings.h` as  `WIFI_MODE_BUTTON`

## Serial2

This board does not have dedicated definitions for Serial2, so you need to pick a couple of PINs and assign them to the TX/RX function.

These pins need to be defined in `include/bonogps_board_settings.h` as `RX2` and `TX2`

## Battery monitoring

`GPIO_35`  can be used to read battery voltage (check this [tutorial](https://www.youtube.com/watch?t=88&v=yZjpYmWVLh8&feature=youtu.be)).

The plan is to replicate the functionality of *battery fuel gauge* available on the [Lipo Rider Plus](https://wiki.seeedstudio.com/Lipo-Rider-Plus/https://wiki.seeedstudio.com/Lipo-Rider-Plus/).

This is not implemented yet.

This is the simple function used to collect voltage

```C
analogRead(GPIO_BATTERY) / 4096.0 * 7.445
```
