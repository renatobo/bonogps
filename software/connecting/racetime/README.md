# Racetime

- [Setup](#setup)
- [GPS Configuration for NMEA messages](#gps-configuration-for-nmea-messages)

Latest supported app: [https://www.racetimeapp.com/en/](https://www.racetimeapp.com/en/)

- tested with v3.3.8 lite
- BT-SPP is the only tested option. ESP32-S3 boards (e.g. LilyGO T-Energy-S3) have no BT-SPP; the firmware has a WiFi (TCP-IP) preset for them, but it hasn't been tested with RaceTime
- `GPGGA`, `GPRMC`, `GPVTG`, `GPGLL` enabled, with main talker id GP

## Setup

Open the app, an icon to connect an external bluetooth adapter is displayed

![Setup](racetime_setup.png)

From the connection page, search your device

![Connecting](racetime_connecting.png)

When activated, the satellite icon lets you check the connection to the device

![Actiated](racetime_activated.png)

The location icon lets you check the quality of signal, showing a red exclamation point if something is not in place correctly

![Signal](racetime_signal.png)

## GPS Configuration for NMEA messages

According to information shared via email with the developer Roberto Morini, RaceTime supports

- `GPGSA`
- `GPGGA`
- `GPRMC`
- `GPVTG`
- `GPGLL`

which means we have to enable the _Main Talker id = GP_ option.
As there is no display of satellite positions, `GPGSA` is not enabled.